import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/prepare_local_validation.py'


def api():
    assert SCRIPT.is_file(), 'The portable validation preparation helper is missing'
    spec = importlib.util.spec_from_file_location('prepare_local_validation', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha(data):
    return hashlib.sha256(data).hexdigest()


@pytest.fixture
def archive(tmp_path):
    root = tmp_path / 'relocated project'
    root.mkdir()
    contents = {
        'scan_cache': ('outputs/fin301/cache/scans.npz', b'exact sensor cache'),
        'protocol': ('outputs/fin301/verification/protocol.json', b'{"frozen":true}\r\n'),
        'pcd_original': ('outputs/fin301/result/lidar_scan_map.pcd', b'original points\r\n'),
        'registration': ('indoor_mapping/geometry.py', b'# retained CRLF bytes\r\n'),
    }
    for relative, data in contents.values():
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    paths = {key: relative.replace('/', '\\') for key, (relative, _) in contents.items()}
    paths['registration'] = r'C:\Users\old-user\Downloads\AutoDrivingProject2.2\indoor_mapping\geometry.py'
    hashes = {key: sha(data) for key, (_, data) in contents.items()}
    provenance = {'scope': 'full-map repeat-observation stability', 'source_paths': paths,
                  'source_sha256': hashes, 'sources_unchanged_after_evaluation': True,
                  'quality_gates': {'minimum_overlap': .5},
                  'training_separation': {'query_count': 100}}
    saved = {'scope': provenance['scope'], 'protocol_sha256': hashes['protocol'],
             'pcd_sha256': hashes['pcd_original'], 'quality_gates': provenance['quality_gates'],
             'pcd_path': paths['pcd_original'], 'queries': [{'scan_index': 17}]}
    parent = root / 'outputs/fin301/temporal_filter/validation'
    parent.mkdir(parents=True)
    prov_path = parent / 'provenance.json'
    result_path = parent / 'original/localization_evaluation.json'
    result_path.parent.mkdir()
    prov_path.write_text(json.dumps(provenance, indent=2) + '\n', encoding='utf-8')
    result_path.write_bytes((json.dumps(saved, indent=2) + '\r\n').encode())
    return root, prov_path, result_path, contents


def test_relocation_creates_usable_local_copy_without_rewriting_history(archive, tmp_path):
    root, prov, original, contents = archive
    before = {path: path.read_bytes() for path in root.rglob('*') if path.is_file()}
    external = tmp_path / 'external scans.npz'
    external.write_bytes(contents['scan_cache'][1])
    output = root / 'local_validation'
    outcome = api().prepare_validation(root, prov, original, output, scan_cache=external)
    local = json.loads((output / 'provenance.json').read_text())
    assert outcome['validated_source_count'] == 4
    assert Path(local['source_paths']['registration']) == root / 'indoor_mapping/geometry.py'
    assert Path(local['source_paths']['protocol']) == root / 'outputs/fin301/verification/protocol.json'
    assert Path(local['source_paths']['scan_cache']) == external
    assert local['source_sha256'] == json.loads(before[prov])['source_sha256']
    assert local['local_path_preparation']['archived_provenance_sha256'] == sha(before[prov])
    assert local['local_path_preparation']['original_result_sha256'] == sha(before[original])
    assert local['local_path_preparation']['source_path_mapping']['registration']['archived_path'].startswith('C:')
    assert (output / 'original/localization_evaluation.json').read_bytes() == before[original]
    assert all(path.read_bytes() == payload for path, payload in before.items())
    # Running the same preparation again must be safe and byte-identical.
    first_local = (output / 'provenance.json').read_bytes()
    api().prepare_validation(root, prov, original, output, scan_cache=external)
    assert (output / 'provenance.json').read_bytes() == first_local


def test_check_only_validates_every_source_without_creating_output(archive):
    root, prov, original, _ = archive
    output = root / 'never-created'
    outcome = api().prepare_validation(root, prov, original, output, check_only=True)
    assert outcome['validated_source_count'] == 4
    assert outcome['check_only'] is True
    assert not output.exists()


def test_changed_frozen_source_fails_before_writing_any_output(archive):
    root, prov, original, _ = archive
    (root / 'indoor_mapping/geometry.py').write_bytes(b'different algorithm')
    with pytest.raises(ValueError, match='SHA-256 mismatch.*registration'):
        api().prepare_validation(root, prov, original, root / 'local_validation')
    assert not (root / 'local_validation').exists()


def test_missing_cache_explains_dataset_extraction_or_external_override(archive):
    root, prov, original, _ = archive
    (root / 'outputs/fin301/cache/scans.npz').unlink()
    with pytest.raises(ValueError) as error:
        api().prepare_validation(root, prov, original, root / 'local_validation', check_only=True)
    assert 'audit_bag.py' in str(error.value)
    assert '--scan-cache' in str(error.value)
    assert not (root / 'local_validation').exists()


@pytest.mark.parametrize('bad_path', [r'..\outside.py', r'D:\unrelated\geometry.py', r'C:relative.py'])
def test_unrecognized_or_escaping_source_paths_are_rejected(archive, bad_path):
    root, prov, original, _ = archive
    data = json.loads(prov.read_text())
    data['source_paths']['registration'] = bad_path
    prov.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='[Pp]ath|[Aa]bsolute|traversal'):
        api().prepare_validation(root, prov, original, root / 'local_validation')
    assert not (root / 'local_validation').exists()


def test_output_cannot_overwrite_archived_provenance(archive):
    root, prov, original, _ = archive
    before = prov.read_bytes()
    with pytest.raises(ValueError, match='historical|archived|overwrite'):
        api().prepare_validation(root, prov, original, prov.parent)
    assert prov.read_bytes() == before


def test_original_result_must_match_frozen_protocol(archive):
    root, prov, original, _ = archive
    saved = json.loads(original.read_text())
    saved['protocol_sha256'] = '0' * 64
    original.write_text(json.dumps(saved))
    with pytest.raises(ValueError, match='[Pp]rotocol'):
        api().prepare_validation(root, prov, original, root / 'local_validation')
    assert not (root / 'local_validation').exists()


def test_refuses_to_replace_different_existing_local_output(archive):
    root, prov, original, _ = archive
    output = root / 'local_validation'
    output.mkdir()
    destination = output / 'provenance.json'
    destination.write_bytes(b'previous local experiment')
    with pytest.raises(ValueError, match='already exists|different bytes'):
        api().prepare_validation(root, prov, original, output)
    assert destination.read_bytes() == b'previous local experiment'
