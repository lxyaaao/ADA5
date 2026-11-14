import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile

import pytest
import numpy as np


def packager():
    path = Path(__file__).resolve().parents[1] / 'scripts' / 'package_delivery.py'
    # The old script builds an archive at import time. Do not execute that
    # side effect while checking the new reusable packaging interface.
    tree = ast.parse(path.read_text(encoding='utf-8'))
    assert not any(isinstance(node, (ast.For, ast.With)) for node in tree.body), 'packager must not build an archive at import time'
    spec = importlib.util.spec_from_file_location('delivery_packager_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_dataset_bundle_uses_current_readme_and_excludes_old_results(tmp_path):
    module = packager()
    paths = {
        'README.md': 'old recording',
        'docs/fin301_readme.md': 'new fin301 recording',
        'docs/references.md': 'references',
        'docs/report_notes_zh.md': 'old report',
        'outputs/result/map.pcd': 'old point cloud',
        'outputs/fin301/result/map.pcd': 'new point cloud',
        'outputs/fin301/cache/scans.npz': 'raw measurements',
        'outputs/fin301/audit/audit.json': '{}',
        'outputs/fin301/audit/recorded_baseline_UNVERIFIED.pgm': 'baseline',
        'indoor_mapping/geometry.py': 'code',
        'indoor_mapping/__pycache__/geometry.pyc': 'cache',
    }
    for name, content in paths.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    files = module.collect_files(tmp_path, 'fin301')
    assert files['README.md'] == tmp_path / 'docs/fin301_readme.md'
    assert 'outputs/fin301/result/map.pcd' in files
    assert 'outputs/fin301/audit/audit.json' in files
    assert 'indoor_mapping/geometry.py' in files
    assert 'outputs/result/map.pcd' not in files
    assert 'docs/report_notes_zh.md' not in files
    assert not any('cache' in name or 'baseline' in name for name in files)


def test_archive_manifest_detects_content_changed_after_hashing(tmp_path):
    module = packager()
    source = tmp_path / 'sample.txt'
    source.write_text('verified content')
    archive = tmp_path / 'delivery.zip'
    module.write_archive({'sample.txt': source}, archive)
    assert module.verify_archive(archive)['files'] == 1
    with zipfile.ZipFile(archive) as reader:
        manifest = json.loads(reader.read('DELIVERY_MANIFEST.json'))
    with zipfile.ZipFile(archive, 'w') as writer:
        writer.writestr('sample.txt', 'changed content')
        writer.writestr('DELIVERY_MANIFEST.json', json.dumps(manifest))
    with pytest.raises(ValueError, match='hash|size'):
        module.verify_archive(archive)


def verifier():
    path = Path(__file__).resolve().parents[1] / 'scripts' / 'verify_delivery.py'
    spec = importlib.util.spec_from_file_location('delivery_verifier_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_point_cloud_check_uses_current_counts_and_requires_planar_metric_map():
    module = verifier()
    points = np.array([[1, 2, 0], [3, 4, 0], [5, 6, 0]], dtype=float)
    metadata = {'points': 3, 'unit': 'metre', 'frame_id': 'map', 'fields': ['x', 'y', 'z']}
    assert module.check_point_cloud(points, {'pcd_points': 3}, metadata)['pcd_points'] == 3
    with pytest.raises(ValueError, match='count'):
        module.check_point_cloud(points, {'pcd_points': 4}, metadata)
    points[0, 2] = 1.0
    with pytest.raises(ValueError, match='plane'):
        module.check_point_cloud(points, {'pcd_points': 3}, metadata)


def test_mixed_map_keeps_planar_lidar_prefix_and_estimated_visual_height():
    module = verifier()
    points = np.array([[1, 2, 0], [3, 4, 0], [5, 6, -.7]], dtype=float)
    metadata = {'points': 3, 'unit': 'metre', 'frame_id': 'map', 'fields': ['x', 'y', 'z'],
                'geometry': 'lidar_scan_plane_plus_triangulated_visual_landmarks',
                'lidar_point_count': 2, 'visual_point_count': 1}
    metrics = {'pcd_points': 3, 'lidar_point_count': 2, 'visual_point_count': 1}
    assert module.check_point_cloud(points, metrics, metadata)['visual_point_count'] == 1
    points[0, 2] = .2
    with pytest.raises(ValueError, match='plane'):
        module.check_point_cloud(points, metrics, metadata)


def test_root_readme_links_are_rebased_without_changing_docs_copy(tmp_path):
    module = packager()
    source = tmp_path / 'docs/fin301_readme.md'
    source.parent.mkdir()
    source.write_text('[Map](../outputs/fin301/result/map.pcd)')
    archive = tmp_path / 'readme.zip'
    module.write_archive({'README.md': source, 'docs/fin301_readme.md': source}, archive)
    with zipfile.ZipFile(archive) as reader:
        assert reader.read('README.md').decode() == '[Map](outputs/fin301/result/map.pcd)'
        assert reader.read('docs/fin301_readme.md').decode() == source.read_text()


def test_packaging_rejects_verification_copied_from_another_dataset(tmp_path):
    module = packager()
    marker = tmp_path / 'marker.txt'
    marker.write_text('unchanged')
    checks = tmp_path / 'outputs/fin301/verification/artifact_checks.json'
    checks.parent.mkdir(parents=True)
    checks.write_text(json.dumps({'passed': True, 'dataset': 'legacy',
        'checked_file_sha256': {'marker.txt': hashlib.sha256(marker.read_bytes()).hexdigest()}}))
    with pytest.raises(ValueError, match='dataset'):
        module.build_archive(tmp_path, 'fin301')


def test_report_source_hash_check_detects_stale_results(tmp_path):
    module = verifier()
    relative = 'outputs/fin301/result/metrics.json'
    path = tmp_path / relative
    path.parent.mkdir(parents=True)
    path.write_text('{"pcd_points": 3}')
    hashes = {relative.replace('/', '\\'): hashlib.sha256(path.read_bytes()).hexdigest()}
    assert module.check_hashes(tmp_path, hashes, required_prefix=Path('outputs/fin301'))
    path.write_text('{"pcd_points": 4}')
    with pytest.raises(ValueError, match='hash'):
        module.check_hashes(tmp_path, hashes, required_prefix=Path('outputs/fin301'))


def test_report_manifest_may_hash_its_builder_without_accepting_old_dataset(tmp_path):
    module = verifier()
    builder = tmp_path / 'scripts/build_report.py'
    builder.parent.mkdir()
    builder.write_text('# report builder')
    hashes = {'scripts/build_report.py': hashlib.sha256(builder.read_bytes()).hexdigest()}
    options = {'required_prefix': Path('outputs/fin301'), 'allowed_paths': [Path('scripts/build_report.py')]}
    assert module.check_hashes(tmp_path, hashes, **options)
    old = tmp_path / 'outputs/result/metrics.json'
    old.parent.mkdir(parents=True)
    old.write_text('{}')
    hashes['outputs/result/metrics.json'] = hashlib.sha256(old.read_bytes()).hexdigest()
    with pytest.raises(ValueError, match='different dataset'):
        module.check_hashes(tmp_path, hashes, **options)


def test_visual_evidence_layout_uses_configured_six_views_and_checks_every_error():
    module = verifier()
    row = {'timestamps_ns': json.dumps([1793312145547223432 + i * 300_000_000 for i in range(6)]),
           'image_ids': json.dumps(list(range(6))), 'pixels_uv': json.dumps([[100., 120.]] * 6),
           'reprojection_px_all_views': json.dumps([.3, .4, .5, .6, .7, .8]),
           'max_reprojection_px': '.8'}
    stamps, ids, pixels, errors = module.parse_visual_track_evidence(row, 6, 2.)
    assert len(stamps) == len(ids) == len(errors) == 6 and pixels.shape == (6, 2)
    truncated = dict(row, pixels_uv=json.dumps([[100., 120.]] * 4))
    with pytest.raises(ValueError, match='view'):
        module.parse_visual_track_evidence(truncated, 6, 2.)
    inconsistent = dict(row, reprojection_px_all_views=json.dumps([.3, .4, .5, .6, .7, 2.1]))
    with pytest.raises(ValueError, match='reprojection'):
        module.parse_visual_track_evidence(inconsistent, 6, 2.)
