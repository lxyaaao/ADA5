"""Validate archived controls and prepare a separate, relocated provenance copy.

Historical JSON and frozen evaluators are never rewritten. All source bytes must
match their archived SHA-256 values. This prepares inputs; it does not rerun or
independently confirm the archived localization experiment.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path, PureWindowsPath
import re

ROOT = Path(__file__).resolve().parents[1]
ARCHIVED_ROOT_NAME = 'AutoDrivingProject2.2'
SCOPE = 'full-map repeat-observation stability'


def file_hash(path: Path) -> str:
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def resolve_archived_path(value: str, project_root: Path) -> Path:
    """Translate recorded Windows paths without consulting the old filesystem."""
    if not isinstance(value, str) or not value or '\x00' in value:
        raise ValueError('Archived source path must be a nonempty string')
    root = project_root.resolve()
    windows = PureWindowsPath(value)
    if '..' in windows.parts:
        raise ValueError(f'Path traversal in archived source: {value}')
    native = Path(value)
    if native.is_absolute():
        local = native.resolve()
        if local.is_relative_to(root):
            return local
    if windows.drive:
        if not windows.is_absolute():
            raise ValueError(f'Drive-relative archived path is unsupported: {value}')
        anchors = [index for index, part in enumerate(windows.parts)
                   if part.casefold() == ARCHIVED_ROOT_NAME.casefold()]
        if len(anchors) != 1:
            raise ValueError(f'Unrecognized absolute archived path: {value}')
        parts = windows.parts[anchors[0] + 1:]
    elif windows.root or native.is_absolute():
        raise ValueError(f'Unrecognized absolute archived path: {value}')
    else:
        # PureWindowsPath recognizes both slash styles on every host OS.
        parts = windows.parts
    local = root.joinpath(*parts).resolve()
    if not local.is_relative_to(root):
        raise ValueError(f'Archived path resolves outside the project: {value}')
    return local


def prepare_validation(project_root: Path, provenance_path: Path, original_result: Path,
                       output: Path, *, scan_cache: Path | None = None,
                       check_only: bool = False) -> dict:
    root = Path(project_root).resolve()
    provenance_path = Path(provenance_path).resolve()
    original_result = Path(original_result).resolve()
    output = Path(output).resolve()
    if output == root or not output.is_relative_to(root):
        raise ValueError('Output directory must be a separate directory inside the project')
    provenance_bytes = provenance_path.read_bytes()
    original_bytes = original_result.read_bytes()
    provenance = json.loads(provenance_bytes)
    saved = json.loads(original_bytes)
    if provenance.get('scope') != SCOPE or provenance.get('sources_unchanged_after_evaluation') is not True:
        raise ValueError('Expected a completed unchanged-source exact-PCD baseline provenance')
    paths, hashes = provenance.get('source_paths'), provenance.get('source_sha256')
    if not isinstance(paths, dict) or not isinstance(hashes, dict) or not paths or set(paths) != set(hashes):
        raise ValueError('Every archived source path must have exactly one frozen SHA-256')
    required = {'scan_cache', 'protocol', 'pcd_original'}
    if not required.issubset(paths):
        raise ValueError('Baseline provenance lacks scan-cache, protocol or original-PCD sources')
    resolved, mapping = {}, {}
    for key, archived in paths.items():
        expected = hashes[key]
        if not isinstance(expected, str) or not re.fullmatch(r'[0-9a-f]{64}', expected):
            raise ValueError(f'Invalid frozen SHA-256 for source {key}')
        local = (Path(scan_cache).expanduser().resolve()
                 if key == 'scan_cache' and scan_cache is not None
                 else resolve_archived_path(archived, root))
        if not local.is_file():
            if key == 'scan_cache':
                raise ValueError(
                    f'Missing scan cache: {local}. Extract the same recording with '
                    'python scripts/audit_bag.py --bag data/fin301/fin301_0.db3 --output outputs/fin301, '
                    'or supply --scan-cache /path/to/scans.npz. Its bytes must match the frozen SHA-256.')
            raise ValueError(f'Missing frozen source {key}: {local}')
        actual = file_hash(local)
        if actual != expected:
            raise ValueError(f'SHA-256 mismatch for {key}: {local}; expected {expected}, got {actual}. '
                             'Do not change historical hashes to accept different source bytes.')
        resolved[key] = local
        mapping[key] = {'archived_path': archived, 'local_path': str(local), 'sha256': actual,
                        'external_cache_override': key == 'scan_cache' and scan_cache is not None}
    if saved.get('scope') != SCOPE:
        raise ValueError('Original result has a different evaluation scope')
    if saved.get('protocol_sha256') != hashes['protocol']:
        raise ValueError('Original result protocol differs from the frozen provenance')
    if saved.get('pcd_sha256') != hashes['pcd_original']:
        raise ValueError('Original result PCD differs from the frozen provenance')
    if saved.get('quality_gates') != provenance.get('quality_gates'):
        raise ValueError('Original result quality gates differ from the frozen provenance')
    protected = {provenance_path, original_result, *resolved.values()}
    destinations = [output / 'provenance.json', output / 'original/localization_evaluation.json']
    if any(path.resolve() in protected for path in destinations):
        raise ValueError('Output would overwrite archived provenance or a historical source')
    # Reject symlinked output descendants that would leave the selected folder.
    if any(not path.resolve().is_relative_to(output) for path in destinations):
        raise ValueError('Output path resolves outside the selected local directory')
    metadata = {
        'schema': 'localized_validation_paths_v1',
        'archived_provenance_path': str(provenance_path),
        'archived_provenance_sha256': hashlib.sha256(provenance_bytes).hexdigest(),
        'original_result_path': str(original_result),
        'original_result_sha256': hashlib.sha256(original_bytes).hexdigest(),
        'project_root': str(root), 'source_path_mapping': mapping,
        'historical_sources_modified': False, 'content_hashes_preserved': True,
    }
    localized = copy.deepcopy(provenance)
    localized['source_paths'] = {key: str(path) for key, path in resolved.items()}
    localized['local_path_preparation'] = metadata
    payloads = [(json.dumps(localized, indent=2, allow_nan=False) + '\n').encode('utf-8'), original_bytes]
    # Check both files before creating either, preserving earlier local runs too.
    for destination, payload in zip(destinations, payloads):
        if destination.exists() and (not destination.is_file() or destination.read_bytes() != payload):
            raise ValueError(f'Output already exists with different bytes: {destination}; choose another --output')
    if not check_only:
        for destination, payload in zip(destinations, payloads):
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not destination.exists():
                destination.write_bytes(payload)
    return {'status': 'VALIDATED' if check_only else 'PREPARED',
            'validated_source_count': len(resolved), 'check_only': bool(check_only),
            'output_directory': str(output),
            'archived_provenance_sha256': metadata['archived_provenance_sha256'],
            'original_result_sha256': metadata['original_result_sha256'],
            'historical_sources_modified': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--provenance', type=Path,
                        default=ROOT / 'outputs/fin301/temporal_filter/validation/provenance.json')
    parser.add_argument('--original-result', type=Path,
                        help='Defaults to original/localization_evaluation.json beside the provenance')
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs/fin301/local_validation',
                        help='New local control directory; historical evidence is never rewritten')
    parser.add_argument('--scan-cache', type=Path,
                        help='Optional external scans.npz; the archived SHA-256 must still match')
    parser.add_argument('--check-only', action='store_true', help='Validate without creating output files')
    args = parser.parse_args()
    original = args.original_result or args.provenance.parent / 'original/localization_evaluation.json'
    try:
        outcome = prepare_validation(ROOT, args.provenance, original, args.output,
                                     scan_cache=args.scan_cache, check_only=args.check_only)
    except (OSError, ValueError, TypeError) as error:
        parser.exit(2, f'Error: {error}\n')
    print(json.dumps(outcome, indent=2))


if __name__ == '__main__':
    main()
