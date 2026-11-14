"""Package fin301 filtering evidence, reproducible code, PCDs and an offline viewer."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import build_viewer


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    filtering = ROOT/'outputs/fin301/filtering'
    result = ROOT/'outputs/fin301/result'
    output = ROOT/'outputs/fin301/viewer_clean_en'
    archive = ROOT/'deliverables/fin301_viewer_clean_en.zip'
    filters = read_json(filtering/'filter_report.json')
    reference = read_json(filtering/'validation/filtering_validation.json')
    exported = read_json(filtering/'exported_validation/exported_validation.json')
    occupancy = read_json(filtering/'occupancy_filter_report.json')
    occupancy_validation = read_json(filtering/'exported_validation/occupancy_supported/exported_validation.json')
    assert hashlib.sha256((result/'lidar_scan_map.pcd').read_bytes()).hexdigest() == filters['source_sha256']
    assert reference['original_reproduction']['summary_numbers_exactly_equal']
    summary = {
        'dataset': 'fin301', 'default_localization_map': 'lidar_scan_map.pcd',
        'primary_artifacts_replaced': False,
        'selection_note': 'Filtered maps are reversible comparison candidates. Retain the original as the default; aggregate stability gains do not guarantee every location improves.',
        'filtering': filters, 'separated_reference_development_validation': reference,
        'exported_pcd_repeat_observation_validation': exported,
        'occupancy_supported_filter': occupancy,
        'occupancy_supported_repeat_observation_validation': occupancy_validation,
        'absolute_localization_accuracy_validated': False,
        'grid_semantics': 'map.yaml and map.pgm remain the original occupancy grid. Removed PCD points are not assigned free space.',
        'references': ['https://pointclouds.org/documentation/tutorials/remove_outliers.html',
                       'https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.query_ball_point.html']}
    summary_path = filtering/'cleanup_summary.json'
    summary_path.write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    names = ('conservative', 'balanced', 'radius_conservative', 'radius_balanced', 'occupancy_supported')
    pcds = [filtering/f'lidar_scan_map_clean_{name}.pcd' for name in names]
    built = build_viewer.build(result, ROOT/'viewer', output, archive, 'fin301', pcds, summary_path)

    # Enrich the viewer package with the complete bounded experiment and source.
    with zipfile.ZipFile(archive) as handle:
        members = {name: handle.read(name) for name in handle.namelist()}
    additions = {}
    for path in sorted(filtering.rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            additions['filtering/'+path.relative_to(filtering).as_posix()] = path.read_bytes()
    for path in sorted((ROOT/'indoor_mapping').glob('*.py')):
        additions['code/indoor_mapping/'+path.name] = path.read_bytes()
    scripts = ('filter_pcd_map.py', 'filter_pcd_occupancy.py', 'evaluate_map_filtering.py', 'evaluate_exported_pcd_filtering.py',
               'evaluate_exported_pcd_candidate.py',
               'evaluate_localization.py', 'package_map_cleanup.py', 'build_viewer.py')
    for name in scripts:
        additions['code/scripts/'+name] = (ROOT/'scripts'/name).read_bytes()
    additions['code/requirements.txt'] = (ROOT/'requirements.txt').read_bytes()
    additions['CLEANUP_README.md'] = (ROOT/'docs/map_filtering_fin301.md').read_bytes()
    additions['code/tests/test_map_filter.py'] = (ROOT/'tests/test_map_filter.py').read_bytes()
    additions['code/tests/test_occupancy_filter.py'] = (ROOT/'tests/test_occupancy_filter.py').read_bytes()
    for name, data in additions.items():
        if name in members:
            raise ValueError(f'Supplementary file collision: {name}')
        members[name] = data
        build_viewer.write_atomic(output/name, data)
    manifest = json.loads(members.pop('viewer_manifest.json'))
    manifest['supplementary_content'] = 'Filtering candidates, masks, validation evidence and Python source; original map files remain unchanged.'
    manifest['files'] = [{'path': name, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
                         for name, data in sorted(members.items())]
    members['viewer_manifest.json'] = (json.dumps(manifest, indent=2)+'\n').encode('utf-8')
    build_viewer.write_atomic(output/'viewer_manifest.json', members['viewer_manifest.json'])
    temporary = archive.with_suffix('.zip.tmp')
    with zipfile.ZipFile(temporary, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as handle:
        for name, data in sorted(members.items()):
            handle.writestr(name, data)
    build_viewer.verify_package(temporary, members)
    temporary.replace(archive)
    print(json.dumps({'status': 'PASS', 'viewer': built['html'], 'zip': str(archive),
                      'zip_bytes': archive.stat().st_size, 'members': len(members),
                      'original_pcd_sha256': filters['source_sha256']}, indent=2))


if __name__ == '__main__':
    main()
