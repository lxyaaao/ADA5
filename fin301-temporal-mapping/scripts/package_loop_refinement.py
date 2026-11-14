"""Package the bounded loop tests, evidence and three offline map viewers."""
from __future__ import annotations
import hashlib
import html
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import build_viewer


def main():
    experiment = ROOT/'outputs/fin301/loop_refinement'
    decision = json.loads((experiment/'selection.json').read_text(encoding='utf-8'))
    build_viewer.build(ROOT/'outputs/fin301/result', ROOT/'viewer', experiment/'viewer_current',
                       ROOT/'deliverables/fin301_loop_current_reference_viewer.zip', 'fin301',
                       [ROOT/'outputs/fin301/filtering/lidar_scan_map_clean_occupancy_supported.pcd'],
                       ROOT/'outputs/fin301/filtering/occupancy_filter_report.json',
                       'fin301 - Current one-loop reference')
    profiles = [('Current one-loop reference', 'current-one-loop', 'viewer_current'),
                ('All-heading loop test', 'all-heading', 'viewer_all_heading'),
                ('Multikeyframe loop test', 'multikeyframe', 'viewer_multikeyframe')]
    cards = []
    for label, name, directory in profiles:
        recommended = '<strong class="tag">Selected by the recorded policy</strong>' if decision['selected'] == name else '<span class="tag">Comparison candidate</span>'
        cards.append(f'<article>{recommended}<h2>{html.escape(label)}</h2><p>Open the map in its own matched frame, trajectory and occupancy grid.</p>'
                     f'<a href="{directory}/viewer.html">Open English viewer →</a></article>')
    selected = html.escape(decision['selected'])
    page = f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>fin301 · Loop-closure comparison</title><style>body{{font:16px/1.6 system-ui,sans-serif;background:#f2f5f6;color:#173642;max-width:1100px;margin:40px auto;padding:0 24px}}h1{{line-height:1.2}}.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:18px;margin:28px 0}}article{{background:white;border:1px solid #cbd7dd;border-radius:12px;padding:24px}}h2{{font-size:21px}}a{{color:#006b89}}.tag{{font-size:12px;color:#47727c}}.note{{background:#e4eef1;padding:18px;border-radius:8px}}code{{background:white;padding:2px 5px}}</style>
<h1>fin301 · Loop-closure comparison</h1><p>Selected default: <b>{selected}</b>. Original files are preserved. Each viewer works offline without a server.</p>
<div class="cards">{''.join(cards)}</div><p>In each viewer, use <b>Point cloud file → Filtered: occupancy supported</b> to inspect the cleaned PCD. Select <code>lidar_scan_map.pcd</code> for its unfiltered counterpart. Candidate grids and camera landmarks were rebuilt using their own proposed trajectories.</p>
<p class="note">Two measured-loop variants were tested with the same 780 mapping scans and fixed localization conditions. The new loops produce about 3.5 cm of additional trajectory correction. These are reused development and repeat-observation tests; no independent physical pose reference is available. A lower fitted loop residual does not establish accuracy.</p>
<p><a href="loop_refinement_report.md">Read the test report</a> · <a href="selection.json">Selection evidence</a> · <a href="visual_audit/visual_audit.md">Camera audit</a></p>
</html>'''
    (experiment/'index.html').write_text(page, encoding='utf-8')
    files = {}
    def add(path, name=None):
        if not path.is_file() or path.is_symlink() or any(part in ('__pycache__', 'plot_cache', '.matplotlib') for part in path.parts):
            return
        if path.suffix in ('.pyc', '.pyo'):
            return
        files[name or path.relative_to(ROOT).as_posix()] = path
    for path in experiment.rglob('*'):
        # The exporter reproduction control is validated in its provenance; omit
        # its redundant full map files and figures from the user package.
        if 'export_control' in path.parts and path.name not in ('export_provenance.json', 'map_insertion_indices.json'):
            continue
        add(path)
    for directory in ('indoor_mapping', 'scripts', 'tests', 'config', 'launch', 'resource'):
        for path in (ROOT/directory).rglob('*'):
            add(path)
    for name in ('requirements.txt', 'requirements-lock.txt', 'package.xml', 'setup.py', 'setup.cfg', 'pytest.ini'):
        add(ROOT/name)
    for name in ('fin301_readme.md', 'references.md', 'localization_validation_fin301.md', 'map_filtering_fin301.md'):
        add(ROOT/'docs'/name)
    # Preserve the original artifacts needed to inspect the matched controls.
    for path in (ROOT/'outputs/fin301/result').iterdir():
        add(path)
    controls = ['verification/localization_protocol.json',
                'verification/localization_final/localization_evaluation.json',
                'filtering/exported_validation/original/localization_evaluation.json',
                'filtering/exported_validation/provenance.json',
                'filtering/exported_validation/occupancy_supported/localization_evaluation.json',
                'filtering/exported_validation/occupancy_supported/provenance.json',
                'filtering/lidar_scan_map_clean_occupancy_supported.pcd', 'filtering/occupancy_filter_report.json']
    for name in controls:
        add(ROOT/'outputs/fin301'/name)
    members = {name: path.read_bytes() for name, path in sorted(files.items())}
    members['OPEN_COMPARISON.html'] = b'<!doctype html><html lang="en"><meta charset="utf-8"><meta http-equiv="refresh" content="0;url=outputs/fin301/loop_refinement/index.html"><a href="outputs/fin301/loop_refinement/index.html">Open loop comparison</a></html>'
    members['README.md'] = (f'# fin301 loop-closure tests\n\nExtract the entire ZIP, then open **OPEN_COMPARISON.html**. The three English viewers are self-contained and need no internet or local server.\n\nSelected default: **{decision["selected"]}**. The original map is preserved. Review `outputs/fin301/loop_refinement/loop_refinement_report.md` for matched tests, limitations and reproduction commands.\n\nEach experiment folder has `result/map.pcd`, `result/lidar_scan_map.pcd`, `result/map.yaml`, its companion `map.pgm`, `trajectory.csv`, and separately rebuilt camera landmarks. Its `filtering` folder has the occupancy-supported exact-subset PCD. Do not mix a candidate PCD with another candidate trajectory or grid.\n\nRaw recordings/cache files are excluded. Reproduction uses the existing project sensor cache. This package contains source code, constraint evidence, source snapshots, map artifacts, tests and viewers. Absolute accuracy remains unvalidated.\n').encode('utf-8')
    manifest = {name: {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()} for name, data in members.items()}
    members['DELIVERY_MANIFEST.json'] = (json.dumps(manifest, indent=2)+'\n').encode('utf-8')
    archive = ROOT/'deliverables/fin301_loop_refinement.zip'
    temporary = archive.with_suffix('.zip.tmp')
    with zipfile.ZipFile(temporary, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as handle:
        for name, data in sorted(members.items()):
            handle.writestr(name, data)
    build_viewer.verify_package(temporary, members)
    temporary.replace(archive)
    print(json.dumps({'status': 'PASS', 'selected': decision['selected'], 'zip': str(archive),
                      'bytes': archive.stat().st_size, 'members': len(members),
                      'entrypoint': str(experiment/'index.html')}, indent=2))


if __name__ == '__main__':
    main()
