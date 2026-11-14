"""Build English offline viewers, temporal filtering report and verified ZIP."""
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


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    experiment = ROOT/'outputs/fin301/temporal_filter'
    filtering = load(experiment/'temporal_filter_report.json')
    validation = load(experiment/'validation/temporal_localization_review.json')
    variants = validation['variants']
    profiles = [('original', 'Original reference', ROOT/'outputs/fin301/result'),
                ('conservative', 'Conservative temporal map', experiment/'conservative/result'),
                ('static', 'Strict temporal support map', experiment/'static/result')]
    viewer_checks = []
    for name, label, result in profiles:
        viewer_checks.append(build_viewer.build(result, ROOT/'viewer', experiment/f'viewer_{name}',
                            ROOT/f'deliverables/fin301_temporal_{name}_viewer.zip', 'fin301',
                            display_title=f'fin301 - {label}'))
    rows = []
    for key, label in [('original','Original'), ('occupancy_supported','Previous occupancy filter'),
                       ('temporal_conservative','Conservative temporal filter'), ('temporal_static','Strict temporal support')]:
        value = variants[key]; summary = value['queries_summary']
        rows.append(f"| {label} | {value['pcd_points']:,} | {summary['accepted_registrations']}/800 | {summary['all_starts_agree_queries']}/100 |")
    report = f'''# fin301 temporal map filtering

The requested change removes temporally inconsistent LiDAR points while preserving fixed furniture where the observations support it. It uses the current delivered one-loop trajectory, without changing positions, adding loop constraints, smoothing walls or redrawing room structure. Both delivered LiDAR PCDs retain the exact original ASCII coordinate rows.

## Delivered maps

- **Conservative**: {filtering['candidates']['conservative']['retained_points']:,} LiDAR points; removes {filtering['conflicted_points']:,} strong temporal conflicts and preserves {filtering['insufficient_points']:,} points with insufficient evidence. This is the default cleanup view because insufficient observation is not evidence of motion.
- **Strict temporal support**: {filtering['static_supported_points']:,} LiDAR points; keeps only repeatedly supported points. It also excludes ambiguous or insufficient persistence evidence, potentially removing fixed structures and reducing coverage. All original points have at least one hit; this category includes mixed hit/free evidence, not just sparse observations.
- The original {filtering['input_points']:,}-point LiDAR map remains available for comparison. The 29 triangulated camera landmarks and every timestamp/pose are unchanged. Camera landmarks are a separate layer and were not temporally classified. Use `lidar_scan_map.pcd` for 2D localization; `map.pcd` additionally contains those camera landmarks.

Open `index.html` after extracting the whole ZIP. Each English viewer works offline, including orbit/pan, measurement, trajectory replay and downloads. Each map directory contains **map.pcd, lidar_scan_map.pcd, map.yaml and the required map.pgm companion**, plus coordinate metadata and trajectory. Browser interaction was not exercised in this task; the parser/renderer tests and embedded-file checks are recorded separately.

## Fixed method

The filter evaluates all {filtering['evidence_scans']:,} accepted, timestamped 2D LiDAR observations in the unchanged local map frame. A hit is a measured endpoint within 0.12 m of a map point. A visible-free contradiction requires every ray in a conservative angular neighborhood (at least 0.10 m lateral uncertainty) to extend beyond the point by more than 0.20 m. Invalid/no-return rays do not establish free space, and a closer object blocks free evidence behind it.

Each 2-second epoch supplies at most one vote per point; a hit overrides free evidence in the same epoch. Strong conflict requires at least three visible-free epochs spanning at least 4 seconds, with free votes at least 70% of hit-plus-free votes. Static support requires at least three hit epochs spanning at least 10 seconds and at most 30% free votes. Spans use actual supporting timestamps. These parameters were fixed before viewing the candidate results. Occluded and unobserved epochs are excluded from the vote denominator.

The YAML grid uses the same metres, resolution, origin and axes as the original. Original occupied cell centers farther than 0.10 m from retained LiDAR points become **unknown**, not free; all originally free and unknown cells are preserved. This raster is a conservative companion derived from the original occupancy grid, not a new ray-cast reconstruction or proof that removed obstacles are safely traversable. The PCD is an endpoint representation and can differ from the grid's accumulated occupancy classification.

## Matched localization check

| Map | LiDAR points | Accepted registrations | All-eight stable queries |
|---|---:|---:|---:|
{chr(10).join(rows)}

All variants use the same 100 query scans, eight starting perturbations, preprocessing and registration gates. Stability requires all eight accepted registrations to agree within 0.15 m and 3 degrees. These are **full-map repeat-observation diagnostics**, not absolute localization accuracy. Raw query IDs are excluded from filter evidence, but all 100 queries have another evidence observation within 0.069 seconds. The reused development data are strongly correlated. See `validation/temporal_localization_review.md` for paired gains, losses, newly failed locations and conditional metrics; a cleaner map does not imply better localization.

The previous occupancy-supported map remains stronger on both aggregate counts than either temporal candidate. It is preserved, and these temporal files supplement it rather than replace it as the localization reference. Conservative is the default **cleanup view**, not an accuracy ranking. Query scans still contain transient returns; removing their corresponding target points can reduce overlap without proving that the static structure is wrong. These tests concern exact PCD registration, not occupancy-grid traversability.

## Evidence and limitations

`temporal_before_after.png` shows identical-axis original/conservative/strict views. `visual_audit/main_room_temporal_detail.png` highlights removals in the cluttered room: conservative removes 988 of 26,650 points there; the dense structure largely remains. `temporal_point_evidence.csv` and `temporal_evidence.npz` retain per-point counts, event spans, source scan IDs and raw epoch flags. `temporal_support_examples.png` illustrates the recorded evidence. `visual_audit/visual_evidence.md` links timestamped camera panels: moving people and persistent stools are visible, but camera field of view and plane height do not identify every removed LiDAR point.

Temporal inconsistency can result from moving people/objects, range errors or residual pose misregistration. A stationary person may remain. Repeated support is not proof of permanence; excluded areas in the strict map may include real thin furniture, walls observed too briefly, or structures with mixed evidence. No external measured distances or checkpoint ground truth were supplied, so physical accuracy remains unvalidated.

## Reproduction

Run from the original project workspace with the existing fin301 sensor cache and original result. The archived provenance records absolute source paths: relocating the project requires updating those path fields to the equivalent byte-identical sources before running its hash checks. Viewers and map files do not depend on those absolute paths.

```text
python scripts/filter_temporal_map.py
python scripts/export_temporal_map.py
python scripts/evaluate_exported_pcd_candidate.py --baseline-dir outputs/fin301/temporal_filter/validation --candidate outputs/fin301/temporal_filter/lidar_scan_map_clean_temporal_conservative.pcd --name temporal_conservative
python scripts/evaluate_exported_pcd_candidate.py --baseline-dir outputs/fin301/temporal_filter/validation --candidate outputs/fin301/temporal_filter/lidar_scan_map_clean_temporal_static.pcd --name temporal_static
python scripts/summarize_temporal_validation.py --candidate temporal_conservative --candidate temporal_static --filter-report outputs/fin301/temporal_filter/temporal_filter_report.json
python scripts/package_temporal_filter.py
```

The ZIP includes the copied immutable baseline controls and code, but excludes the multi-gigabyte recording/cache. Existing ROS 2 integration remains unchanged; this filtering stage runs offline after recorded sensor reconstruction.

The implementation is a custom planar visibility/time-consistency filter. Related primary sources: [Removert (Kim and Kim, IROS 2020)](https://gisbi-kim.github.io/publications/gkim-2020-iros.pdf) on visibility-based dynamic-map removal, and [OctoMap](https://octomap.github.io/) on occupancy/free-space representation. This code does not implement or claim equivalence to either complete system.
'''
    (experiment/'temporal_filter_report.md').write_text(report, encoding='utf-8')
    notes = {
        'original': 'Unfiltered reference; original observations and map preserved.',
        'conservative': 'Default cleanup view: remove strong temporal conflicts; retain insufficient evidence.',
        'static': 'Stricter inspection view: exclude ambiguous or insufficient persistence evidence; some real structures can disappear.'}
    cards = ''.join(f'<article><h2>{html.escape(label)}</h2><p>{notes[name]}</p><a href="viewer_{name}/viewer.html">Open English viewer</a></article>'
                    for name,label,_ in profiles)
    summary_rows = ''.join(f'<tr><td>{html.escape(row.split("|")[1].strip())}</td>'+''.join(f'<td>{cell.strip()}</td>' for cell in row.split('|')[2:5])+'</tr>' for row in rows)
    page = f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>fin301 - Temporal map filtering</title><style>body{{font:16px/1.6 system-ui,sans-serif;background:#f3f6f7;color:#183641;max-width:1180px;margin:32px auto;padding:0 24px}}h1{{line-height:1.2}}.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:18px}}article{{background:white;border:1px solid #cbd8dd;border-radius:12px;padding:22px}}h2{{font-size:21px}}a{{color:#006b89}}img{{width:100%;background:white;border-radius:10px}}table{{border-collapse:collapse;width:100%;background:white}}td,th{{text-align:left;border-bottom:1px solid #d2dfe3;padding:9px}}.note{{background:#e1edf0;border-radius:10px;padding:18px}}</style>
<h1>fin301 - Temporal map filtering</h1><p>Remove inconsistent observations using time and visibility. Fixed furniture is preserved when supported by the recorded measurements. All map coordinates and poses remain unchanged.</p>
<div class="cards">{cards}</div><p class="note">The conservative view retains points without enough evidence. The strict view also excludes them. Removed points are not automatically moving people, and removal never creates free space in the companion grid.</p>
<img src="temporal_before_after.png" alt="Original, conservative temporal filter and strict support map at identical axes">
<h2>Same-query localization diagnostics</h2><table><thead><tr><th>Map</th><th>LiDAR points</th><th>Accepted /800</th><th>Stable /100</th></tr></thead><tbody>{summary_rows}</tbody></table>
<p>Full-map repeat-observation tests with correlated data; no physical accuracy reference. The previous occupancy-supported map remains stronger on these aggregate counts and is preserved. Conservative is the default cleanup view, not a new accuracy winner. Read paired failures and limitations before selecting a localization target.</p>
<p><a href="temporal_filter_report.md">Method and reproduction</a> &middot; <a href="validation/temporal_localization_review.md">Detailed localization check</a> &middot; <a href="visual_audit/visual_evidence.md">Camera evidence</a> &middot; <a href="temporal_support_examples.png">Temporal evidence examples</a> &middot; <a href="visual_audit/main_room_temporal_detail.png">Cluttered room detail</a></p>
<p>PCD + YAML + PGM downloads: <a href="conservative/result/map.pcd">Conservative combined PCD</a> / <a href="conservative/result/lidar_scan_map.pcd">LiDAR PCD</a> / <a href="conservative/result/map.yaml">YAML</a> / <a href="conservative/result/map.pgm">PGM</a>. All distances are in metres.</p></html>'''
    (experiment/'index.html').write_text(page, encoding='utf-8')
    (experiment/'viewer_build_checks.json').write_text(json.dumps(viewer_checks, indent=2)+'\n', encoding='utf-8')
    files = {}
    def add(path):
        if not path.is_file() or path.is_symlink() or any(part in ('__pycache__','plot_cache','.matplotlib') for part in path.parts) or path.suffix in ('.pyc','.pyo'):
            return
        files[path.relative_to(ROOT).as_posix()] = path
    for directory in (experiment, ROOT/'indoor_mapping', ROOT/'scripts', ROOT/'tests', ROOT/'config', ROOT/'launch', ROOT/'resource', ROOT/'viewer', ROOT/'outputs/fin301/result'):
        for path in directory.rglob('*'): add(path)
    for name in ('requirements.txt','requirements-lock.txt','package.xml','setup.py','setup.cfg','pytest.ini'):
        add(ROOT/name)
    for name in ('verification/localization_protocol.json','filtering/exported_validation/occupancy_supported/localization_evaluation.json',
                 'filtering/exported_validation/occupancy_supported/provenance.json','filtering/lidar_scan_map_clean_occupancy_supported.pcd',
                 'filtering/lidar_scan_map_clean_conservative.pcd','filtering/lidar_scan_map_clean_radius_balanced.pcd'):
        add(ROOT/'outputs/fin301'/name)
    members = {name:path.read_bytes() for name,path in sorted(files.items())}
    members['OPEN_VIEWER.html'] = b'<!doctype html><html lang="en"><meta charset="utf-8"><meta http-equiv="refresh" content="0;url=outputs/fin301/temporal_filter/index.html"><a href="outputs/fin301/temporal_filter/index.html">Open temporal map comparison</a></html>'
    members['README.md'] = b'# fin301 temporal filter update\n\nExtract the whole ZIP and double-click OPEN_VIEWER.html. Three English viewers work offline. Start with the conservative cleanup map; the strict map also removes insufficiently observed geometry. PCD, YAML and companion PGM are in each temporal_filter/<profile>/result directory. See temporal_filter_report.md for fixed rules, measured diagnostics, limitations and reproduction. Original data are preserved; raw recording/cache are excluded.\n'
    members['DELIVERY_MANIFEST.json'] = (json.dumps({name:{'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()} for name,data in members.items()}, indent=2)+'\n').encode('utf-8')
    archive = ROOT/'deliverables/fin301_temporal_filter.zip'
    temporary = archive.with_suffix('.zip.tmp')
    with zipfile.ZipFile(temporary,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as handle:
        for name,data in sorted(members.items()):handle.writestr(name,data)
    build_viewer.verify_package(temporary,members)
    temporary.replace(archive)
    print(json.dumps({'status':'PASS','zip':str(archive),'bytes':archive.stat().st_size,'members':len(members),
                      'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'entrypoint':str(experiment/'index.html')},indent=2))


if __name__ == '__main__':
    main()
