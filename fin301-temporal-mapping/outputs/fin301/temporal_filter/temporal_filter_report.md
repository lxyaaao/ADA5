# fin301 temporal map filtering

The requested change removes temporally inconsistent LiDAR points while preserving fixed furniture where the observations support it. It uses the current delivered one-loop trajectory, without changing positions, adding loop constraints, smoothing walls or redrawing room structure. Both delivered LiDAR PCDs retain the exact original ASCII coordinate rows.

## Delivered maps

- **Conservative**: 47,679 LiDAR points; removes 2,104 strong temporal conflicts and preserves 8,065 points with insufficient evidence. This is the default cleanup view because insufficient observation is not evidence of motion.
- **Strict temporal support**: 39,614 LiDAR points; keeps only repeatedly supported points. It also excludes ambiguous or insufficient persistence evidence, potentially removing fixed structures and reducing coverage. All original points have at least one hit; this category includes mixed hit/free evidence, not just sparse observations.
- The original 49,783-point LiDAR map remains available for comparison. The 29 triangulated camera landmarks and every timestamp/pose are unchanged. Camera landmarks are a separate layer and were not temporally classified. Use `lidar_scan_map.pcd` for 2D localization; `map.pcd` additionally contains those camera landmarks.

Open `index.html` after extracting the whole ZIP. Each English viewer works offline, including orbit/pan, measurement, trajectory replay and downloads. Each map directory contains **map.pcd, lidar_scan_map.pcd, map.yaml and the required map.pgm companion**, plus coordinate metadata and trajectory. Browser interaction was not exercised in this task; the parser/renderer tests and embedded-file checks are recorded separately.

## Fixed method

The filter evaluates all 3,526 accepted, timestamped 2D LiDAR observations in the unchanged local map frame. A hit is a measured endpoint within 0.12 m of a map point. A visible-free contradiction requires every ray in a conservative angular neighborhood (at least 0.10 m lateral uncertainty) to extend beyond the point by more than 0.20 m. Invalid/no-return rays do not establish free space, and a closer object blocks free evidence behind it.

Each 2-second epoch supplies at most one vote per point; a hit overrides free evidence in the same epoch. Strong conflict requires at least three visible-free epochs spanning at least 4 seconds, with free votes at least 70% of hit-plus-free votes. Static support requires at least three hit epochs spanning at least 10 seconds and at most 30% free votes. Spans use actual supporting timestamps. These parameters were fixed before viewing the candidate results. Occluded and unobserved epochs are excluded from the vote denominator.

The YAML grid uses the same metres, resolution, origin and axes as the original. Original occupied cell centers farther than 0.10 m from retained LiDAR points become **unknown**, not free; all originally free and unknown cells are preserved. This raster is a conservative companion derived from the original occupancy grid, not a new ray-cast reconstruction or proof that removed obstacles are safely traversable. The PCD is an endpoint representation and can differ from the grid's accumulated occupancy classification.

## Matched localization check

| Map | LiDAR points | Accepted registrations | All-eight stable queries |
|---|---:|---:|---:|
| Original | 49,783 | 283/800 | 11/100 |
| Previous occupancy filter | 30,165 | 332/800 | 29/100 |
| Conservative temporal filter | 47,679 | 286/800 | 16/100 |
| Strict temporal support | 39,614 | 271/800 | 18/100 |

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
