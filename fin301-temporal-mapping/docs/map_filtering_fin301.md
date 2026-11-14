# fin301 point-cloud cleanup

The cleanup produces separate PCD variants in `outputs/fin301/filtering`. Original map artifacts and the existing project delivery are preserved. Every exported point is copied from an original ASCII XYZ row: no coordinates are shifted, averaged, snapped onto a wall, or synthesized. Camera landmarks are excluded from this planar filter.

## Method and limits

The radius filter counts other LiDAR map points within 0.12 m. The plain `radius_conservative` preset requires at least three neighbors, and `radius_balanced` requires six. Points failing that test are removed. This is the standard radius-outlier idea implemented locally with SciPy's `cKDTree`, rather than a call to PCL.

The `conservative`, `balanced` and `strong` presets add protection for sparse straight segments and corner branches. A candidate is retained if a line through the original point has at least four other supporting returns within 0.45 m, no more than 0.025 m perpendicular distance, and at least 0.18 m span. This protects thin geometry but can also preserve coherent false segments or accidental alignments. These presets require three, six and ten nearby points respectively before line protection.

The full map contains 49,783 scan-plane points. Plain radius filtering removes 829 points (1.67%) or 3,445 points (6.92%); the line-protected variants remove only 98, 238 and 298 points. Much of the visually cluttered region consists of dense or locally coherent returns, so these filters cannot turn it into an empty floor plan. The data do not independently distinguish furniture, reflections, moving returns and misregistered structures. Removing real sparse features can harm localization.

The original `map.yaml` describes the original `map.pgm`, including its resolution, origin and thresholds. It does not describe a PCD. The filtered PCDs share the same metric map frame, but the grid is deliberately retained as the original occupancy context. Removing a point is not evidence that its cell is free. The map origin is the first LiDAR pose, with initial x forward and y left; z=0 denotes the scan plane.

After the exact-PCD test exposed poor matcher compatibility with the accumulated point union, one additional fixed candidate was evaluated: `occupancy_supported`. It retains original PCD points whose grid-cell center is at most 0.10 m from an occupied-cell center in the existing ray-traced grid. The Euclidean distance transform uses the actual YAML origin, yaw, resolution, negate and strict occupancy thresholds. This is not a threshold sweep. It retains 30,165 points and excludes 19,618 (39.41%), including 17,983 points in cells classed free and 1,635 in cells classed unknown. Every point in an occupied cell is retained.

This removes substantially more interior clutter, but the grid is derived from the same observations. Grid agreement is correlated consistency evidence, not truth: a real sparse surface or transient obstacle can be excluded. The original grid and point coordinates are not edited. This candidate is tested only in the full-map repeat-observation protocol; using its full-map mask inside a temporally excluded reference-only test would leak other observations.

## Validation

The final quantitative selection and source hashes are in `outputs/fin301/filtering/cleanup_summary.json`. The detailed reference-only evaluation retains the frozen 100 queries, eight initializations per query, and 20 earlier-pass queries. It excludes temporally nearby reference scans and compares filters against both the original reference assembly and a 0.03 m spacing control. Filtering is applied only to allowed reference observations, never by consulting the full-map keep mask.

Because those reference subsets have a different density from the complete PCD, a second test localizes the same query scans directly against exported PCD files. That full-map test includes temporally nearby mapping observations and is explicitly a repeat-observation stability test. It is not the earlier-pass or temporal-exclusion experiment.

Neither test has independently measured vehicle positions. Acceptance and agreement between initializations are development indicators, not absolute localization error. These recordings and query sets have already been used in method selection, so they are not a new independent test set. Inspect the per-query gains and losses rather than interpreting a lower fitting residual as proven physical accuracy.

The separated-reference results are:

| Variant | Full PCD points removed | Accepted starts / 800 | Stable queries / 100 | Earlier-pass stable / 20 |
|---|---:|---:|---:|---:|
| Original | 0 | 712 | 71 | 11 |
| Spacing-only control | Not a PCD export | 704 | 68 | 12 |
| Conservative, segment protection | 98 (0.20%) | 719 | 75 | 12 |
| Balanced, segment protection | 238 (0.48%) | 720 | 74 | 12 |
| Radius conservative | 829 (1.67%) | 726 | 79 | 13 |
| Radius balanced | 3,445 (6.92%) | 641 | 66 | 10 |

The original remains the default. The protected conservative variant is a mild comparison candidate: six queries gain stability and two lose it against the original protocol, while its visual change is small. Plain radius conservative has higher aggregate counts but four formerly stable queries become unstable, and one query loses all eight accepted starts. Plain radius balanced is a stronger visual-cleanup candidate that clearly degrades this reference-only localization protocol; it is not recommended as a replacement localization map. Full-PCD deletion percentages in the table must not be confused with deletion rates in the sparser reference-only targets.

The **direct exported-PCD** test is a separate diagnostic:

| Exact exported cloud | Accepted starts / 800 | Stable queries / 100 |
|---|---:|---:|
| Original | 283 | 11 |
| Conservative, segment protection | 287 | 11 |
| Radius balanced | 299 | 16 |
| Occupancy supported | 332 | 29 |

The occupancy-supported cloud is the clearest visual-cleanup candidate and improves aggregate full-PCD stability in this test. Twenty queries become stable and two formerly stable queries regress; 116 starts gain acceptance while 67 lose it. Only 29/100 queries are stable, so this remains an experimental cleanup candidate rather than a globally reliable localization result. These counts must not be mixed with the separated-reference table. The two source representations expose materially different matcher behavior, and the occupancy mask has not been evaluated with independent physical pose references.

## Use and reproduce

Open `outputs/fin301/viewer_clean_en/viewer.html` offline. Choose **Filtered: occupancy supported** in the **Point cloud file** selector to see the substantially cleaned map. The selector also offers the original and other tested filtered clouds. The original opens by default; all layers retain their coordinate frame. Switching bundled clouds preserves zoom and view position. The sidebar reports the removed fraction for filtered variants. PCD and YAML/PGM downloads are included in the viewer package.

Run from the original project root:

```powershell
.venv/Scripts/python.exe scripts/filter_pcd_map.py
.venv/Scripts/python.exe scripts/filter_pcd_occupancy.py
.venv/Scripts/python.exe scripts/evaluate_map_filtering.py --presets conservative balanced radius_conservative radius_balanced
```

The first command writes masks, candidate PCDs, source hashes and equal-axis figures. The second command evaluates the fixed reference-only protocol. Raw sensor cache files and the original project dependencies are required for evaluation; viewing or importing the PCDs requires no sensor cache. `scripts/evaluate_exported_pcd_filtering.py --help` documents the exported-PCD validation.

The cleanup ZIP includes the viewer, original PCD/YAML/PGM files, filtered PCDs, comparison figures, validation logs, masks and Python sources. For the extracted package, open the top-level `viewer.html`; the project-relative paths above describe the original workspace. To rerun filtering without raw recordings, change into its `code` directory and run `python scripts/filter_pcd_map.py --source ../maps/lidar_scan_map.pcd --output ../regenerated`. Evaluation requires the original sensor cache and frozen protocol from the project.

Algorithm references: [PCL radius-outlier tutorial](https://pointclouds.org/documentation/tutorials/remove_outliers.html), [SciPy KDTree radius query](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.query_ball_point.html). The segment protection and exact-row export are this project's implementation, not a PCL guarantee.
