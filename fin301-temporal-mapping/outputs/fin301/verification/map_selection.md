# fin301 map selection for localization

Selected: `outputs/fin301/result/lidar_scan_map.pcd`, using one LiDAR revisit constraint. The combined `map.pcd` and separate camera landmarks are companion mapping outputs.

| Configuration | Accepted starts | Stable queries | p95 within-query translation spread | Earlier-pass stable queries |
|---|---:|---:|---:|---:|
| baseline | 707/800 | 67/100 | 0.5284 m | 11/20 |
| final | 712/800 | 71/100 | 0.0091 m | 11/20 |
| persistent | 620/800 | 61/100 | 0.8501 m | 8/20 |

Stability requires all eight starts to pass and agree within 0.15 m / 3 degrees. These are development validation results used for selection, not ground-truth errors. Ten queries gain stability and six lose it; earlier-pass stable counts do not increase. No absolute localization accuracy or camera accuracy gain is claimed.

The persistent-point variant is rejected because acceptance and stability worsen. The original baseline and all candidate evidence are retained for inspection.
