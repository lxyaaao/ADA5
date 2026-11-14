# Sensor bag audit

Bag: `data/fin301/fin301_0.db3` (5.548 GB).

Only raw laser scans and RGB images are motion/loop-estimation inputs. CameraInfo and static TF supply calibration. Recorded odometry, poses, dynamic TF, IMU, and maps are excluded.

| Stream | Messages | Unique stamps | Duplicate stamps | Hz (median) | Header reversals | Gaps > 1.5 cadence |
|---|---:|---:|---:|---:|---:|---:|
| `/scan/raw` | 17628 | 17628 | 0 | 29.783 | 0 | 0 |
| `/scan` | 17570 | 17570 | 0 | 29.783 | 0 | 0 |
| `/camera/camera/color/image_raw/compressed` | 5879 | 5879 | 0 | 9.998 | 0 | 15 |

## Time alignment

| Direction | Median (ms) | p95 (ms) | Maximum (ms) | Within 50 ms |
|---|---:|---:|---:|---:|
| camera_to_nearest_raw_scan | 8.397 | 15.852 | 17.228 | 100.000% |
| raw_scan_to_nearest_camera | 25.124 | 48.966 | 3032.668 | 96.721% |

## Scan integrity

The two scan topics share 17570 unique stamps; 0 have byte-identical ranges and 17570 differ. The cache contains only deduplicated `/scan/raw`.
- `/scan/raw`: valid finite in-range fraction mean 60.72%, minimum 46.60%; duplicate-stamp range conflicts 0; frames {'laser': 17628}.
- `/scan`: valid finite in-range fraction mean 57.61%, minimum 43.33%; duplicate-stamp range conflicts 0; frames {'laser': 17570}.

## Camera and calibration

Exported all 5879 compressed frames (167.9 MB); decode failures: 2. Dimensions: {'640x480': 5877}.
Failed-decode image IDs: [2764, 3045]. Original compressed bytes are retained; downstream image processing must skip these frames.
CameraInfo has 1 unique calibrations. Recorded static TF contains 3 edges and 0 conflicting changes. Camera-to-laser static path available: True.
Camera intrinsics and first/last CameraInfo, full static transforms, and transform conventions are in `calibration.json`. Static extrinsics are recorded values; physical accuracy has not been independently established.

## Timing gaps and receipt delays

- `/scan/raw`: duration 591.900 s; largest unique-stamp interval 38.216 ms; estimated missing frames 0; receipt minus header median 34.095 ms, p95 35.272 ms, range [29.700, 42.181] ms.
- `/scan`: duration 589.952 s; largest unique-stamp interval 38.216 ms; estimated missing frames 0; receipt minus header median 34.790 ms, p95 36.236 ms, range [30.221, 44.647] ms.
- `/camera/camera/color/image_raw/compressed`: duration 588.834 s; largest unique-stamp interval 239.788 ms; estimated missing frames 15; receipt minus header median 24.496 ms, p95 48.521 ms, range [0.421, 63.219] ms.
  - Gap 1793312576163071890–1793312576402860098: 0.240 s (estimated 1 frames).
  - Gap 1793312511109032790–1793312511313035538: 0.204 s (estimated 1 frames).
  - Gap 1793312472001559479–1793312472203091310: 0.202 s (estimated 1 frames).
  - Gap 1793312421485159089–1793312421685476134: 0.200 s (estimated 1 frames).
  - Gap 1793312678510103039–1793312678710153193: 0.200 s (estimated 1 frames).

## Recorded map baseline

**`recorded_baseline_UNVERIFIED.{pgm,png,yaml}` is the last recorded `/map`, exported for provenance only. It is not a validated map, a ground-truth map, or the project final output. It is never used by the estimator.**
Recorded `slam_toolbox` logs contain 0 queue-full message-filter drops. These logs concern the online mapping process; the raw scan stream audited here has its own independent completeness checks.

## Reproducibility

Full per-message header and bag timestamps are in `raw_scan_timestamps.csv`, `scan_timestamps.csv`, and `camera_timestamps.csv`. Full gap lists, topic counts, range checks, and synchronization statistics are in `audit.json`. `rosout.json` preserves recorded logging provenance.
Sensor cache: `outputs/fin301/cache/scans.npz`; image manifest: `outputs/fin301/cache/images.csv`.

Timestamp matching measures numerical alignment only; it does not establish hardware synchronization. Gap counts are based on a cadence heuristic. No external ground-truth trajectory is present in this audit.
