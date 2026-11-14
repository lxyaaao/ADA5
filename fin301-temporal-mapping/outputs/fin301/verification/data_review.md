# fin301 independent sensor-data review

Status: complete. The review opened the new SQLite bag read-only, independently counted all topics, manually parsed 16 CDR headers, checked four complete raw range arrays against the exported cache, re-derived timestamp matching from all exported stamps, and rechecked failed images against source bytes. No forbidden sensor or recorded mapping output was used as an estimator input.

Recording start: 2026-10-29T22:15:41.754264+00:00. Metadata span 592.718981374 s; raw-scan header span 591.900184059 s. SQLite and metadata agree on all 313,349 records.

| New-bag stream | Messages / unique stamps | Duplicates | Header reversals | Gaps >1.5 median cadence |
|---|---:|---:|---:|---:|
| /scan/raw | 17,628 / 17,628 | 0 | 0 | 0 |
| /scan | 17,570 / 17,570 | 0 | 0 | 0 |
| /camera/camera/color/image_raw/compressed | 5,879 / 5,879 | 0 | 0 | 15 |

Raw cache is [17628, 1440], float32 ranges and strictly increasing int64 nanosecond stamps. It contains no zero, negative or NaN ranges; 9,971,191 +inf returns remain explicitly invalid. Mean finite in-range raw-beam fraction is 60.72%.

## New-bag timing and message completeness

Camera starts 3.032668365 s after raw LiDAR. There are 91 raw scans before the first camera image. Filtered /scan starts 1.947861990 s after raw; its 58 fewer messages are all raw-only initial timestamps. All 17,570 shared stamps have different range arrays, and no duplicate-stamp conflicts occur in either stream.

| Association | Median / p95 / max (ms) | Within 50 ms |
|---|---:|---:|
| camera_to_raw | 8.397 / 15.852 / 17.228 | 5,879/5,879 (100.000%) |
| raw_to_camera | 25.124 / 48.966 / 3032.668 | 17,050/17,628 (96.721%) |

Within camera coverage, maximum raw-to-camera difference is 108.291 ms. The 3,032.668 ms global maximum is a startup-coverage effect, not a clock-offset estimate. Timestamp matching does not prove hardware synchronization.

There are 5,886 distinct CameraInfo stamps: 10 lack a compressed image, while 3 compressed-image stamps lack CameraInfo. The net count difference of 7 therefore is not the count of missing image events. Calibration is constant across all CameraInfo messages. The 15 cadence-gap estimates in the audit are heuristic and should not be presented as a known acquisition-loss count.

All 5,879 exported image paths exist. OpenCV successfully decodes 5,877 frames at 640 x 480. IDs 2764 (t=1793312422285594020) and 3045 (t=1793312450479810596) fail OpenCV and Pillow; Pillow reports truncated JPEGs. Their exported bytes exactly match the original database payloads, so these failures were not introduced by extraction. Skip the original bad frames.

## Calibration, frames and local origin

New-bag CameraInfo: frame camera_link, plumb_bob distortion; K=[[433.76863, 0.0, 311.07619], [0.0, 430.02557, 221.52648], [0.0, 0.0, 1.0]]; D=[-0.066549, 0.048809, -0.000212, -0.004628, 0.0]. One unique calibration is present. Raw LiDAR frame is laser, 1,440 beams, angular increment 0.004363323096 rad.

- base_link <- laser: translation [-0.082558, -0.017229, 0.034095] m; quaternion xyzw [0.0, 0.0, 0.0, 1.0].
- base_link <- camera_link: translation [0.12323, -0.017229, -0.053395] m; quaternion xyzw [-0.49999999999662686, 0.5000018366025516, -0.49999999999662686, 0.4999981633974483].

The new bag independently contains the same recorded camera intrinsics and three static mounting transforms as the previous bag. Laser rotation is identity, so its recorded axes align with base_link. Camera optical axes are right/down/forward. These equal values do not establish that physical recalibration was performed or that mounting remained unchanged. There is no global/map origin in these static sensor mounts; the estimator may define its local origin at its first accepted raw LiDAR pose, with x forward, y left and z=0 as a scan-plane projection.

## Data-specific concerns

- Use fin301 counts and timestamps; no duplicate scan timestamps/conflicts are present in this recording.
- The first 3.032668365 s of raw LiDAR precede camera startup; enforce the timestamp association gate.
- Two original compressed image payloads are truncated (IDs 2764 and 3045); skip them without image repair.
- CameraInfo and compressed-image delivery are not one-to-one: 10 info stamps lack images, 3 image stamps lack info; intrinsics are constant across recorded CameraInfo.
- Recorded calibration values equal the prior bag but remain physically unverified; use new bag calibration as provenance.
- No independent physical ground truth or externally measured geometry has been supplied.
- Recorded /map is a single unverified message from navigation context and is excluded from estimation.
- The first RGB spot-check contains people and furniture legs; the last is dominated by nearby low-texture cabinet/wall surfaces. Dynamic objects and limited visual texture may reduce reliable associations.
- A planar laser sweep lasts about 33.58 ms; if the estimator does not compensate within-sweep motion, rolling-scan distortion remains a limitation.

Full counts, sample record IDs, bad-image SHA-256 values, calibration comparison and independent calculations are in `data_review.json`. This review validates extraction consistency, not trajectory or map absolute accuracy.
