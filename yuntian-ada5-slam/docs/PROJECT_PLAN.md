# Project 2.2 work plan (group Ada5)

Status 2026-11-12: one complete version (Yuntian's, for comparison within the group) is in
`deliverables/yuntian_version/` (map, README with method and numbers, figures). Code:
`src/ada5_slam` (ROS 2 package) and `scripts/`. Still open: tape measurements of the two corridor
widths in `deliverables/yuntian_version/figures/validation_targets.png`, a tape check of the
sensor mounts, and the group report.

Input: the group's recording `fin301` (FinTech 301, 2026-10-29, 592.7 s).
The 2026-10-22 Canvas announcement named Gore Hall as the area to map;
confirm with the instructor that a FinTech 301 recording is accepted.
Inspection results: `validation/inspection_fin301/INSPECTION.md`.
Workspace: this folder on Windows, `/home/charlie/project2_2_ws` on the VM,
data in `/home/charlie/project2_2_data/fin301`.

Estimation uses only `/scan/raw` (or `/scan`), the compressed camera images,
`camera_info` and `/tf_static`. IMU, magnetometer, `/odom` and the recorded
`/map` are not estimator inputs.

## 1. Calibration and timing checks (Lecture 4)

- Use `camera_info` for K and distortion and `/tf_static` for the laser and
  camera mounts. Verify the mounts by measuring the car and by projecting
  scan points into undistorted images (L4 slides 22–24): wall returns should
  land on walls about 0.09 m above the camera centre.
- Pair each image with the scan whose header stamp is nearest (scans are
  every 33.6 ms). Record the pairing offsets. The header is treated as scan
  start; deskew with constant velocity over the 33.5 ms sweep (L4 slide 26,
  L6 slide 23).
- Use `/scan`: it is `/scan/raw` minus self-returns from the car (two fixed
  sectors, −47° to −45° and +2° to +12°, within 0.5 m). The rear 90° never
  returns, so the usable field of view is 270°.

## 2. LiDAR front end (Lecture 5 ICP; Lecture 6 slides 22–27)

- Preprocess: drop invalid and out-of-range beams, remove returns on the car,
  downsample.
- Scan-to-submap 2D ICP (point-to-line), with a constant-velocity prediction
  in place of the IMU. Keyframes by travelled distance or rotation.
- Motion gate: reject updates that disagree too much with the prediction, and
  skip map insertion for them (L6 slides 41–43).

## 3. Camera contribution (Lecture 6 slides 8–18)

- Rotation prior: ORB matches between consecutive keyframes, essential matrix
  with RANSAC, relative yaw as the ICP initial guess. Monocular translation
  has no scale; the LiDAR supplies it (L6 slide 18).
- Place recognition: compare keyframe images to propose revisits, verify with
  essential-matrix inliers, then measure the metric relative pose with LiDAR
  ICP between the two keyframes. Accept the loop only when both agree.
- Ablation to report: the same run with and without camera loop closure and
  with and without the rotation prior.
- Expect weak or no visual matches during fast turns and at plain walls (13–15%
  of the fin301 frames); fall back to the constant-velocity prediction there.

## 4. Back end and map (Lecture 6 slides 17, 28, 44–45)

- 2D pose graph: ICP odometry edges plus verified loop edges, robust loss,
  optimized with SciPy. Rebuild the map from the corrected poses.
- Occupancy grid at 0.05 m with ray casting (free, occupied, unknown). Save as
  PGM + YAML (ROS `map_server` format) plus a PNG figure. Origin: first
  keyframe pose; x forward, y left, metres. The map shows only the
  horizontal scan plane at the laser's mount height (z = +0.034 m in
  `base_link`), so table tops and anything above or below it are missing.

## 5. ROS 2 integration

- Wrap the core in a ROS 2 node: subscribe to the scan, compressed image and
  `camera_info`; publish pose, path, map and `map -> base_link`. Replay the
  bag through it for a short live check, and run the full recording offline
  with the same core.

## 6. Validation

- Physical reference (required): compare map distances with tape
  measurements in FinTech 301 (corridor width, room length, distance between
  marked points), and the start/end gap if the car returned to a marked spot.
  State each reference's uncertainty.
- Revisit consistency before and after loop closure, at the revisits listed in
  `INSPECTION.md`. fin301 does not end at its start, so its start/end gap is not
  a reference.
- Initialization period, tracking failures and unreliable areas (glass,
  people, blur, low texture).
- The pre-built `/map` may be shown for qualitative comparison only; it is
  another estimate, not a measurement.

## 7. Deliverables

- Map files and a README for opening them.
- One-page group report: method and role of each sensor, calibration, timing,
  frames, no-IMU handling, at least one numerical result with its reference,
  one engineering problem, one failure case, map figure.
- One individual contribution report per member.
