# FinTech 301 indoor map (Ada5, Yuntian's version)

Built from the group's recording `fin301` (FinTech 301, 2026-10-29 18:15:41 EDT, 592.7 s)
with the `ada5_slam` ROS 2 package in `project2_2_ws/src/ada5_slam`. Only the 2D LiDAR and the
forward camera were used: `/scan`, `/camera/camera/color/image_raw/compressed`,
`/camera/camera/color/camera_info` and the sensor mounts in `/tf_static`. The IMU, magnetometer,
`/odom`, `/tf` (odom to base_link), the vehicle's `/map` and AMCL were not used.

## Files

| File | Content |
|---|---|
| `fin301_map.pgm`, `fin301_map.yaml` | the map, ROS `map_server` format (trinary) |
| `fin301_map.png` | map figure with the trajectory and the loop closures |
| `data/trajectory.csv` | estimated pose for each processed scan: `t_s` (s after the first message), `x_m`, `y_m`, `yaw_rad`, match status |
| `data/keyframes.csv` | optimised keyframe poses (the pose-graph nodes) |
| `data/loops.json` | every loop candidate with its camera and LiDAR evidence; `accepted` marks the 12 used |
| `data/consistency.json` | LiDAR revisit check (validation only, see below) |
| `data/summary.json` | all parameters and numbers; `summary_no_camera_prior.json` is the run without the camera heading prior |
| `data/validation_targets.json`, `figures/validation_targets.png` | map distances to compare with tape measurements |
| `figures/` | calibration check, tracking quality, camera vs LiDAR heading, loop closure before and after, failure case |

## Map format and conventions

- 2D occupancy grid, 0.05 m cells, 1241 x 975 cells (62.05 m x 48.75 m). In the PGM, 0 (black) is
  occupied, 254 (white) free and 205 (grey) unknown; the YAML uses `occupied_thresh` 0.65 and
  `free_thresh` 0.196.
- Map frame: origin at `base_link` at the start of the recording; x is the vehicle's forward
  direction at the start, y points left, z up; metres; heading counter-clockwise from x. The YAML
  `origin` (-41.785, -30.869, 0) is the map-frame position of the lower-left cell.
- What the cells mean: the map is the horizontal LiDAR scan plane, 0.034 m above `base_link` and
  0.087 m above the camera (from `/tf_static`; its height above the floor was not measured).
  Occupied cells are where returns ended: walls, furniture legs, pillars, and people who stood
  still long enough. Free cells were crossed by a beam before its return. Unknown cells were
  never crossed, including directions with no return (glass, open space beyond 20 m, the
  LiDAR's blind rear 90 degrees). Anything above or below the scan plane, such as table tops,
  is not in the map.

## How to open

- Any image viewer opens `fin301_map.pgm` and `fin301_map.png`.
- ROS 2 (Jazzy): `ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=fin301_map.yaml`,
  then `ros2 lifecycle set /map_server configure` and `ros2 lifecycle set /map_server activate`,
  and add a Map display on `/map` (fixed frame `map`) in RViz2. The `nav2_map_server` package is
  not installed on the course VM (`sudo apt install ros-jazzy-nav2-map-server`).

## Method

| Stage | Sensor | What it does |
|---|---|---|
| Scan preprocessing | LiDAR | every 2nd scan (14.8 Hz); 0.15-20 m; motion during the 34 ms sweep removed with a constant-velocity model; 5 cm voxels |
| Odometry | LiDAR (+ camera) | point-to-line ICP against a local map of the last 30 keyframes; a motion gate rejects matches that disagree with the prediction unless they are strong (inlier share >= 0.6) |
| Heading prior | camera | ORB features in frames 0.2 s apart, essential matrix with RANSAC, rotation converted to the vehicle's heading change. It replaces the constant-velocity heading prediction; when the two disagree by more than 1 degree, ICP runs from both and keeps the better match |
| Place recognition | camera | bag of binary words over ORB descriptors proposes earlier keyframes that look alike; an essential matrix (at least 20 RANSAC inliers) verifies the geometry |
| Loop measurement | LiDAR | ICP of the current scan against a local map around the earlier keyframe; accepted only with inlier share >= 0.65, RMS <= 0.025 m, no degenerate direction, and camera and LiDAR heading changes within 5 degrees. The camera has no scale; the edge's translation comes from the LiDAR |
| Back end | - | SE(2) pose graph over 911 keyframes, 910 odometry edges and 12 loop edges, SciPy least squares with a Huber loss |
| Map | LiDAR | occupancy grid from the keyframe scans at the optimised poses: free along each beam, occupied at its end |

The LiDAR odometry gives the metric motion and the map. The camera contributes the heading
prediction and the revisit proposals and their geometric check. Existing libraries: ROS 2 Jazzy
(rclpy, rosbag2_py, tf2_ros), OpenCV 4.6 (ORB, `findEssentialMat`, `recoverPose`,
`undistortPoints`), NumPy, SciPy (`cKDTree`, `least_squares`), Matplotlib. The ICP, odometry,
heading-prior integration, vocabulary, loop verification, pose graph and occupancy mapping are
our own code.

## Calibration, timing, frames

- Camera intrinsics and distortion from `camera_info` (plumb_bob). Mounts from `/tf_static`:
  the laser is 0.206 m behind and 0.087 m above the camera. Projecting scans into the images
  puts the returns on the pillars, walls and furniture they hit (`figures/calib_projection.png`).
- Each camera frame pairs with the scan whose forward beam is nearest in time: median offset
  1.0 ms, maximum 17 ms (half a scan period). Scan stamps are taken as the start of the sweep.
- Cross-check of the camera mount: over 2,708 frame pairs with a camera estimate, the camera
  heading change agrees with the LiDAR odometry with a median difference of 0.11 degrees
  (95th percentile 3.2 degrees); their ratio while turning is 0.98
  (`figures/camera_vs_lidar_heading.png`). A tilted or mis-rotated camera mount would show here.
- The tape-measured positions of the sensors on the car have not been checked against `/tf_static`.

## Results

| Check | Result |
|---|---|
| LiDAR odometry | 8,785 scans, 3 rejected matches (0:39, when the car first moves), median ICP inlier share 0.78, path 242.9 m |
| Camera heading | verified for 92% of frame pairs at 5 Hz; failures cluster at plain walls and fast turns |
| Loop closure | 2,570 appearance candidates, 38 passed the camera check, 12 accepted after LiDAR verification, all in the lounge (3:25-3:36 back to 2:15-2:28) |
| Loop discrepancy | before optimisation median 6.4 mm (max 10 mm, 0.18 deg); after 2.5 mm (max 3.9 mm) |
| Revisit check (LiDAR, 50 pairs 32-197 s apart) | position median 2.0 cm, 95th percentile 7.0 cm, max 9.1 cm; heading median 0.23 deg, max 0.49 deg |
| Effect of the camera heading prior (same check, odometry only) | 95th percentile 7.1 cm with it and 11.1 cm without; max 9.1 cm and 12.1 cm; medians similar (2.3 and 1.8 cm) |

The revisit check aligns, with LiDAR only, keyframe pairs that the estimate places within 1 m of
each other but that are at least 30 s and 10 m of path apart; the correction ICP needs is how far
the map disagrees with itself at that place. It shows repeatability, not absolute accuracy.

## Physical validation still needed

The recording holds no physical reference and the car did not return to its start, so a tape
measurement is needed. `figures/validation_targets.png` shows two corridor widths with the
camera view of each place:

| Target | Map width | Where |
|---|---|---|
| A | 2.44-2.52 m | corridor passed at 9:14 (`target_A_frame.jpg`) |
| B | 2.30-2.40 m | corridor passed at 8:15 (`target_B_frame.jpg`) |

The lower value is between the inner faces of the occupied cells, the upper between straight
lines fitted to the two walls. Measure wall to wall, perpendicular to the corridor, at the height
of the LiDAR, and record the tape's reading uncertainty (about 1 cm).

## Failure case and limitations

- Without the safeguards, a wrong camera heading at 7:51 produced a prediction that the motion
  gate trusted over a correct ICP match; the odometry then never recovered (1,777 rejected scans,
  `figures/failure_camera_prior_without_safeguard.png`). Trying both predictions and accepting
  strong matches fixed it.
- Camera place recognition is weak in this environment: the camera sits a few centimetres above
  the floor and sees chair legs, people and windows. Most revisits (start area, small room,
  corridor junction at the end) were not recognised. Three loops in the small room (5:48 back to
  4:01) passed the camera check but had a loose LiDAR fit (RMS 3-4 cm) and implied a 15 cm
  correction that the LiDAR revisit check contradicts; the stricter thresholds reject them.
- Glass walls and windows give no or spurious returns; people standing still appear as
  occupied cells; the rear 90 degrees of the LiDAR is blind.
- One recording only; no repeated run or ground truth. Offline processing takes about 30 min on
  the course VM, where OpenCV and NumPy run 10 to 250 times slower than normal (likely the
  VirtualBox/Hyper-V setup); for the same reason the node could not run in real time there.

## Live ROS 2 node (replay test)

`ros2 run ada5_slam slam_node` subscribes to the scan, compressed image and camera_info topics,
reads the mounts from `/tf_static`, publishes `/ada5/pose`, `/ada5/path`, `/ada5/map` and the
transform `map -> base_link`, and on shutdown runs loop closure and the pose graph and saves
`live_map.pgm/.yaml`. `scripts/live_replay_test.sh` replays the first 120 s at half speed and
records the node's output topics. The node ran and published in all three attempts (for example
857 poses recorded in the first), but the course VM could not process scans in time: the median
processing time per scan was 17-590 ms against about 80 ms offline, scans were dropped, and
23-68% of the processed scans failed the match checks, so the live map is not usable. The VM
was also measurably slower during these tests (the same OpenCV benchmark took 58 ms instead of
36.5 ms). The submitted map comes from the offline runner, which uses the same classes on the
recording. A live demonstration needs a faster computer. Records:
`results/fin301/live*/live_summary.json`.

## Reproduce

```bash
cd ~/project2_2_ws && colcon build --packages-select ada5_slam
python3 -I scripts/run_offline.py ~/project2_2_data/fin301 results/fin301/full --cache cache/fin301 --map-name fin301_map
python3 -I scripts/run_offline.py ~/project2_2_data/fin301 results/fin301/no_prior --cache cache/fin301 --no-camera-prior
```

Live (replay) use of the ROS 2 node:

```bash
ros2 launch ada5_slam ada5_slam.launch.py output_dir:=/tmp/ada5_live
ros2 bag play ~/project2_2_data/fin301 --topics /scan /camera/camera/color/image_raw/compressed /camera/camera/color/camera_info /tf_static
```
