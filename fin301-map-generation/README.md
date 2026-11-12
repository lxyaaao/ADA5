# Camera–LiDAR indoor mapping

Reconstructs both `fin301` and `Fin301-11051740` from their bundled original recordings.
Uses Python, ROS 2 and Cartographer 2D. No IMU, GNSS, recorded odometry/map,
manually drawn map or supplied screenshot is used.

## Run

Use Ubuntu 22.04, ROS 2 Humble, SQLite rosbag2 support, and Cartographer ROS 2
(including its Python messages, offline node, trajectory-query node and map exporter).
Source ROS and your Cartographer workspace setup, then:

```bash
pip install -r requirements.txt
git lfs pull
bash run_mapping.sh --dataset both
```

The runner restores the original databases from the bundled Git LFS chunks and verifies
SHA256 hashes. Allow space for about 4.1 GB compressed data, 11.3 GB restored data and
scratch files. `--dataset fin301` or `--dataset Fin301-11051740` selects one recording.
`--rebuild-backend` recomputes the mapping after algorithm changes.
`--data-root`, `--work-root`, `--results-root` and `--cartographer-prefix` support other locations.
Logs and intermediate states stay in ignored `work/`; runtime dependencies are not bundled.

## Method

An initial LiDAR trajectory proposes revisits. Timestamp-associated, undistorted camera
images verify place identity using ORB and homography RANSAC. Robust point-to-line ICP
supplies metric relative geometry. Camera-gated virtual place constraints enter a second
Cartographer optimization. Every third accepted pair is reserved from these added constraints;
a recording with only one accepted pair has no reserved pair. Cameras contribute to optimization,
while LiDAR supplies metric scale and scan-plane occupancy. Camera features are not projected
into LiDAR, so camera mounting information is inspected rather than used for pixel/range fusion.

Raw scan range limits are 0.22–12 m; occupancy resolution is 0.025 m. The camera-loop ICP helper
additionally filters isolated angular returns and uses robust residual weights. Cartographer's
submap matching and global optimization address drift; no corners or walls are drawn manually.
These steps cannot guarantee that every doubled wall or unobserved corner is resolved.

## Results

Each recording has its own directory under `results/` containing:

- `map.pgm` and `map.yaml`: occupied black, free white, unknown gray. Keep them together;
  open the PGM in an image viewer or load the YAML through a ROS map server.
- `map_xinyao_pcd_old.pcd` / `.yaml` for `fin301`, or `map_xinyao_pcd_new.pcd` / `.yaml`
  for `Fin301-11051740`: binary XYZ occupied-cell point cloud, in metres with z = 0.
  Open the PCD in CloudCompare or PCL. Its YAML describes the cloud; it is custom metadata,
  not a ROS occupancy-map YAML. Use `map.yaml` for the ROS map server.
- `vehicle_poses.csv`: measurement time in seconds, vehicle `base_link` x/y in metres and
  heading in radians. Full recorded static mounting transforms convert optimized laser poses
  using `map_from_base = map_from_laser × inverse(base_from_laser)`.
- `quality_checks.json`: initialization coverage, trajectory gaps, sampled scan alignment,
  pose-jump warnings, low-support scans, camera decode counts and numerical revisit disagreement.
  Flagged locations/times identify areas needing inspection; they are heuristic warnings,
  not proven tracking failures. The map represents returns in the LiDAR scan plane.

Each map uses Cartographer's initial laser frame as its local origin and axes. Trajectory
positions and map dimensions are metric. Reliability checks sample approximately once per
second; limited coverage, glass, repetitive views and missing corners can remain undetected.
Revisit residuals reuse the mapping sensors and do not establish absolute accuracy.

## Optional physical-reference checks

No independently measured room distances/checkpoints with uncertainty were supplied.
The results explicitly mark physical-reference validation unavailable; no measurements are invented.
To evaluate actual measurements, supply `--reference-measurements /path/to/measurements.json`.
The JSON object is keyed by dataset name, with a list of checks for each dataset. Each check needs
`timestamp_a_s`, `timestamp_b_s`, `measured_distance_m`, `uncertainty_m` (positive stated bound),
and `source` (how the physical measurement was obtained). Measure the Euclidean separation of
vehicle `base_link` positions at the two recorded times, not the travelled path length.
The result records estimated separation, signed error and comparison with the stated uncertainty.
These checks assess the measured separations, not global map accuracy or a SLAM confidence interval.

## GitHub

Keep `.gitattributes` and install Git LFS before committing the large data chunks.
Clone with Git and run `git lfs pull`; ZIP downloads may contain pointers instead of data.
Unchanged data is reused when pushing later code/result commits. Restored databases and scratch
files are ignored. No reports, comparison versions or report-generation scripts are included.

Dependencies: [Cartographer ROS 2](https://github.com/ros2/cartographer_ros),
[Ceres](https://ceres-solver.org/), [OpenCV](https://opencv.org/), NumPy, SciPy, PyYAML and ROS 2.
Upstream Lua configuration files retain their license notices.
