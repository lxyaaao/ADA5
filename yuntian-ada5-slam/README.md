# Ada5 SLAM: Yuntian's version (fin301)

Indoor 2D SLAM with the 2D LiDAR and the forward camera only (no IMU, no /odom), run on the
group's `fin301` recording. Kept separate from the Cartographer version in
`fin301-map-generation/` so the two can be compared.

- `results/` - the map (`fin301_map.pgm/.yaml/.png`), figures, data and a README with the map
  format, coordinate conventions, method, numbers, validation targets and limitations.
- `src/ada5_slam/` - ROS 2 (ament_python) package: LiDAR ICP odometry, camera heading prior,
  camera place recognition verified by LiDAR, pose graph, occupancy grid, live node.
- `scripts/` - offline runner, calibration check, live replay test, validation targets.
- `docs/` - recording inspection notes and the work plan.

The recording itself is not included (see `fin301-map-generation/data/`).

Build and run on the course VM:

```bash
colcon build --packages-select ada5_slam
python3 -I scripts/run_offline.py <bag_dir> results_out --cache cache --map-name fin301_map
```
