# ROS 2 runtime for fin301

The estimator processes an extracted recording in a batch. The ROS 2 adapter can run that batch or publish generated artifacts. It is not a live camera/LiDAR subscription node or an online navigation system.

Put this project in a ROS 2 workspace, for example `~/mapping_ws/src/indoor_mapping`. From the workspace root, with ROS 2 Jazzy installed:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select indoor_mapping --symlink-install
source install/setup.bash
ros2 launch indoor_mapping mapping.launch.py process_cache:=false output_dir:=/absolute/path/to/fin301-temporal-mapping/outputs/fin301/temporal_filter/conservative/result
```

Use `outputs/fin301/temporal_filter/static/result` for the strict map, or `outputs/fin301/result` for the original map. Keep `map.yaml` beside its referenced `map.pgm`.

Published historical artifacts:

- `/map`: 2D occupancy grid in `map`.
- `/map_points`: combined XYZ PCD. LiDAR returns have z=0 in the scan plane; 29 camera landmarks can have nonzero z.
- `/trajectory`: timestamped `base_link` poses derived from the LiDAR trajectory and recorded mounting transform.

In RViz use Fixed Frame `map` and Map, PointCloud2 and Path displays. Publishers use reliable, transient-local, depth 1 QoS for late subscribers. No live vehicle TF is published; recorded timestamps are retained.

For a new batch, extract observations with `scripts/audit_bag.py` and use `process_cache:=true`, an absolute `cache_dir`, and a new absolute `output_dir`. Set `lidar_revisits:=true` for the evaluated fin301 configuration; its default is false. Temporal cleanup runs separately after reconstruction.

Archived ROS checks for the original reconstruction: [direct artifact publication](outputs/fin301/verification/ros_final_artifacts_test.txt) and [batch processing inside ROS](outputs/fin301/verification/ros_processing_final_test.txt). They are prior-run evidence, not a claim that this checkout or the temporal variants were newly tested under ROS. Current Python tests and viewer validation are separate.
