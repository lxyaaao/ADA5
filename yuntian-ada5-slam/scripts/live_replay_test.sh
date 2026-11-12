#!/usr/bin/env bash
# Replay the start of a recording through the ROS 2 node and keep what it published.
#
# Usage: live_replay_test.sh <bag_dir> <out_dir> [duration_s] [rate]
# Plays from t = 0 because /tf_static (the sensor mounts) is published only at the start.
# Only the topics the method uses are played; /tf, /odom and /map from the vehicle are not.
# Background jobs of a non-interactive shell ignore SIGINT, so the node process itself is
# signalled (rclpy installs its own handler) and the recorder is stopped with SIGTERM.
set -eu
BAG=$1
OUT=$(realpath -m "$2")
DUR=${3:-120}
RATE=${4:-0.5}
WS=$(cd "$(dirname "$0")/.." && pwd)
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
rm -rf "$OUT" && mkdir -p "$OUT"

ros2 run ada5_slam slam_node --ros-args -p output_dir:="$OUT" -p process_every:=3 -p max_prior_wait_s:=0.0 > "$OUT/node.log" 2>&1 &
NODE=$!
ros2 bag record -o "$OUT/node_outputs" /ada5/pose /ada5/path > "$OUT/record.log" 2>&1 &
REC=$!
sleep 4
ros2 bag play "$BAG" --rate "$RATE" --playback-duration "$DUR" \
  --topics /scan /camera/camera/color/image_raw/compressed /camera/camera/color/camera_info /tf_static \
  > "$OUT/play.log" 2>&1
sleep 5
pkill -INT -f "lib/ada5_slam/slam_node" || true
kill -TERM "$REC" || true
timeout 900 tail --pid="$NODE" -f /dev/null || true
wait "$REC" || true
ros2 bag info "$OUT/node_outputs" > "$OUT/node_outputs_info.txt" 2>&1 || true
echo "done: $OUT"
