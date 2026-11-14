"""Artifact conversion checks; these do not claim to exercise a ROS runtime."""

from pathlib import Path

import numpy as np
import pytest
from PIL import Image


def _adapter():
    from indoor_mapping import ros_node
    return ros_node


def test_map_conversion_preserves_metric_origin_and_ros_bottom_row(tmp_path):
    Image.fromarray(np.array([[0, 254], [205, 254]], dtype=np.uint8)).save(tmp_path / "map.pgm")
    (tmp_path / "map.yaml").write_text(
        "image: map.pgm\nresolution: 0.05\norigin: [-2.0, 1.0, 0.2]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8"
    )
    artifact = _adapter().load_map_artifact(tmp_path / "map.yaml")
    assert artifact.resolution == 0.05
    assert artifact.origin == (-2.0, 1.0, 0.2)
    assert (artifact.width, artifact.height) == (2, 2)
    assert artifact.cells == [-1, 0, 100, 0]


def test_trajectory_keeps_nanosecond_timestamp_precision(tmp_path):
    path = tmp_path / "trajectory.csv"
    path.write_text(
        "timestamp_ns,timestamp,x,y,yaw\n1530000000123456789,1530000000.1234567,1,2,0.5\n",
        encoding="utf-8",
    )
    pose = _adapter().load_trajectory(path)[0]
    assert pose.timestamp_ns == 1530000000123456789
    assert _adapter().split_timestamp(pose.timestamp_ns) == (1530000000, 123456789)
    assert (pose.x, pose.y, pose.yaw) == (1.0, 2.0, 0.5)


def test_trajectory_rejects_backward_time(tmp_path):
    path = tmp_path / "trajectory.csv"
    path.write_text("timestamp_ns,x,y,yaw\n200,0,0,0\n100,1,0,0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="timestamp"):
        _adapter().load_trajectory(path)


def test_trajectory_rejects_nonfinite_pose(tmp_path):
    path = tmp_path / "trajectory.csv"
    path.write_text("timestamp_ns,x,y,yaw\n100,nan,0,0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="finite"):
        _adapter().load_trajectory(path)


def test_ascii_pcd_preserves_xyz_and_handles_other_fields(tmp_path):
    path = tmp_path / "map.pcd"
    path.write_text(
        "VERSION .7\nFIELDS intensity x y z\nSIZE 4 4 4 4\nTYPE F F F F\n"
        "COUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n"
        "5 1.2 2.3 0\n8 -1 4 0\n", encoding="ascii"
    )
    points = _adapter().load_ascii_pcd(path)
    np.testing.assert_allclose(points, [[1.2, 2.3, 0], [-1, 4, 0]])
    assert points.dtype == np.dtype("<f4")


def test_pcd_rejects_truncated_points(tmp_path):
    path = tmp_path / "map.pcd"
    path.write_text(
        "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n1 2 0\n", encoding="ascii"
    )
    with pytest.raises(ValueError, match="POINTS"):
        _adapter().load_ascii_pcd(path)


def test_rejects_ros_timestamp_outside_builtin_time_range():
    with pytest.raises(ValueError, match="timestamp"):
        _adapter().split_timestamp(-1)
    with pytest.raises(ValueError, match="timestamp"):
        _adapter().split_timestamp(2147483648 * 10**9)
