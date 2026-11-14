"""Cross-module frame checks for the calibration schema written by bag_io."""

import json

import numpy as np

from indoor_mapping.geometry import pose_compose, pose_inverse
from indoor_mapping.pipeline import _vehicle_offset


def test_audit_matrix_gives_base_from_lidar_and_inverse_recovers_vehicle(tmp_path):
    # A rotated mount ensures a sign/direction mistake cannot cancel itself.
    offset = np.array([-0.08, 0.13, np.pi / 2])
    matrix = np.array([
        [0.0, -1.0, 0.0, offset[0]],
        [1.0, 0.0, 0.0, offset[1]],
        [0.0, 0.0, 1.0, 0.034],
        [0.0, 0.0, 0.0, 1.0],
    ])
    audit = {'tf_static': [{
        'parent': 'base_link', 'child': 'laser',
        'translation_xyz_m': matrix[:3, 3].tolist(),
        'quaternion_xyzw': [0, 0, np.sqrt(0.5), np.sqrt(0.5)],
        'T_parent_child': matrix.tolist(),
    }]}
    path = tmp_path / 'calibration.json'
    path.write_text(json.dumps(audit))
    recovered = _vehicle_offset(path)
    np.testing.assert_allclose(recovered, offset, atol=1e-12)
    world_from_base = np.array([3.0, -2.0, 0.4])
    world_from_lidar = pose_compose(world_from_base, offset)
    np.testing.assert_allclose(
        pose_compose(world_from_lidar, pose_inverse(recovered)),
        world_from_base, atol=1e-12,
    )


def test_missing_mount_transform_does_not_invent_vehicle_offset(tmp_path):
    path = tmp_path / 'calibration.json'
    path.write_text(json.dumps({'tf_static': []}))
    assert _vehicle_offset(path) is None
