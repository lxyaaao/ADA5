import cv2
import numpy as np

from indoor_mapping.visual_landmarks import camera_world_matrices, interpolate_lidar_poses, triangulate_tracks


def observations(points, cameras, intrinsic, distortion):
    pixels = []
    for world_from_camera in cameras:
        camera_from_world = np.linalg.inv(world_from_camera)
        rotation, _ = cv2.Rodrigues(camera_from_world[:3, :3])
        image, _ = cv2.projectPoints(points, rotation, camera_from_world[:3, 3], intrinsic, distortion)
        pixels.append(image[:, 0])
    return np.array(pixels)


def fixture():
    intrinsic = np.array([[430., 0, 320], [0, 425., 240], [0, 0, 1]])
    distortion = np.array([-.06, .04, -.001, -.003, 0.0])
    cameras = np.repeat(np.eye(4)[None], 4, axis=0)
    cameras[:, 0, 3] = [0, .15, .30, .45]
    points = np.array([[.1, -.2, 3.0], [-.3, .1, 4.0]])
    return intrinsic, distortion, cameras, points


def test_calibrated_four_view_triangulation_recovers_metric_points():
    intrinsic, distortion, cameras, truth = fixture()
    pixels = observations(truth, cameras, intrinsic, distortion)
    points, valid, diagnostic = triangulate_tracks(pixels, cameras, intrinsic, distortion)
    assert valid.all()
    np.testing.assert_allclose(points, truth, atol=1e-5)
    assert diagnostic['max_reprojection_px'].max() < 1e-4


def test_intermediate_view_error_is_rejected_even_when_endpoints_agree():
    intrinsic, distortion, cameras, truth = fixture()
    pixels = observations(truth, cameras, intrinsic, distortion)
    pixels[1, 0, 1] += 8
    _, valid, _ = triangulate_tracks(pixels, cameras, intrinsic, distortion)
    assert valid.tolist() == [False, True]


def test_triangulation_rejects_negative_depth_and_zero_parallax():
    intrinsic, distortion, cameras, truth = fixture()
    truth[0, 2] = -3
    pixels = observations(truth, cameras, intrinsic, distortion)
    _, valid, _ = triangulate_tracks(pixels, cameras, intrinsic, distortion)
    assert not valid[0]
    cameras[:] = np.eye(4)
    pixels = observations(truth, cameras, intrinsic, distortion)
    _, valid, _ = triangulate_tracks(pixels, cameras, intrinsic, distortion)
    assert not valid.any()


def test_pose_interpolation_preserves_nanoseconds_wraps_yaw_and_rejects_extrapolation():
    base = 1793312145547223432
    times = np.array([base, base + 200_000_000], dtype=np.int64)
    poses = np.array([[0, 0, np.deg2rad(179)], [1, 0, np.deg2rad(-179)]])
    interpolated, valid = interpolate_lidar_poses(times, poses, np.array([base - 1, base + 100_000_000], dtype=np.int64))
    assert valid.tolist() == [False, True]
    np.testing.assert_allclose(interpolated[1, :2], [.5, 0], atol=1e-12)
    assert abs(abs(interpolated[1, 2]) - np.pi) < 1e-12


def test_camera_extrinsic_direction_uses_inverse_of_camera_from_lidar():
    # Camera origin is forward and below the planar LiDAR origin.
    lidar_from_camera = np.array([[0, 0, 1, .2], [-1, 0, 0, 0], [0, -1, 0, -.09], [0, 0, 0, 1]])
    world_from_camera = camera_world_matrices(np.array([[1, 2, np.pi / 2]]), np.linalg.inv(lidar_from_camera))[0]
    np.testing.assert_allclose(world_from_camera[:3, 3], [1, 2.2, -.09], atol=1e-12)
    np.testing.assert_allclose(world_from_camera[:3, 2], [0, 1, 0], atol=1e-12)


def test_jpeg_validation_accepts_transport_zero_padding_but_rejects_truncation():
    from indoor_mapping.visual_landmarks import _complete_jpeg
    okay, encoded = cv2.imencode('.jpg', np.full((32, 48, 3), 180, np.uint8))
    assert okay
    content = encoded.tobytes()
    assert _complete_jpeg(content)
    assert _complete_jpeg(content + b'\x00')
    assert not _complete_jpeg(content[:-2])


def test_six_view_triangulation_checks_intermediate_observations():
    intrinsic, distortion, _, truth = fixture()
    cameras = np.repeat(np.eye(4)[None], 6, axis=0)
    cameras[:, 0, 3] = np.linspace(0, .45, 6)
    pixels = observations(truth, cameras, intrinsic, distortion)
    points, valid, diagnostic = triangulate_tracks(pixels, cameras, intrinsic, distortion)
    assert valid.all()
    np.testing.assert_allclose(points, truth, atol=1e-5)
    assert diagnostic['reprojection_px_all_views'].shape == (6, 2)
    pixels[4, 0, 1] += 8
    _, valid, _ = triangulate_tracks(pixels, cameras, intrinsic, distortion)
    assert valid.tolist() == [False, True]


def test_upward_world_ray_gate_does_not_clip_legitimate_negative_world_z():
    intrinsic, distortion, _, _ = fixture()
    cameras = np.repeat(np.eye(4)[None], 6, axis=0)
    cameras[:, :3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    cameras[:, 1, 3] = np.linspace(0, .45, 6)
    cameras[:, 2, 3] = -.1
    # Both are below the laser plane. Only the first is above the camera horizon.
    truth = np.array([[3., 0., -.02], [3., .2, -.2]])
    pixels = observations(truth, cameras, intrinsic, distortion)
    _, unrestricted, _ = triangulate_tracks(pixels, cameras, intrinsic, distortion)
    points, selected, diagnostic = triangulate_tracks(
        pixels, cameras, intrinsic, distortion, require_upward_world_rays=True)
    assert unrestricted.all()
    assert selected.tolist() == [True, False]
    assert diagnostic['upward_ray_ok'].tolist() == [True, False]
    assert points[0, 2] < 0
    np.testing.assert_allclose(points[0], truth[0], atol=1e-5)
