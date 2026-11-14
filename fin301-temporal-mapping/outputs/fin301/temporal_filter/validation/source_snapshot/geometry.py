"""Planar sensor geometry and robust point-to-line scan registration.

Poses are ``[x_m, y_m, yaw_rad]`` and map coordinates from the local frame
into the parent frame. ``register_scan(current, previous)`` therefore returns
the current sensor frame in the previous sensor frame. Accumulate it with
``pose_compose(previous_world_pose, result['pose'])``.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


def wrap_angle(angle):
    """Wrap radians to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def _pose(pose):
    result = np.asarray(pose, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError('pose must contain three finite values [x, y, yaw]')
    return result


def _points(points):
    result = np.asarray(points, dtype=float)
    if result.size == 0:
        return np.empty((0, 2), dtype=float)
    if result.ndim != 2 or result.shape[1] != 2:
        raise ValueError('points must have shape (N, 2)')
    return result


def pose_compose(a, b):
    """Compose parent-from-A with A-from-B, returning parent-from-B."""
    a, b = _pose(a), _pose(b)
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([
        a[0] + c * b[0] - s * b[1],
        a[1] + s * b[0] + c * b[1],
        wrap_angle(a[2] + b[2]),
    ])


def pose_inverse(pose):
    """Invert a parent-from-local SE(2) transform."""
    x, y, angle = _pose(pose)
    c, s = np.cos(angle), np.sin(angle)
    return np.array([-c * x - s * y, s * x - c * y, wrap_angle(-angle)])


def pose_between(a, b):
    """Return A-from-B for poses A and B expressed in the same parent frame."""
    return pose_compose(pose_inverse(a), b)


def transform_points(points, pose):
    """Apply a local-to-parent transform to an (N, 2) point array."""
    points, pose = _points(points), _pose(pose)
    c, s = np.cos(pose[2]), np.sin(pose[2])
    rotation = np.array([[c, -s], [s, c]])
    return points @ rotation.T + pose[:2]


def voxel_downsample(points, voxel_size=0.04):
    """Return centroids of finite points in occupied square voxels.

    A nonpositive voxel size disables subsampling, preserving input order.
    """
    points = _points(points)
    points = points[np.all(np.isfinite(points), axis=1)]
    if voxel_size <= 0 or len(points) == 0:
        return points.copy()
    if not np.isfinite(voxel_size):
        raise ValueError('voxel_size must be finite')
    cells = np.floor(points / voxel_size).astype(np.int64)
    _, inverse, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    return np.column_stack([
        np.bincount(inverse, weights=points[:, dim]) / counts for dim in range(2)
    ])


def scan_to_points(
    ranges, angle_min, angle_increment, range_min=0.1, range_max=12.0,
    voxel_size=0.04, outlier_neighbors=6, outlier_sigma=3.0,
):
    """Convert a planar LaserScan into finite XY metric return points.

    Range limits are inclusive. Infinite/no-return rays are discarded; this
    function cannot recover their free-space evidence. Optional statistical
    filtering follows metric voxelization so dense nearby angular returns do
    not set a spacing threshold that erases distant surfaces. It removes points
    whose mean k-neighbor distance exceeds the robust median by
    ``outlier_sigma`` scaled median absolute deviations. Disable it
    with ``outlier_neighbors=0`` when preserving every valid ray for mapping.
    """
    values = np.asarray(ranges, dtype=float)
    if values.ndim != 1:
        raise ValueError('ranges must be a one-dimensional array')
    if not np.isfinite(angle_min) or not np.isfinite(angle_increment) or angle_increment == 0:
        raise ValueError('angular start and nonzero increment must be finite')
    if not 0 <= range_min < range_max or not np.isfinite(range_max):
        raise ValueError('range limits must satisfy 0 <= range_min < range_max')
    valid = np.isfinite(values) & (values >= range_min) & (values <= range_max)
    angles = angle_min + np.flatnonzero(valid) * angle_increment
    points = np.column_stack([values[valid] * np.cos(angles), values[valid] * np.sin(angles)])
    points = voxel_downsample(points, voxel_size)
    if outlier_neighbors > 0 and len(points) > outlier_neighbors + 2:
        k = min(int(outlier_neighbors) + 1, len(points))
        distances, _ = cKDTree(points).query(points, k=k)
        spacing = distances[:, 1:].mean(axis=1)
        median = np.median(spacing)
        mad = 1.4826 * np.median(np.abs(spacing - median))
        # A small relative floor handles uniformly sampled walls without
        # rejecting ordinary variations from range/angle quantization.
        threshold = median + outlier_sigma * max(mad, 0.2 * median, 1e-4)
        points = points[spacing <= threshold]
    return points


def _target_normals(points, tree, neighbors=9):
    """Estimate unoriented line normals using local PCA."""
    k = min(neighbors, len(points))
    _, indices = tree.query(points, k=k)
    patches = points[indices]
    centered = patches - patches.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', centered, centered) / k
    values, vectors = np.linalg.eigh(covariance)
    normals = vectors[:, :, 0]
    valid = (values[:, 1] > 1e-7) & (values[:, 0] < 0.25 * values[:, 1])
    return normals, valid


def _correspondences(source, pose, target, tree, normals, normal_valid, distance_cap):
    moved = transform_points(source, pose)
    distances, indices = tree.query(moved)
    valid = (distances <= distance_cap) & normal_valid[indices]
    if np.count_nonzero(valid) < 10:
        return None
    p, q, n, distances = moved[valid], target[indices[valid]], normals[indices[valid]], distances[valid]
    # Trim the worst Euclidean matches before the robust line fit. Keep a
    # minimum floor so exact/nearly exact correspondences survive roundoff.
    trimmed = distances <= max(np.quantile(distances, 0.9), 0.015)
    p, q, n = p[trimmed], q[trimmed], n[trimmed]
    residuals = np.einsum('ij,ij->i', n, p - q)
    jacobian = np.column_stack([n[:, 0], n[:, 1], -n[:, 0] * p[:, 1] + n[:, 1] * p[:, 0]])
    return jacobian, residuals, p, float(np.mean(valid)), distances[trimmed]


def register_scan(source, target, initial=None, max_distance=0.5, iterations=30):
    """Register source points into target coordinates using robust multiscale ICP.

    Args:
        source: Current scan, (N, 2) meters.
        target: Previous scan or local submap in its reference frame, (M, 2).
        initial: Initial source-to-target ``[x, y, yaw]``; identity by default.
        max_distance: Final Euclidean nearest-neighbor gate in meters; coarse
            levels use up to 1.5 times this gate.
        iterations: Approximate total number of iterations across three scales.

    Returns:
        Dict containing ``pose`` (source-to-target), ``rmse`` (trimmed
        point-to-line residual meters), ``inlier_ratio`` (fraction of finite
        source points passing the final correspondence/normal gate),
        ``condition`` (dimensionless radius-normalized Hessian condition),
        ``degenerate``, ``success``, ``reason``, ``iterations`` and
        ``euclidean_rmse``. Success requires enough overlap, a small residual,
        and observable geometry. A corridor can fit well but be degenerate;
        its unconstrained direction must not be treated as a valid estimate.

    This local registration requires overlapping scans and a reasonable
    initial guess. Residuals are internal consistency, not physical accuracy.
    """
    if not np.isfinite(max_distance) or max_distance <= 0 or iterations < 1:
        raise ValueError('max_distance and iterations must be positive')
    source, target = _points(source), _points(target)
    source = source[np.all(np.isfinite(source), axis=1)]
    target = target[np.all(np.isfinite(target), axis=1)]
    pose = np.zeros(3) if initial is None else _pose(initial).copy()
    result = dict(pose=pose, rmse=float('inf'), inlier_ratio=0.0,
                  condition=float('inf'), degenerate=True, success=False,
                  reason='insufficient_points', iterations=0, euclidean_rmse=float('inf'))
    if min(len(source), len(target)) < 12:
        return result
    levels = [(0.12, 1.5), (0.06, 1.1), (0.0, 1.0)]
    iterations_per_level = max(1, int(np.ceil(iterations / len(levels))))
    for voxel, gate_factor in levels:
        src = voxel_downsample(source, voxel)
        dst = voxel_downsample(target, voxel)
        if min(len(src), len(dst)) < 12:
            continue
        tree = cKDTree(dst)
        normals, normal_valid = _target_normals(dst, tree)
        for _ in range(iterations_per_level):
            matches = _correspondences(src, pose, dst, tree, normals, normal_valid, max_distance * gate_factor)
            if matches is None:
                break
            jacobian, residuals, _, _, _ = matches
            scale = max(0.015, 1.4826 * np.median(np.abs(residuals - np.median(residuals))))
            weights = np.minimum(1.0, 1.5 * scale / np.maximum(np.abs(residuals), 1e-12))
            weighted = jacobian * np.sqrt(weights[:, None])
            hessian = weighted.T @ weighted
            gradient = jacobian.T @ (weights * residuals)
            damping = 1e-7 * max(np.trace(hessian), 1.0)
            delta = -np.linalg.solve(hessian + damping * np.eye(3), gradient)
            # Bound a bad local linearization without discarding its direction.
            fraction = min(1.0, 0.3 / max(np.linalg.norm(delta[:2]), 1e-12), 0.15 / max(abs(delta[2]), 1e-12))
            delta *= fraction
            pose = pose_compose(delta, pose)
            result['iterations'] += 1
            if np.linalg.norm(delta[:2]) < 1e-5 and abs(delta[2]) < 1e-5:
                break
    tree = cKDTree(target)
    normals, normal_valid = _target_normals(target, tree)
    matches = _correspondences(source, pose, target, tree, normals, normal_valid, max_distance)
    result['pose'] = pose
    if matches is None:
        result['reason'] = 'insufficient_correspondences'
        return result
    jacobian, residuals, moved, inlier_ratio, distances = matches
    radius = max(0.5, np.sqrt(np.mean(np.sum((moved - moved.mean(axis=0)) ** 2, axis=1))))
    # Recenter the rotational Jacobian so conditioning does not depend on the
    # arbitrary world origin of a local submap.
    center = moved.mean(axis=0)
    jacobian[:, 2] += jacobian[:, 0] * center[1] - jacobian[:, 1] * center[0]
    jacobian[:, 2] /= radius
    eigenvalues = np.linalg.eigvalsh(jacobian.T @ jacobian)
    condition = float(eigenvalues[-1] / max(eigenvalues[0], 1e-12))
    rmse = float(np.sqrt(np.mean(residuals ** 2)))
    degenerate = bool(condition > 1e5)
    success = bool(not degenerate and inlier_ratio >= 0.25 and rmse <= min(0.15, max_distance * 0.4))
    result.update(rmse=rmse, inlier_ratio=inlier_ratio, condition=condition,
                  degenerate=degenerate, success=success,
                  euclidean_rmse=float(np.sqrt(np.mean(distances ** 2))),
                  reason='ok' if success else ('degenerate_geometry' if degenerate else 'poor_fit'))
    return result
