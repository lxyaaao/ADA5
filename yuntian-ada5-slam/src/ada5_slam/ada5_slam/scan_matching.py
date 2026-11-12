"""Scan preprocessing and 2D point-to-line ICP (Lecture 5; Lecture 6 slides 22-27)."""
import numpy as np
from scipy.spatial import cKDTree

from .geometry import transform, wrap


def scan_to_base(ranges, angles, beam_dt, laser_pose, rmin, rmax, vel=None):
    """Valid returns as N x 2 points in base_link at the scan's mid time.

    beam_dt is each beam's time relative to the mid time. With a body
    velocity (vx, vy, w) the motion during the 34 ms sweep is removed with a
    constant-velocity model.
    """
    ok = np.isfinite(ranges) & (ranges > rmin) & (ranges < rmax)
    r, a, dt = ranges[ok].astype(float), angles[ok], beam_dt[ok]
    pts = transform(laser_pose, np.column_stack([r * np.cos(a), r * np.sin(a)]))
    if vel is not None:
        th = vel[2] * dt
        c, s = np.cos(th), np.sin(th)
        pts = np.column_stack([c * pts[:, 0] - s * pts[:, 1] + vel[0] * dt,
                               s * pts[:, 0] + c * pts[:, 1] + vel[1] * dt])
    return pts


def voxel_filter(pts, size):
    """Centroid of the points in each size x size cell."""
    if len(pts) == 0:
        return pts
    k = np.floor(pts / size).astype(np.int64)
    key = (k[:, 0] + (1 << 20)) * (1 << 21) + (k[:, 1] + (1 << 20))
    _, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    return np.column_stack([np.bincount(inv, pts[:, 0]), np.bincount(inv, pts[:, 1])]) / cnt[:, None]


def line_normals(pts, k=8, radius=0.25):
    """Normal of the local line through each point and whether the fit is line-like."""
    tree = cKDTree(pts)
    d, idx = tree.query(pts, k=k, distance_upper_bound=radius)
    valid = np.isfinite(d)
    P = np.vstack([pts, [0.0, 0.0]])[idx]
    cnt = valid.sum(1)
    mu = (P * valid[..., None]).sum(1) / np.maximum(cnt, 1)[:, None]
    Q = (P - mu[:, None, :]) * valid[..., None]
    cxx, cyy = (Q[..., 0] ** 2).sum(1), (Q[..., 1] ** 2).sum(1)
    cxy = (Q[..., 0] * Q[..., 1]).sum(1)
    phi = 0.5 * np.arctan2(2 * cxy, cxx - cyy)         # direction of the fitted line
    half, det = 0.5 * (cxx + cyy), cxx * cyy - cxy ** 2
    disc = np.sqrt(np.maximum(half ** 2 - det, 0.0))
    ratio = (half - disc) / np.maximum(half + disc, 1e-12)
    return np.column_stack([-np.sin(phi), np.cos(phi)]), (cnt >= 4) & (ratio < 0.15)


class LocalMap:
    """Points with line normals and a KD-tree for point-to-line matching."""

    def __init__(self, pts):
        normals, good = line_normals(pts)
        self.pts, self.normals = pts[good], normals[good]
        self.tree = cKDTree(self.pts)

    def __len__(self):
        return len(self.pts)


def icp(src, lmap, init, max_iter=30, gate_start=0.6, gate_end=0.15, huber=0.05, inlier_dist=0.10):
    """Align src (N x 2, sensor frame) to lmap starting from init (x, y, theta).

    Rotation is applied about the current sensor position so the update is
    well conditioned far from the map origin. Returns the pose and quality
    figures: inlier share, point-to-line RMS, Gauss-Newton Hessian.
    """
    pose = np.array(init, dtype=float)
    H = np.zeros((3, 3))
    it = 0
    for it in range(max_iter):
        p = transform(pose, src)
        d, j = lmap.tree.query(p, distance_upper_bound=max(gate_end, gate_start * 0.75 ** it))
        m = np.isfinite(d)
        if m.sum() < 25:
            return pose, {'ok': False, 'inlier': 0.0, 'rmse': np.inf, 'iters': it + 1, 'H': H}
        jm = j[m]
        nx, ny = lmap.normals[jm, 0], lmap.normals[jm, 1]
        px, py = p[m, 0], p[m, 1]
        r = nx * (px - lmap.pts[jm, 0]) + ny * (py - lmap.pts[jm, 1])
        w = np.where(np.abs(r) <= huber, 1.0, huber / np.maximum(np.abs(r), 1e-12))
        a2 = ny * (px - pose[0]) - nx * (py - pose[1])     # d r / d theta about the sensor
        H = np.array([[np.sum(w * nx * nx), np.sum(w * nx * ny), np.sum(w * nx * a2)],
                      [0.0, np.sum(w * ny * ny), np.sum(w * ny * a2)],
                      [0.0, 0.0, np.sum(w * a2 * a2)]])
        H = np.triu(H) + np.triu(H, 1).T
        g = np.array([np.sum(w * nx * r), np.sum(w * ny * r), np.sum(w * a2 * r)])
        dx = -np.linalg.solve(H + 1e-6 * np.eye(3), g)
        pose = np.array([pose[0] + dx[0], pose[1] + dx[1], float(wrap(pose[2] + dx[2]))])
        if np.hypot(dx[0], dx[1]) < 1e-4 and abs(dx[2]) < 1e-5:
            break
    p = transform(pose, src)
    d, j = lmap.tree.query(p, distance_upper_bound=inlier_dist)
    m = np.isfinite(d)
    r = (np.sum(lmap.normals[j[m]] * (p[m] - lmap.pts[j[m]]), axis=1) if m.any() else np.array([np.inf]))
    return pose, {'ok': True, 'inlier': float(m.mean()), 'rmse': float(np.sqrt(np.mean(r ** 2))),
                  'iters': it + 1, 'H': H, 'n': int(len(src))}


def translation_eigen(H):
    """Smaller eigenvalue of the 2 x 2 translation block and its unit eigenvector (closed form)."""
    a, b, c = H[0, 0], H[0, 1], H[1, 1]
    half, disc = 0.5 * (a + c), np.sqrt(max(0.25 * (a - c) ** 2 + b * b, 0.0))
    phi = 0.5 * np.arctan2(2 * b, a - c)                 # direction of the larger eigenvalue
    return half - disc, np.array([-np.sin(phi), np.cos(phi)])


def weak_direction(H, n_points, threshold):
    """Unit translation direction the scan barely constrains (None if well constrained)."""
    lam, v = translation_eigen(H)
    return v if lam / max(n_points, 1) < threshold else None
