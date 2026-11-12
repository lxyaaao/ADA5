"""Revisit consistency check (validation only, LiDAR only; nothing here changes the map).

For keyframe pairs that the estimate places within `radius` m of each other
but that are at least `min_gap_s` apart in time and `min_path_m` apart along
the path, ICP aligns the later scan with a local map around the earlier
keyframe, starting from the estimated relative pose. The correction ICP
applies is how far the estimate disagrees with what the LiDAR sees on the
revisit. Pairs whose alignment is weak are reported separately.
"""
import numpy as np

from .geometry import between
from .loop_closure import submap_around
from .scan_matching import icp, weak_direction


def revisit_pairs(poses, times, path, radius=1.0, min_gap_s=30.0, min_path_m=10.0, every=3):
    """One earlier partner (the closest) for every `every`-th keyframe that has one."""
    pairs = []
    for i in range(0, len(poses), every):
        ok = (times[i] - times > min_gap_s) & (path[i] - path > min_path_m)
        if not ok.any():
            continue
        d = np.hypot(*(poses[:, :2] - poses[i, :2]).T)
        d[~ok] = np.inf
        j = int(np.argmin(d))
        if d[j] <= radius:
            pairs.append((i, j))
    return pairs


def check(keyframes, poses, p, radius=1.0, min_gap_s=30.0, min_path_m=10.0):
    """Disagreement (m, deg) between the estimate and LiDAR alignment at revisits."""
    kfs = [dict(k, pose=q) for k, q in zip(keyframes, poses)]
    times = np.array([k['t'] for k in kfs])
    path = np.r_[0, np.cumsum(np.hypot(*np.diff(poses[:, :2], axis=0).T))]
    out = []
    for i, j in revisit_pairs(poses, times, path, radius, min_gap_s, min_path_m):
        est = between(poses[j], poses[i])
        lmap = submap_around(kfs, j, p['loop_submap_half'], p['map_voxel'])
        pose, q = icp(kfs[i]['pts'], lmap, est, 40, 0.5, p['icp_gate_end'], p['icp_huber'], p['icp_inlier_dist'])
        if not q['ok']:
            continue
        corr = between(est, pose)
        strong = (q['inlier'] >= p['loop_min_lidar_inlier'] and q['rmse'] <= p['loop_max_rmse']
                  and weak_direction(q['H'], len(kfs[i]['pts']), p['degenerate_eig']) is None)
        out.append({'i': i, 'j': j, 't_i': float(times[i]), 't_j': float(times[j]),
                    'inlier': q['inlier'], 'strong': bool(strong),
                    'error_m': float(np.hypot(corr[0], corr[1])), 'error_deg': float(np.degrees(abs(corr[2])))})
    return out


def summarize(rows):
    s = [r for r in rows if r['strong']]
    if not s:
        return {'pairs_checked': len(rows), 'pairs_with_strong_alignment': 0}
    e, a = np.array([r['error_m'] for r in s]), np.array([r['error_deg'] for r in s])
    gaps = np.array([r['t_i'] - r['t_j'] for r in s])
    return {'pairs_checked': len(rows), 'pairs_with_strong_alignment': len(s),
            'time_between_visits_s_min_max': [float(gaps.min()), float(gaps.max())],
            'error_m_p50_p95_max': [float(np.median(e)), float(np.percentile(e, 95)), float(e.max())],
            'error_deg_p50_p95_max': [float(np.median(a)), float(np.percentile(a, 95)), float(a.max())]}
