"""Loop closure: the camera proposes and verifies a revisit, the LiDAR measures it.

A loop edge is added only when (1) the bag of words ranks an earlier
keyframe among the most similar, (2) an essential matrix between the two
images has enough RANSAC inliers, (3) ICP aligns the current scan with the
earlier keyframe's local map, and (4) the camera and LiDAR heading changes
agree. The camera has no scale; the edge's translation comes from the LiDAR.
"""
import numpy as np

from .geometry import between, transform, wrap
from .scan_matching import LocalMap, icp, voxel_filter, weak_direction
from .visual import BagOfWords


def nearest_images(times, image_times, max_dt):
    k = np.clip(np.searchsorted(image_times, times), 1, len(image_times) - 1)
    k = np.where(np.abs(image_times[k - 1] - times) <= np.abs(image_times[k] - times), k - 1, k)
    return np.where(np.abs(image_times[k] - times) <= max_dt, k, -1)


def submap_around(keyframes, j, half, voxel):
    """Points of keyframes j-half..j+half expressed in keyframe j's frame."""
    lo, hi = max(0, j - half), min(len(keyframes), j + half + 1)
    pts = [transform(between(keyframes[j]['pose'], keyframes[k]['pose']), keyframes[k]['pts'])
           for k in range(lo, hi)]
    return LocalMap(voxel_filter(np.vstack(pts), voxel))


def passes(r, p):
    """Acceptance test for a loop candidate that went through camera and LiDAR verification."""
    return bool(r.get('camera_ok') and 'lidar_inlier' in r
                and r['lidar_inlier'] >= p['loop_min_lidar_inlier'] and r['lidar_rmse'] <= p['loop_max_rmse']
                and r['yaw_disagreement_deg'] <= p['loop_max_yaw_disagreement_deg']
                and not r.get('degenerate', False)
                and r['frontend_discrepancy_m'] <= p['loop_max_correction_frac'] * r.get('path_between_m', 1e9) + 1.0)


def find_loops(keyframes, images, image_times, cam, p, log=print, kf_image_index=None):
    """kf_image_index: image index per keyframe (-1: none); by default the nearest image in time."""
    t = np.array([k['t'] for k in keyframes])
    img_idx = (np.asarray(kf_image_index) if kf_image_index is not None
               else nearest_images(t, image_times, p['loop_image_max_dt']))
    feats = [cam.features(images[k]) if k >= 0 else (np.zeros((0, 2), np.float32), None) for k in img_idx]
    log(f'  loop closure: {sum(f[1] is not None for f in feats)} keyframe images with features')
    bow = BagOfWords(p['bow_words']).fit([f[1] for f in feats if f[1] is not None])
    V = np.array([bow.vector(f[1]) if f[1] is not None else np.zeros(p['bow_words']) for f in feats])
    S = V @ V.T
    path = np.r_[0, np.cumsum(np.hypot(*np.diff(np.array([k['pose'][:2] for k in keyframes]), axis=0).T))]

    candidates = []
    for i in range(len(keyframes)):
        earlier = np.where((t[i] - t > p['loop_min_gap_s']) & (path[i] - path > p['loop_min_path_m']))[0]
        if not earlier.size or feats[i][1] is None:
            continue
        top = earlier[np.argsort(S[i, earlier])[::-1][:p['loop_top_k']]]
        candidates += [(i, int(j), float(S[i, j])) for j in top if S[i, j] >= p['loop_min_score']]
    log(f'  loop closure: {len(candidates)} appearance candidates')

    tried, accepted = [], []
    for i, j, score in candidates:
        yaw_c, n_in, direction = cam.relative_motion(feats[j], feats[i], min_inliers=p['loop_min_inliers'])
        r = {'i': i, 'j': j, 't_i': float(t[i]), 't_j': float(t[j]), 'score': score,
             'camera_inliers': int(n_in), 'camera_ok': yaw_c is not None}
        tried.append(r)
        if yaw_c is None:
            continue
        lmap = submap_around(keyframes, j, p['loop_submap_half'], p['map_voxel'])
        # Initial guesses: the current estimate of the relative pose, and the camera's heading
        # with the camera's direction of travel at a few scales (the camera has no scale).
        guesses = [between(keyframes[j]['pose'], keyframes[i]['pose']), np.array([0.0, 0.0, yaw_c])]
        if direction is not None:
            d = direction[:2] / max(np.linalg.norm(direction[:2]), 1e-9)
            guesses += [np.array([s * d[0], s * d[1], yaw_c]) for s in (0.5, 1.0, 2.0)]
        best = None
        for g in guesses:
            pose, q = icp(keyframes[i]['pts'], lmap, g, 40, 1.0, p['icp_gate_end'], p['icp_huber'],
                          p['icp_inlier_dist'])
            if q['ok'] and (best is None or q['inlier'] > best[1]['inlier']):
                best = (pose, q)
        if best is None:
            continue
        pose, q = best
        drift = between(between(keyframes[j]['pose'], keyframes[i]['pose']), pose)
        r.update({'lidar_inlier': q['inlier'], 'lidar_rmse': q['rmse'], 'z': pose.tolist(),
                  'yaw_disagreement_deg': float(np.degrees(abs(wrap(pose[2] - yaw_c)))),
                  'frontend_discrepancy_m': float(np.hypot(drift[0], drift[1])),
                  'frontend_discrepancy_deg': float(np.degrees(abs(drift[2])))})
        r['degenerate'] = weak_direction(q['H'], len(keyframes[i]['pts']), p['degenerate_eig']) is not None
        r['path_between_m'] = float(path[i] - path[j])
        r['accepted'] = passes(r, p)
        if r['accepted']:
            accepted.append(r)
    log(f'  loop closure: {sum(x["camera_ok"] for x in tried)} passed the camera check, '
        f'{len(accepted)} accepted after LiDAR verification')
    return accepted, tried, S
