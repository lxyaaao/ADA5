"""Offline run of the full method on a recording.

Stages: camera heading track, LiDAR odometry (with the camera heading prior
unless disabled), camera place recognition verified by LiDAR, pose-graph
optimisation, occupancy map. The ROS 2 node uses the same classes on live
or replayed topics.

Usage: python3 -m ada5_slam.pipeline <bag_dir> <out_dir> [--cache DIR]
       [--config YAML] [--no-camera-prior] [--no-camera-loops] [--frontend-only]
"""
import argparse
import json
import math
import os
import pickle
import time

import numpy as np
import yaml

from . import consistency
from .geometry import between, compose, wrap
from .lidar_odometry import LidarOdometry
from .loop_closure import find_loops, passes
from .occupancy import OccupancyGrid
from .pose_graph import edge_errors, optimize
from .recording import Recording
from .visual import Camera, YawTrack, build_yaw_track

DEFAULTS = {
    # LiDAR input
    'process_every': 2,            # every 2nd scan (15 Hz)
    'range_min': 0.15, 'range_max': 20.0,
    # LiDAR odometry
    'scan_voxel': 0.05, 'icp_voxel': 0.08, 'map_voxel': 0.05, 'local_map_keyframes': 30,
    'icp_max_iter': 30, 'icp_gate_start': 0.6, 'icp_gate_end': 0.15,
    'icp_huber': 0.05, 'icp_inlier_dist': 0.10,
    'min_inlier': 0.35, 'max_jump_m': 0.30, 'max_jump_deg': 12.0, 'degenerate_eig': 0.02,
    'strong_inlier': 0.60, 'prior_check_deg': 1.0,
    'nominal_dt': 0.0675,          # time between processed scans at process_every = 2
    'kf_dist': 0.25, 'kf_deg': 8.0,
    # camera
    'use_camera_prior': True, 'use_camera_loops': True,
    'camera_heading_every': 2,     # heading between frames 0.2 s apart (5 Hz)
    'orb_features_heading': 600, 'orb_features_loop': 1000,
    'camera_ransac_px': 1.5, 'camera_ransac_iters': 300,
    # loop closure
    'loop_image_max_dt': 0.06, 'bow_words': 200, 'loop_min_gap_s': 30.0, 'loop_min_path_m': 5.0,
    'loop_top_k': 3, 'loop_min_score': 0.25, 'loop_min_inliers': 20, 'loop_submap_half': 10,
    # A clean scan-to-map alignment has an RMS close to the LiDAR noise (about 0.02 m); loops
    # with a larger RMS or a smaller overlap were biased in fin301 (small room, 5:48 <- 4:01).
    'loop_min_lidar_inlier': 0.65, 'loop_max_rmse': 0.025, 'loop_max_yaw_disagreement_deg': 5.0,
    'loop_max_correction_frac': 0.10,
    # pose graph (odometry sigmas set from the LiDAR odometry's revisit errors, see consistency.py)
    'odom_sigma_xy': 0.005, 'odom_sigma_xy_per_m': 0.01, 'odom_sigma_deg': 0.1,
    'odom_sigma_deg_per_deg': 0.01, 'lost_inflation': 10.0,
    'loop_sigma_xy': 0.02, 'loop_sigma_deg': 0.3, 'graph_f_scale': 3.0,
    # map
    'map_resolution': 0.05, 'map_max_free': 12.0,
}


def load_params(path=None):
    p = dict(DEFAULTS)
    if path:
        with open(path) as f:
            p.update(yaml.safe_load(f) or {})
    return p


def scan_timing(rec):
    """Beam times relative to each scan's mid time, and the mid times (s after t0)."""
    n = rec.ranges.shape[1]
    ti = rec.scan_meta['time_increment']
    return (np.arange(n) - n / 2) * ti, rec.scan_t + (n / 2) * ti - rec.t0


def camera_from(rec, p, n_features):
    R_bc, _ = rec.mount('base_link', rec.camera_info['frame_id'])
    return Camera(rec.camera_info['K'], rec.camera_info['D'], R_bc, n_features,
                  p['camera_ransac_px'], p['camera_ransac_iters'])


def camera_heading(rec, cam, p, cache=None):
    """Heading changes between frames (cached: they do not depend on the LiDAR).

    Returns the track, the times of the frames used, the heading change from
    the previous used frame (nan if unverified) and the inlier counts.
    """
    key = (f"e{p['camera_heading_every']}_f{p['orb_features_heading']}_"
           f"r{p['camera_ransac_px']}_{p['camera_ransac_iters']}")
    path = os.path.join(cache, f'camera_heading_{key}.npz') if cache else None
    times = rec.image_t - rec.t0
    if path and os.path.exists(path):
        z = np.load(path)
        idx, yaw, inl = z['idx'], z['yaw'], z['inliers']
        return YawTrack(times[idx], yaw, np.isfinite(yaw)), times[idx], yaw, inl
    track, idx, yaw, inl = build_yaw_track(cam, rec.images, times, every=p['camera_heading_every'])
    if path:
        np.savez(path, idx=idx, yaw=yaw, inliers=inl)
    return track, times[idx], yaw, inl


def run_frontend(rec, params, yaw_prior=None, log_every=1000, max_time=None):
    beam_dt, t_mid = scan_timing(rec)
    odo = LidarOdometry(rec.angles, beam_dt, rec.laser_pose2d(), params)
    out, prev_t = [], None
    for k, s in enumerate(range(0, len(t_mid), params['process_every'])):
        t = float(t_mid[s])
        if max_time is not None and t > max_time:
            break
        inc = yaw_prior.increment(prev_t, t) if (yaw_prior is not None and prev_t is not None) else None
        r = odo.step(t, rec.ranges[s], inc)
        r['scan_index'] = s
        out.append(r)
        prev_t = t
        if log_every and k % log_every == 0:
            print(f'  front end {t:7.1f} s  keyframes {len(odo.keyframes):5d}', flush=True)
    return odo, out


def frontend_summary(recs):
    st = [r['status'] for r in recs]
    poses = np.array([r['pose'] for r in recs])
    inl = np.array([r['inlier'] for r in recs[1:]])
    weak = np.array([r['weak'] for r in recs[1:]], float)
    return {
        'scans_processed': len(recs),
        'status_counts': {s: st.count(s) for s in sorted(set(st))},
        'keyframes': int(sum(r['keyframe'] for r in recs)),
        'inlier_p5_p50': np.percentile(inl, [5, 50]).round(3).tolist(),
        'weak_eigen_p1_p5_p50': np.nanpercentile(weak, [1, 5, 50]).round(4).tolist(),
        'iterations_mean': round(float(np.mean([r['iters'] for r in recs[1:]])), 2),
        'path_length_m': round(float(np.hypot(*np.diff(poses[:, :2], axis=0).T).sum()), 2),
        'prior_used_share': round(float(np.mean([r['prior_used'] for r in recs])), 3),
        'both_predictions_tried_share': round(float(np.mean([r['both_tried'] for r in recs])), 3),
        'camera_prediction_chosen_share': round(float(np.mean([r['init_used'] == 'camera' for r in recs])), 3),
        'camera_chosen_when_both_tried': round(float(
            np.mean([r['init_used'] == 'camera' for r in recs if r['both_tried']] or [np.nan])), 3),
        'prediction_error_deg_p50_p95': np.percentile(
            [abs(np.degrees(wrap(r['pose'][2] - r['pred'][2]))) for r in recs[1:]], [50, 95]).round(3).tolist(),
    }


def heading_agreement(recs, image_t, yaw_cam):
    """Camera heading change per frame pair against the LiDAR odometry over the same interval."""
    t = np.array([r['t'] for r in recs])
    yaw_l = np.unwrap([r['pose'][2] for r in recs])
    k = np.where(np.isfinite(yaw_cam) & (image_t > t[0]) & (image_t < t[-1]))[0]
    k = k[k > 0]
    d_lidar = np.interp(image_t[k], t, yaw_l) - np.interp(image_t[k - 1], t, yaw_l)
    d_cam = yaw_cam[k]
    turning = np.abs(d_lidar) > np.radians(0.5)
    slope = float(np.sum(d_cam[turning] * d_lidar[turning]) / np.sum(d_lidar[turning] ** 2))
    err = np.degrees(np.abs(d_cam - d_lidar))
    return {'pairs': int(len(k)), 'pairs_with_camera_estimate_share': float(np.isfinite(yaw_cam[1:]).mean()),
            'abs_diff_deg_p50_p95': np.percentile(err, [50, 95]).round(3).tolist(),
            'slope_camera_vs_lidar_when_turning': round(slope, 4),
            'correlation': round(float(np.corrcoef(d_cam, d_lidar)[0, 1]), 4)}, (d_lidar, d_cam)


def odometry_edges(keyframes, recs, p):
    kf_rec = [k for k, r in enumerate(recs) if r['keyframe']]
    bad = np.cumsum([r['status'] == 'lost' for r in recs])
    edges = []
    for m in range(1, len(keyframes)):
        z = between(keyframes[m - 1]['pose'], keyframes[m]['pose'])
        s_xy = p['odom_sigma_xy'] + p['odom_sigma_xy_per_m'] * np.hypot(z[0], z[1])
        s_th = np.radians(p['odom_sigma_deg'] + p['odom_sigma_deg_per_deg'] * abs(np.degrees(z[2])))
        if bad[kf_rec[m]] > bad[kf_rec[m - 1]]:
            s_xy, s_th = s_xy * p['lost_inflation'], s_th * p['lost_inflation']
        edges.append((m - 1, m, z, np.diag([1 / s_xy, 1 / s_xy, 1 / s_th])))
    return edges, kf_rec


def corrected_trajectory(recs, kf_rec, kf_front, kf_opt):
    out, m = [], 0
    for k, r in enumerate(recs):
        while m + 1 < len(kf_rec) and kf_rec[m + 1] <= k:
            m += 1
        out.append(compose(kf_opt[m], between(kf_front[m], r['pose'])))
    return np.array(out)


def build_map(keyframes, poses, laser_xy, p):
    return OccupancyGrid(p['map_resolution']).build(poses, [k['pts'] for k in keyframes], laser_xy,
                                                    max_free=p['map_max_free'])


def clean(obj):
    """JSON-safe copy: numpy types to Python, NaN/inf to None."""
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return clean(obj.tolist())
    if isinstance(obj, (np.floating, float)):
        return float(obj) if math.isfinite(obj) else None
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    return obj


def save_json(path, obj):
    with open(path, 'w') as f:
        json.dump(clean(obj), f, indent=1)


FRONTEND_KEYS = ['process_every', 'range_min', 'range_max', 'scan_voxel', 'icp_voxel', 'map_voxel',
                 'local_map_keyframes', 'min_inlier', 'strong_inlier', 'prior_check_deg', 'kf_dist', 'kf_deg',
                 'use_camera_prior', 'camera_heading_every', 'orb_features_heading']


def save_trajectory(path, t, poses, status):
    with open(path, 'w') as f:
        f.write('t_s,x_m,y_m,yaw_rad,status\n')
        for ti, q, s in zip(t, poses, status):
            f.write(f'{ti:.4f},{q[0]:.4f},{q[1]:.4f},{q[2]:.5f},{s}\n')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--cache')
    ap.add_argument('--config')
    ap.add_argument('--no-camera-prior', action='store_true')
    ap.add_argument('--no-camera-loops', action='store_true')
    ap.add_argument('--frontend-only', action='store_true')
    ap.add_argument('--map-name', default='map')
    ap.add_argument('--max-time', type=float, help='stop after this many seconds of data (tests)')
    ap.add_argument('--reuse-frontend', action='store_true',
                    help='load frontend_state.pkl from out_dir instead of rerunning the LiDAR odometry')
    ap.add_argument('--reuse-loops', action='store_true',
                    help='re-apply the acceptance thresholds to loops.json instead of searching again '
                         '(only candidates that were verified before; needs --reuse-frontend)')
    a = ap.parse_args(argv)
    p = load_params(a.config)
    p['use_camera_prior'] &= not a.no_camera_prior
    p['use_camera_loops'] &= not a.no_camera_loops
    os.makedirs(a.out, exist_ok=True)
    from . import report
    timing = {}

    t0 = time.time()
    rec = Recording.load(a.bag, a.cache)
    cam_heading = camera_from(rec, p, p['orb_features_heading'])
    cam_loop = camera_from(rec, p, p['orb_features_loop'])
    image_t = rec.image_t - rec.t0
    track, cam_t, yaw_cam, cam_inl = camera_heading(rec, cam_heading, p, a.cache)
    timing['load_and_camera_heading_s'] = time.time() - t0
    print(f'camera heading verified for {np.isfinite(yaw_cam[1:]).mean():.1%} of frame pairs')

    t1 = time.time()
    state_path = os.path.join(a.out, 'frontend_state.pkl')
    if a.reuse_frontend and os.path.exists(state_path):
        with open(state_path, 'rb') as f:
            state = pickle.load(f)
        changed = {k for k in FRONTEND_KEYS if state['params'].get(k) != p[k]}
        if changed:
            raise SystemExit(f'saved front end used different settings: {sorted(changed)}')
        keyframes, recs = state['keyframes'], state['recs']
        print(f'reusing the saved front end ({len(keyframes)} keyframes)')
    else:
        odo, recs = run_frontend(rec, p, track if p['use_camera_prior'] else None, max_time=a.max_time)
        keyframes = odo.keyframes
        recs = [{k: v for k, v in r.items() if k != 'pts'} for r in recs]
        with open(state_path, 'wb') as f:
            pickle.dump({'params': p, 'keyframes': keyframes, 'recs': recs}, f)
        timing['lidar_odometry_s'] = time.time() - t1
    summary = {'params': p, 'frontend': frontend_summary(recs)}
    summary['camera_vs_lidar_heading'], (d_l, d_c) = heading_agreement(recs, cam_t, yaw_cam)
    report.plot_frontend(recs, keyframes, os.path.join(a.out, 'frontend.png'))
    report.plot_heading_check(d_l, d_c, os.path.join(a.out, 'camera_vs_lidar_heading.png'),
                              interval_s=float(np.median(np.diff(cam_t))))
    t_rec = np.array([r['t'] for r in recs])
    status = [r['status'] for r in recs]
    front_poses = np.array([r['pose'] for r in recs])
    save_trajectory(os.path.join(a.out, 'trajectory_frontend.csv'), t_rec, front_poses, status)

    kfs = keyframes
    kf_front = np.array([k['pose'] for k in kfs])
    loops, tried = [], []
    if not a.frontend_only:
        loops_path = os.path.join(a.out, 'loops.json')
        if p['use_camera_loops'] and a.reuse_loops and a.reuse_frontend and os.path.exists(loops_path):
            with open(loops_path) as f:
                tried = json.load(f)['tried']
            for r in tried:   # records from older runs lack some fields: they may only lose acceptance
                r['accepted'] = passes(r, p) and (r['accepted'] or 'degenerate' in r)
            loops = [r for r in tried if r['accepted']]
            print(f'reusing {len(tried)} loop candidates; {len(loops)} pass the current thresholds')
        elif p['use_camera_loops']:
            t2 = time.time()
            loops, tried, _ = find_loops(kfs, rec.images, image_t, cam_loop, p)
            timing['loop_closure_s'] = time.time() - t2
        summary['loop_closure'] = {'candidates': len(tried), 'camera_verified': sum(r['camera_ok'] for r in tried),
                                   'accepted': len(loops)}
        edges, kf_rec = odometry_edges(kfs, recs, p)
        L_loop = np.diag([1 / p['loop_sigma_xy']] * 2 + [1 / np.radians(p['loop_sigma_deg'])])
        loop_edges = [(l['j'], l['i'], np.array(l['z']), L_loop) for l in loops]
        t3 = time.time()
        if loop_edges:
            kf_opt, res = optimize(kf_front, edges + loop_edges, f_scale=p['graph_f_scale'])
            summary['pose_graph'] = {'keyframes': len(kfs), 'odometry_edges': len(edges),
                                     'loop_edges': len(loop_edges), 'cost_start': None,
                                     'cost_end': float(res.cost), 'iterations': int(res.nfev)}
            before, after = edge_errors(kf_front, loop_edges), edge_errors(kf_opt, loop_edges)
            summary['revisit_consistency'] = {
                'loop_error_before_m_p50_max': [float(np.median(before[:, 0])), float(before[:, 0].max())],
                'loop_error_before_deg_p50_max': [float(np.median(before[:, 1])), float(before[:, 1].max())],
                'loop_error_after_m_p50_max': [float(np.median(after[:, 0])), float(after[:, 0].max())],
                'loop_error_after_deg_p50_max': [float(np.median(after[:, 1])), float(after[:, 1].max())],
                'keyframe_shift_m_p50_max': [float(np.median(np.hypot(*(kf_opt - kf_front)[:, :2].T))),
                                             float(np.hypot(*(kf_opt - kf_front)[:, :2].T).max())]}
        else:
            kf_opt = kf_front.copy()
            summary['pose_graph'] = {'keyframes': len(kfs), 'loop_edges': 0}
        timing['pose_graph_s'] = time.time() - t3
        final = corrected_trajectory(recs, kf_rec, kf_front, kf_opt)
        save_trajectory(os.path.join(a.out, 'trajectory.csv'), t_rec, final, status)
        np.savetxt(os.path.join(a.out, 'keyframes.csv'), np.column_stack(
            [[k['t'] for k in kfs], kf_opt]), delimiter=',', header='t_s,x_m,y_m,yaw_rad', comments='', fmt='%.5f')
        save_json(os.path.join(a.out, 'loops.json'), {'accepted': loops, 'tried': tried})

        t5 = time.time()
        rows_front = consistency.check(kfs, kf_front, p)
        rows_final = consistency.check(kfs, kf_opt, p)
        summary['revisit_consistency_lidar_check'] = {
            'note': 'validation only: LiDAR ICP at keyframe pairs within 1 m, >30 s and >10 m of path apart',
            'lidar_odometry_only': consistency.summarize(rows_front),
            'final': consistency.summarize(rows_final)}
        save_json(os.path.join(a.out, 'consistency.json'), {'lidar_odometry_only': rows_front, 'final': rows_final})
        timing['consistency_check_s'] = time.time() - t5

        t4 = time.time()
        laser_xy = rec.laser_pose2d()[:2]
        grid_before = build_map(kfs, kf_front, laser_xy, p)
        grid = build_map(kfs, kf_opt, laser_xy, p)
        st = grid.save(a.out, a.map_name)
        timing['map_s'] = time.time() - t4
        summary['map'] = {'file': f'{a.map_name}.yaml', 'resolution_m': p['map_resolution'],
                          'size_cells': list(grid.shape), 'origin_m': grid.origin.tolist(),
                          'occupied_cells': int((st == 1).sum()), 'free_cells': int((st == 0).sum()),
                          'occupied_cells_before_loop_closure': int((grid_before.states() == 1).sum())}
        report.plot_map(grid, final, t_rec, kf_opt, loops, os.path.join(a.out, f'{a.map_name}.png'))
        report.plot_before_after(grid_before, grid, kf_front, kf_opt, loops,
                                 os.path.join(a.out, 'loop_closure_before_after.png'))
        report.plot_tracking(recs, cam_t, yaw_cam, cam_inl, loops, os.path.join(a.out, 'tracking.png'))
    summary['timing_s'] = {k: round(v, 1) for k, v in timing.items()}
    save_json(os.path.join(a.out, 'summary.json'), summary)
    print(json.dumps(clean({k: v for k, v in summary.items() if k != 'params'}), indent=1))


if __name__ == '__main__':
    main()
