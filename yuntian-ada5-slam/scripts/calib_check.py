#!/usr/bin/env python3
"""Check the camera calibration and the camera-to-LiDAR mount (Lecture 4).

Projects /scan points into the raw camera images using camera_info (K, D)
and the /tf_static mounts, for a few frames with close structure, and
summarises how images pair with scans by timestamp.

Usage: python3 -I calib_check.py <bag_dir> <cache_dir> <out_dir>
"""
import os
import sys

sys.path[:0] = [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep)
                if p.startswith('/opt/ros/')]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'ada5_slam'))

import json

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from ada5_slam.recording import Recording

TIMES = [3.8, 62.0, 181.0, 330.0, 392.0, 452.0]  # s after the start, frames with close structure


def project(r, ang, R_bl, t_bl, R_bc, t_bc, K, D):
    ok = np.isfinite(r) & (r > 0.1)
    pl = np.column_stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]), np.zeros(ok.sum())])
    pc = (pl @ R_bl.T + t_bl - t_bc) @ R_bc          # laser -> base_link -> camera optical frame
    n = pc[:, :2] / np.maximum(pc[:, 2:3], 1e-6)
    keep = (pc[:, 2] > 0.2) & (np.abs(n[:, 0]) < 0.9) & (np.abs(n[:, 1]) < 0.75)
    uv, _ = cv2.projectPoints(pc[keep].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
    return uv.reshape(-1, 2), pc[keep, 2]


def main():
    bag, cache, out = sys.argv[1:4]
    os.makedirs(out, exist_ok=True)
    rec = Recording.load(bag, cache)
    K, D = rec.camera_info['K'], rec.camera_info['D']
    w, h = rec.camera_info['width'], rec.camera_info['height']
    R_bc, t_bc = rec.mount('base_link', rec.camera_info['frame_id'])
    R_bl, t_bl = rec.mount('base_link', rec.scan_meta['frame_id'])
    ang = rec.angles
    fwd = int(np.argmin(np.abs(ang)))
    fwd_t = rec.scan_t + fwd * rec.scan_meta['time_increment']  # forward beam measured here

    i = np.clip(np.searchsorted(fwd_t, rec.image_t), 1, len(fwd_t) - 1)
    near = np.where(np.abs(fwd_t[i - 1] - rec.image_t) <= np.abs(fwd_t[i] - rec.image_t), i - 1, i)
    off_ms = 1e3 * (rec.image_t - fwd_t[near])
    laser_in_cam = (t_bl - t_bc) @ R_bc
    stats = {
        'images': int(len(rec.image_t)), 'scans': int(len(rec.scan_t)),
        'forward_beam_delay_ms': 1e3 * fwd * rec.scan_meta['time_increment'],
        'image_minus_nearest_scan_ms': {
            'median': float(np.median(off_ms)), 'p95_abs': float(np.percentile(np.abs(off_ms), 95)),
            'max_abs': float(np.abs(off_ms).max())},
        'laser_origin_in_camera_frame_m': laser_in_cam.tolist(),
        'scan_plane_above_camera_m': float(-laser_in_cam[1]),
        'K': K.tolist(), 'D': D.tolist(),
    }

    fig, axs = plt.subplots(2, 3, figsize=(18, 9.6))
    for ax, t_rel in zip(axs.ravel(), TIMES):
        k = int(np.argmin(np.abs(rec.image_t - rec.t0 - t_rel)))
        s = near[k]
        img = cv2.cvtColor(cv2.imdecode(np.frombuffer(rec.images[k], np.uint8), cv2.IMREAD_COLOR),
                           cv2.COLOR_BGR2RGB)
        uv, depth = project(rec.ranges[s], ang, R_bl, t_bl, R_bc, t_bc, K, D)
        ax.imshow(img)
        sc = ax.scatter(uv[:, 0], uv[:, 1], c=np.clip(depth, 0, 6), cmap='turbo', s=6, vmin=0, vmax=6)
        ax.axhline(K[1, 2], color='w', ls='--', lw=0.7)
        ax.set_xlim(0, w)
        ax.set_ylim(h, 0)
        ax.set_title(f't = {t_rel:.0f} s, scan offset {off_ms[k]:+.1f} ms, {len(uv)} points', fontsize=10)
        ax.axis('off')
    fig.colorbar(sc, ax=axs, shrink=0.6, label='distance along the optical axis (m)')
    fig.suptitle('LiDAR scan plane projected into the camera image (K, D from camera_info; mounts from '
                 '/tf_static). Dashed: principal row. The plane is '
                 f'{stats["scan_plane_above_camera_m"]:.3f} m above the camera, so points sit just above it.',
                 fontsize=11)
    fig.savefig(os.path.join(out, 'calib_projection.png'), dpi=90, bbox_inches='tight')
    with open(os.path.join(out, 'calib_check.json'), 'w') as f:
        json.dump(stats, f, indent=1)
    print(json.dumps(stats, indent=1))


if __name__ == '__main__':
    main()
