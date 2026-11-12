#!/usr/bin/env python3
"""Draw an overview figure from inspect_bag.py output.

Usage: python3 plot_inspection.py <inspection_dir>
"""
import glob
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np


def main():
    d = sys.argv[1]
    summary = json.load(open(os.path.join(d, 'bag_inspection.json')))
    scans = json.load(open(os.path.join(d, 'scan_samples.json')))
    images = sorted(glob.glob(os.path.join(d, 'samples', 'image_*')))

    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(2, 4)

    ax = fig.add_subplot(gs[0, 0:2])
    odom_path = os.path.join(d, 'odom_xy_context_only.npy')
    if os.path.exists(odom_path):
        a = np.load(odom_path)
        sc = ax.scatter(a[:, 1], a[:, 2], c=a[:, 0] - a[0, 0], s=2, cmap='viridis')
        ax.plot(a[0, 1], a[0, 2], 'g^', ms=10, label='start')
        ax.plot(a[-1, 1], a[-1, 2], 'rs', ms=8, label='end')
        fig.colorbar(sc, ax=ax, label='time since start (s)')
        ax.legend()
    ax.set_title('Recorded /odom (context only, not used for estimation)')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.axis('equal'); ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[0, 2])
    map_path = os.path.join(d, 'recorded_map.npy')
    meta = summary.get('recorded_map')
    if os.path.exists(map_path) and meta:
        g = np.load(map_path).astype(float)
        img = np.where(g < 0, 0.5, 1.0 - g / 100.0)
        res, (ox, oy) = meta['resolution_m'], meta['origin_xy_m']
        ext = [ox, ox + g.shape[1] * res, oy, oy + g.shape[0] * res]
        ax.imshow(img, cmap='gray', origin='lower', extent=ext, vmin=0, vmax=1)
        ax.set_title(f"Recorded /map ({meta['width']}x{meta['height']} @ {res} m)")
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')

    ax = fig.add_subplot(gs[0, 3])
    for k, s in scans.items():
        r = np.asarray(s['ranges'], dtype=float)
        th = s['angle_min'] + s['angle_increment'] * np.arange(r.size)
        ok = np.isfinite(r) & (r > 0)
        ax.scatter(r[ok] * np.cos(th[ok]), r[ok] * np.sin(th[ok]), s=1, label=f'scan {k}')
    ax.plot(0, 0, 'k^')
    ax.set_title('/scan samples in the laser frame'); ax.axis('equal'); ax.grid(alpha=0.3)
    ax.legend(markerscale=6, fontsize=8)

    for i, path in enumerate(images[:4]):
        ax = fig.add_subplot(gs[1, i])
        ax.imshow(mpimg.imread(path))
        ax.set_title(os.path.basename(path), fontsize=9)
        ax.axis('off')

    fig.tight_layout()
    out = os.path.join(d, 'inspection_overview.png')
    fig.savefig(out, dpi=110)
    print(out)


if __name__ == '__main__':
    main()
