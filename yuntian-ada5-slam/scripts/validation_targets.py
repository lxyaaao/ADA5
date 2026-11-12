#!/usr/bin/env python3
"""Map distances to check against tape measurements in FinTech 301.

For each target (a centre point and a direction in the map frame) the
script walks from the centre in both directions until it reaches an
occupied cell and reports the free width between the two surfaces. It also
saves the camera frame taken nearest to the centre so the same place can be
found on site. Widths are between the inner faces of the occupied cells, so
the map value carries about +/-0.05 m from the 0.05 m cells alone.

Usage: python3 -I validation_targets.py <run_dir> <cache_dir> <targets.json> <map_name> <out_dir>
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
import yaml

from ada5_slam.recording import Recording


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:                     # magic, width, height, maxval
        end = data.index(b'\n', pos)
        line = data[pos:end]
        pos = end + 1
        if not line.startswith(b'#'):
            tokens += line.split()
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[pos:pos + w * h], np.uint8).reshape(h, w)


def main():
    run, cache, targets_file, name, out = sys.argv[1:6]
    os.makedirs(out, exist_ok=True)
    meta = yaml.safe_load(open(os.path.join(run, f'{name}.yaml')))
    img = read_pgm(os.path.join(run, meta['image']))[::-1]          # row 0 = smallest y
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]
    occ = img == 0
    traj = np.genfromtxt(os.path.join(run, 'trajectory.csv'), delimiter=',', names=True, dtype=None,
                         encoding=None)
    rec = Recording.load(None, cache)
    image_t = rec.image_t - rec.t0
    targets = json.load(open(targets_file))

    def occupied(x, y):
        i, j = int(np.floor((y - oy) / res)), int(np.floor((x - ox) / res))
        return 0 <= i < occ.shape[0] and 0 <= j < occ.shape[1] and occ[i, j]

    results = []
    fig, axs = plt.subplots(len(targets), 2, figsize=(13, 4.6 * len(targets)), squeeze=False)
    for k, tg in enumerate(targets):
        cx, cy = tg['centre']
        u = np.array([np.cos(np.radians(tg['direction_deg'])), np.sin(np.radians(tg['direction_deg']))])
        reach = []
        for sgn in (1, -1):
            s = 0.0
            while s < tg.get('max_m', 15.0) and not occupied(cx + sgn * s * u[0], cy + sgn * s * u[1]):
                s += 0.01
            reach.append(s)
        ray_width = reach[0] + reach[1]
        width = tg.get('line_fit_width_m', ray_width)   # wall-line fit when available (robust to gaps)
        p1, p2 = np.array([cx, cy]) + reach[0] * u, np.array([cx, cy]) - reach[1] * u
        n = int(np.argmin(np.hypot(traj['x_m'] - cx, traj['y_m'] - cy)))
        t_near = float(traj['t_s'][n])
        f = int(np.argmin(np.abs(image_t - t_near)))
        frame = cv2.cvtColor(cv2.imdecode(np.frombuffer(rec.images[f], np.uint8), cv2.IMREAD_COLOR),
                             cv2.COLOR_BGR2RGB)
        cv2.imwrite(os.path.join(out, f'target_{tg["name"]}_frame.jpg'), frame[:, :, ::-1])
        results.append({'name': tg['name'], 'what': tg['what'], 'centre_m': [cx, cy],
                        'direction_deg': tg['direction_deg'], 'map_width_m': round(width, 3),
                        'ray_width_m': round(ray_width, 3),
                        'walls_angle_deg': tg.get('walls_angle_deg'), 'fit_rms_m': tg.get('fit_rms_m'),
                        'endpoints_m': [p1.round(3).tolist(), p2.round(3).tolist()],
                        'cell_uncertainty_m': 2 * res / 2,
                        'nearest_pass_s': round(t_near, 1),
                        'vehicle_distance_from_centre_m': round(float(np.hypot(traj['x_m'][n] - cx,
                                                                                traj['y_m'][n] - cy)), 2)})
        ax = axs[k, 0]
        pad = max(width, 3.0)
        ext = [ox, ox + occ.shape[1] * res, oy, oy + occ.shape[0] * res]
        shade = np.where(img == 0, 0.0, np.where(img == 254, 1.0, 0.78))
        ax.imshow(shade, cmap='gray', origin='lower', extent=ext, vmin=0, vmax=1, interpolation='nearest')
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], '-', color='red', lw=2)
        ax.plot(traj['x_m'][n], traj['y_m'][n], 'b^', ms=9)
        ax.set_xlim(cx - pad, cx + pad)
        ax.set_ylim(cy - pad, cy + pad)
        ax.set_aspect('equal')
        ax.set_title(f'{tg["name"]}: {tg["what"]} = {width:.2f} m in the map (blue: vehicle at {t_near:.0f} s)',
                     fontsize=10)
        axs[k, 1].imshow(frame)
        axs[k, 1].set_title(f'camera at {t_near:.0f} s (to find the place)', fontsize=10)
        axs[k, 1].axis('off')
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'validation_targets.png'), dpi=90)
    with open(os.path.join(out, 'validation_targets.json'), 'w') as f:
        json.dump(results, f, indent=1)
    print(json.dumps(results, indent=1))


if __name__ == '__main__':
    main()
