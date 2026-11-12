#!/usr/bin/env python3
"""Propose places where a map width is easy to check with a tape measure.

Samples the estimated trajectory every few seconds; at each point it finds
the direction with the shortest free span through the point (perpendicular
to the walls in a corridor) and keeps points whose span is corridor-like and
whose two ends are long straight walls. Prints the candidates and writes a
targets file for validation_targets.py.

Usage: python3 -I find_width_targets.py <run_dir> <map_name> <out_targets.json> [n_targets]
"""
import json
import os
import sys

import numpy as np
import yaml


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        end = data.index(b'\n', pos)
        line = data[pos:end]
        pos = end + 1
        if not line.startswith(b'#'):
            tokens += line.split()
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[pos:pos + w * h], np.uint8).reshape(h, w)[::-1]


def main():
    run, name, out = sys.argv[1:4]
    n_targets = int(sys.argv[4]) if len(sys.argv) > 4 else 3
    meta = yaml.safe_load(open(os.path.join(run, f'{name}.yaml')))
    img = read_pgm(os.path.join(run, meta['image']))
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]
    occ, free = img == 0, img == 254
    traj = np.genfromtxt(os.path.join(run, 'trajectory.csv'), delimiter=',', names=True, dtype=None, encoding=None)

    def cell(x, y):
        return int(np.floor((y - oy) / res)), int(np.floor((x - ox) / res))

    def span(x, y, th, max_m=7.0):
        u = np.array([np.cos(th), np.sin(th)])
        ends = []
        for sgn in (1, -1):
            s, hit = 0.0, False
            while s < max_m:
                i, j = cell(x + sgn * s * u[0], y + sgn * s * u[1])
                if not (0 <= i < occ.shape[0] and 0 <= j < occ.shape[1]) or not free[i, j] and not occ[i, j]:
                    break                                  # left the map or reached unknown space
                if occ[i, j]:
                    hit = True
                    break
                s += 0.01
            ends.append((s, hit))
        return ends

    occ_ij = np.argwhere(occ)
    occ_xy = np.column_stack([ox + (occ_ij[:, 1] + 0.5) * res, oy + (occ_ij[:, 0] + 0.5) * res])

    def wall_lines(x, y, th, half_len=1.5, max_w=6.0):
        """Fit a straight line to each wall within +/-half_len along the corridor.

        Returns (width between the two parallel lines, angle between them in deg, RMS of the
        fits) or None when either side lacks a straight wall.
        """
        nrm, alg = np.array([np.cos(th), np.sin(th)]), np.array([-np.sin(th), np.cos(th)])
        d = occ_xy - [x, y]
        a, s = d @ nrm, d @ alg
        keep = (np.abs(s) <= half_len) & (np.abs(a) <= max_w / 2 + 0.3)
        a, s = a[keep], s[keep]
        fits = []
        for side in (1, -1):
            m = side * a > 0.05
            if m.sum() < 20:
                return None
            band = m & (side * a < np.min(side * a[m]) + 0.15)      # the nearest wall on this side
            if band.sum() < 20 or np.ptp(s[band]) < 1.5:
                return None
            A = np.column_stack([s[band], np.ones(band.sum())])
            coef, *_ = np.linalg.lstsq(A, a[band], rcond=None)
            r = a[band] - A @ coef
            ok = np.abs(r) < 0.05
            coef, *_ = np.linalg.lstsq(A[ok], a[band][ok], rcond=None)
            rms = float(np.sqrt(np.mean((a[band][ok] - A[ok] @ coef) ** 2)))
            fits.append((coef, rms))
        (c1, r1), (c2, r2) = fits
        ang = abs(np.degrees(np.arctan(c1[0]) - np.arctan(c2[0])))
        width = abs(c1[1] - c2[1]) * np.cos(0.5 * (np.arctan(c1[0]) + np.arctan(c2[0])))
        return float(width), float(ang), max(r1, r2)

    cands = []
    for k in range(0, len(traj), 15):              # every 15 processed scans (about 1 s)
        x, y, t = float(traj['x_m'][k]), float(traj['y_m'][k]), float(traj['t_s'][k])
        best = None
        for th in np.radians(np.arange(0, 180, 2)):
            (a, ha), (b, hb) = span(x, y, th)
            if ha and hb and (best is None or a + b < best[0]):
                best = (a + b, th, a, b)
        if best is None or not (1.0 <= best[0] <= 6.0):
            continue
        _, th, a, b = best
        fit = wall_lines(x, y, th)
        if fit is not None and fit[1] < 3.0 and fit[2] < 0.03 and abs(fit[0] - (a + b)) < 0.15:
            u = np.array([np.cos(th), np.sin(th)])
            c = np.array([x, y]) + 0.5 * (a - b) * u          # midway between the walls
            cands.append({'t_s': t, 'centre': [round(float(c[0]), 3), round(float(c[1]), 3)],
                          'direction_deg': round(float(np.degrees(th)), 1), 'width_m': round(fit[0], 3),
                          'walls_angle_deg': round(fit[1], 2), 'fit_rms_m': round(fit[2], 4)})
    print(f'{len(cands)} corridor-like candidates')
    chosen = []
    for c in sorted(cands, key=lambda c: c['t_s']):
        if all(np.hypot(c['centre'][0] - d['centre'][0], c['centre'][1] - d['centre'][1]) > 5.0 for d in chosen):
            chosen.append(c)
    for c in chosen:
        print(c)
    step = max(1, len(chosen) // n_targets)
    picked = chosen[::step][:n_targets]
    targets = [{'name': chr(65 + k), 'what': 'distance between the two parallel walls', 'centre': c['centre'],
                'direction_deg': c['direction_deg'], 'line_fit_width_m': c['width_m'],
                'walls_angle_deg': c['walls_angle_deg'], 'fit_rms_m': c['fit_rms_m']}
               for k, c in enumerate(picked)]
    with open(out, 'w') as f:
        json.dump(targets, f, indent=1)
    print(json.dumps(targets, indent=1))


if __name__ == '__main__':
    main()
