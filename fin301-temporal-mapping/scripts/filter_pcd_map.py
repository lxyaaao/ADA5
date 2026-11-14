"""Create reversible, exact-subset LiDAR density-cleanup candidates."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial import cKDTree

from indoor_mapping.map_filter import PRESETS, filter_points, read_ascii_pcd, write_pcd_subset


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def spatial_removal(points, keep, tile_m=2.):
    tiles = np.floor(points[:, :2]/tile_m).astype(np.int64)
    unique, inverse, total = np.unique(tiles, axis=0, return_inverse=True, return_counts=True)
    removed = np.bincount(inverse[~keep], minlength=len(unique))
    order = np.argsort(-removed, kind='stable')[:20]
    return [{'bounds_xy_m': [float(unique[i, 0]*tile_m), float(unique[i, 1]*tile_m),
                             float((unique[i, 0]+1)*tile_m), float((unique[i, 1]+1)*tile_m)],
             'input_points': int(total[i]), 'removed_points': int(removed[i]),
             'removed_fraction': float(removed[i]/total[i])}
            for i in order if removed[i]]


def diagnostic_plots(points, candidates, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    minxy, maxxy = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
    extent = np.maximum(maxxy-minxy, 1.)
    limits = [minxy[0]-.03*extent[0], maxxy[0]+.03*extent[0],
              minxy[1]-.03*extent[1], maxxy[1]+.03*extent[1]]
    columns = 3 if len(candidates) > 3 else 2
    rows = int(np.ceil((len(candidates)+1)/columns))
    fig, axes = plt.subplots(rows, columns, figsize=(6*columns, 6*rows), sharex=True, sharey=True)
    axes = axes.ravel()
    plots = [('Original LiDAR', np.ones(len(points), bool))]+list(candidates.items())
    for ax, (name, keep) in zip(axes, plots):
        ax.scatter(points[keep, 0], points[keep, 1], s=.5, c='#234b60', edgecolors='none', linewidths=0, rasterized=True)
        ax.set_title(f'{name}: {keep.sum():,} points; removed {(~keep).sum():,}')
        ax.set(xlim=limits[:2], ylim=limits[2:], xlabel='Map x (m)', ylabel='Map y (m)')
        ax.set_aspect('equal'); ax.grid(alpha=.15)
    for ax in axes[len(plots):]:
        ax.set_visible(False)
    fig.suptitle('Density cleanup candidates | identical metric scale; no coordinate movement')
    fig.tight_layout(); fig.savefig(output/'before_after_equal_axes.png', dpi=170); plt.close(fig)
    fig, axes = plt.subplots(1, len(candidates), figsize=(5*len(candidates), 7), sharex=True, sharey=True)
    for ax, (name, keep) in zip(np.atleast_1d(axes), candidates.items()):
        ax.scatter(points[:, 0], points[:, 1], s=.45, color='#c0c8cc', edgecolors='none', linewidths=0, rasterized=True)
        ax.scatter(points[~keep, 0], points[~keep, 1], s=3., color='#bc3e35', edgecolors='none', linewidths=0, rasterized=True)
        ax.set_title(f'{name}: removed points in red')
        ax.set(xlim=limits[:2], ylim=limits[2:], xlabel='Map x (m)', ylabel='Map y (m)')
        ax.set_aspect('equal'); ax.grid(alpha=.15)
    fig.suptitle('Removed points are candidates for exclusion, not confirmed false obstacles')
    fig.tight_layout(); fig.savefig(output/'removed_points_diagnostic.png', dpi=170); plt.close(fig)
    # Fixed crop for every method: choose the 2m tile with the largest balanced
    # removal count, then show surrounding context, not a hand-selected success.
    preferred_name = 'radius_balanced' if 'radius_balanced' in candidates else 'balanced'
    preferred = candidates.get(preferred_name, next(iter(candidates.values())))
    tiles = spatial_removal(points, preferred)
    if tiles:
        x0, y0, x1, y1 = tiles[0]['bounds_xy_m']
        crop = [x0-2, x1+2, y0-2, y1+2]
        fig, axes = plt.subplots(rows, columns, figsize=(5*columns, 5*rows), sharex=True, sharey=True)
        axes = np.atleast_1d(axes).ravel()
        for ax, (name, keep) in zip(axes, plots):
            ax.scatter(points[keep, 0], points[keep, 1], s=2.5, c='#234b60', edgecolors='none', linewidths=0)
            ax.set(xlim=crop[:2], ylim=crop[2:], xlabel='Map x (m)', ylabel='Map y (m)')
            ax.set_title(name); ax.set_aspect('equal'); ax.grid(alpha=.15)
        for ax in axes[len(plots):]:
            ax.set_visible(False)
        fig.suptitle(f'Same crop around highest {preferred_name} removal 2m tile; dense clutter may remain')
        fig.tight_layout(); fig.savefig(output/'detail_equal_axes.png', dpi=170); plt.close(fig)
    if 'radius_balanced' in candidates:
        fig, axes = plt.subplots(1, 2, figsize=(13, 7), sharex=True, sharey=True)
        for ax, (name, keep) in zip(axes, [plots[0], ('radius_balanced', candidates['radius_balanced'])]):
            ax.scatter(points[keep, 0], points[keep, 1], s=.7, color='#234b60', edgecolors='none', linewidths=0, rasterized=True)
            ax.set_title(f'{name}: {keep.sum():,} points')
            ax.set(xlim=limits[:2], ylim=limits[2:], xlabel='Map x (m)', ylabel='Map y (m)')
            ax.set_aspect('equal'); ax.grid(alpha=.15)
        fig.suptitle('Plain radius comparison | same axes, exact subsets; sparse real features may be removed')
        fig.tight_layout(); fig.savefig(output/'radius_balanced_before_after.png', dpi=170); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('outputs/fin301/result/lidar_scan_map.pcd'))
    parser.add_argument('--output', type=Path, default=Path('outputs/fin301/filtering'))
    parser.add_argument('--presets', nargs='+', choices=list(PRESETS), default=list(PRESETS))
    args = parser.parse_args(argv)
    if args.output.resolve() == args.source.resolve().parent:
        parser.error('Use a separate output directory; original result artifacts must remain intact')
    args.output.mkdir(parents=True, exist_ok=True)
    source_hash = file_hash(args.source)
    points, header, data_rows = read_ascii_pcd(args.source)
    if not len(points):
        parser.error('Source map is empty')
    report = {'source': str(args.source), 'source_sha256': source_hash,
              'input_points': len(points), 'units': 'metres', 'frame': 'first_laser_pose',
              'operation': 'Exact ASCII row subset; no averaging, smoothing, fitting, or coordinate motion',
              'occupancy_grid': 'Unchanged original grid; point removal does not establish free space',
              'camera_landmarks': 'Unchanged and excluded from filtering',
              'selection': 'Unselected development candidates; localization evaluation required',
              'bounds_min_xyz_m': points.min(axis=0).tolist(),
              'bounds_max_xyz_m': points.max(axis=0).tolist(),
              'source_code_sha256': {str(path): file_hash(path) for path in
                                     [Path('indoor_mapping/map_filter.py'), Path(__file__)]},
              'candidates': {}}
    candidates = {}
    for preset in args.presets:
        retained, stats, keep = filter_points(points, preset)
        name = f'lidar_scan_map_clean_{preset}.pcd'
        target = write_pcd_subset(args.output/name, header, data_rows, keep)
        reread, _, rows = read_ascii_pcd(target)
        if not np.array_equal(reread, retained) or rows != [r for r, k in zip(data_rows, keep) if k]:
            raise RuntimeError('Exact-subset verification failed')
        mask_path = args.output/f'keep_mask_{preset}.npy'
        np.save(mask_path, keep)
        stats.update(file=name, sha256=file_hash(target), keep_mask=mask_path.name,
                     keep_mask_sha256=file_hash(mask_path),
                     original_row_indices_preserved=True,
                     top_removed_2m_tiles=spatial_removal(points, keep))
        report['candidates'][preset] = stats
        candidates[preset] = keep
        print(f'{preset}: {len(points):,} -> {len(retained):,}; removed {stats["removed_points"]:,}; line-protected {stats["line_protected_points"]:,}', flush=True)
    diagnostic_plots(points, candidates, args.output)
    if source_hash != file_hash(args.source):
        raise RuntimeError('Source map unexpectedly changed')
    (args.output/'filter_report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return report


if __name__ == '__main__':
    main()
