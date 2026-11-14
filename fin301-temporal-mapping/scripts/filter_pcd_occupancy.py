"""Export one experimental fixed-10cm occupancy-supported LiDAR PCD subset."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from PIL import Image
import yaml

from indoor_mapping.map_filter import read_ascii_pcd, write_pcd_subset
from indoor_mapping.occupancy_filter import filter_occupancy_supported


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def plots(points, keep, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    low, high = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
    margin = np.maximum(high-low, 1)*.03
    full = [low[0]-margin[0], high[0]+margin[0], low[1]-margin[1], high[1]+margin[1]]
    removed = points[~keep, :2]
    if len(removed):
        tiles, counts = np.unique(np.floor(removed/2).astype(int), axis=0, return_counts=True)
        tile = tiles[np.argmax(counts)]*2
        detail = [tile[0]-3, tile[0]+5, tile[1]-3, tile[1]+5]
    else:
        detail = full
    for filename, limits, title in [
        ('occupancy_supported_before_after.png', full, 'Experimental occupancy agreement | original and fixed 0.10m support'),
        ('occupancy_supported_detail.png', detail, 'Same crop around highest-removal 2m tile | no coordinate changes')]:
        fig, axes = plt.subplots(1, 2, figsize=(13, 7), sharex=True, sharey=True)
        for ax, name, mask in zip(axes, ('Original LiDAR', 'Occupancy-supported candidate'), (np.ones(len(points), bool), keep)):
            ax.scatter(points[mask, 0], points[mask, 1], s=.7 if limits is full else 2.,
                       color='#234b60', edgecolors='none', linewidths=0, rasterized=True)
            ax.set(xlim=limits[:2], ylim=limits[2:], xlabel='Map x (m)', ylabel='Map y (m)')
            ax.set_title(f'{name}: {mask.sum():,} points'); ax.set_aspect('equal'); ax.grid(alpha=.15)
        fig.suptitle(title+'\nGrid uses the same observations; excluded points are not asserted free')
        fig.tight_layout(); fig.savefig(output/filename, dpi=170); plt.close(fig)
    return {'detail_bounds_xy_m': [float(value) for value in detail],
            'detail_selection': '8m square around the 2m tile with the largest removal count',
            'same_point_size_and_axis_scale_before_after': True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('outputs/fin301/result/lidar_scan_map.pcd'))
    parser.add_argument('--yaml', type=Path, default=Path('outputs/fin301/result/map.yaml'))
    parser.add_argument('--output', type=Path, default=Path('outputs/fin301/filtering'))
    args = parser.parse_args(argv)
    if args.output.resolve() in (args.source.resolve().parent, args.yaml.resolve().parent):
        parser.error('Use a separate output directory; original artifacts must remain intact')
    args.output.mkdir(parents=True, exist_ok=True)
    metadata = yaml.safe_load(args.yaml.read_text(encoding='utf-8'))
    image_path = args.yaml.parent/metadata['image']
    sources = {'source_pcd': args.source, 'source_yaml': args.yaml, 'source_image': image_path,
               'filter_module': ROOT/'indoor_mapping/occupancy_filter.py',
               'exact_subset_io_module': ROOT/'indoor_mapping/map_filter.py', 'script': Path(__file__)}
    hashes = {name: digest(path) for name, path in sources.items()}
    points, header, rows = read_ascii_pcd(args.source)
    if not len(points):
        parser.error('Source PCD is empty')
    with Image.open(image_path) as image:
        if image.mode not in ('L', 'P'):
            raise ValueError('Expected original 8-bit grayscale occupancy image')
        pixels = np.asarray(image.convert('L'))
    retained, stats, keep = filter_occupancy_supported(points, pixels, metadata)
    target = args.output/'lidar_scan_map_clean_occupancy_supported.pcd'
    write_pcd_subset(target, header, rows, keep)
    parsed, _, selected_rows = read_ascii_pcd(target)
    assert np.array_equal(parsed, retained)
    assert selected_rows == [row for row, yes in zip(rows, keep) if yes]
    mask = args.output/'keep_mask_occupancy_supported.npy'
    np.save(mask, keep)
    stats.update(file=target.name, sha256=digest(target), keep_mask=mask.name,
                 keep_mask_sha256=digest(mask), source_paths={name: str(path) for name, path in sources.items()},
                 source_sha256=hashes, frame='first_laser_pose', units='metres',
                 original_ascii_data_rows_preserved=True, visual_landmarks='unchanged and excluded',
                 original_grid='unchanged; retained as the original occupancy context',
                 plotting=plots(points, keep, args.output))
    if any(digest(path) != hashes[name] for name, path in sources.items()):
        raise RuntimeError('A source artifact or module changed during export')
    (args.output/'occupancy_filter_report.json').write_text(json.dumps(stats, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'retained': len(retained), 'removed': int((~keep).sum()),
                      'retention_by_original_cell_class': stats['retention_by_original_cell_class'],
                      'file': str(target)}, indent=2))
    return stats


if __name__ == '__main__':
    main()
