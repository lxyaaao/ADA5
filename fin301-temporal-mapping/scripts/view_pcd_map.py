"""Draw the exact XYZ points from a planar ASCII PCD in metric map coordinates."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault('MPLCONFIGDIR', str(ROOT / '.mplconfig'))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from indoor_mapping.ros_node import load_ascii_pcd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    points = load_ascii_pcd(args.pcd)
    if not len(points) or not np.allclose(points[:, 2], 0):
        raise ValueError('Expected a nonempty LiDAR scan-plane PCD with z=0')
    fig, ax = plt.subplots(figsize=(9, 9), layout='constrained')
    ax.scatter(points[:, 0], points[:, 1], s=1.1, color='#243e50',
               edgecolors='none', rasterized=True)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('Map x (m)', fontsize=11)
    ax.set_ylabel('Map y (m)', fontsize=11)
    ax.set_title(f'fin301 | Localization PCD — top view\n{len(points):,} points; z = 0 m',
                 fontsize=14, pad=15)
    ax.grid(alpha=.16, linewidth=.7)
    ax.set_axisbelow(True)
    ax.margins(.035)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=200, facecolor='white')
    plt.close(fig)
    evidence = {'source_pcd': str(args.pcd),
                'source_sha256': hashlib.sha256(args.pcd.read_bytes()).hexdigest(),
                'point_count': len(points), 'points_plotted': len(points),
                'xyz_min_m': points.min(axis=0).tolist(),
                'xyz_max_m': points.max(axis=0).tolist(),
                'equal_axis_scale': True, 'filtered_or_rotated': False,
                'figure': str(args.output)}
    args.output.with_suffix('.json').write_text(json.dumps(evidence, indent=2), encoding='utf-8')
    print(json.dumps(evidence, indent=2))


if __name__ == '__main__':
    main()
