"""Plot evaluated query locations and multi-start stability, without ground-truth claims."""
from pathlib import Path
import argparse
import json
import os
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault('MPLCONFIGDIR', str(Path(__file__).resolve().parents[1] / '.mplconfig'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from indoor_mapping.ros_node import load_ascii_pcd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation', type=Path, required=True)
    parser.add_argument('--pcd', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    evidence = json.loads(args.evaluation.read_text(encoding='utf-8'))
    points = load_ascii_pcd(args.pcd)
    if not np.allclose(points[:, 2], 0):
        raise ValueError('Use the LiDAR scan-plane PCD, not projected visual landmarks')
    fig, axes = plt.subplots(1, 2, figsize=(11, 6.2), constrained_layout=True)
    for ax, group, title in zip(axes, ['queries', 'crosspass_queries'],
                               ['Separated reference observations', 'Earlier-pass reference observations']):
        queries = evidence[group]
        ax.scatter(points[:, 0], points[:, 1], s=.5, color='#b5bec6', rasterized=True)
        for stable, color, marker, label in [(True, '#00876c', 'o', 'All 8 starts agree'),
                                              (False, '#c54443', 'x', 'Rejected or inconsistent starts')]:
            chosen = [q for q in queries if q['agreement']['all_starts_agree'] == stable]
            if chosen:
                xy = np.array([q['seed_pose'][:2] for q in chosen])
                ax.scatter(xy[:, 0], xy[:, 1], s=29, color=color, marker=marker,
                           label=f'{label}: {len(chosen)}/{len(queries)}', linewidths=1.3)
        ax.set_aspect('equal')
        ax.set_xlabel('Map x (m)')
        ax.set_ylabel('Map y (m)')
        ax.set_title(title, fontsize=11)
        ax.legend(fontsize=8, loc='upper left')
        ax.grid(alpha=.15)
    fig.suptitle('Held-out scan localization stability: agreement within 0.15 m / 3 degrees', fontsize=12)
    fig.supxlabel('Query locations use estimated baseline seeds. Internal consistency only; no independent position ground truth.', fontsize=9)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    plt.close(fig)


if __name__ == '__main__':
    main()
