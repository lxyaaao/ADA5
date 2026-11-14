"""Plot frozen loop-refinement graphs without changing experiment sources."""
import argparse
import hashlib
import json
from pathlib import Path
import os

import numpy as np


def plot(directory):
    directory = Path(directory)
    os.environ.setdefault('MPLCONFIGDIR', str(directory/'plot_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    trajectory = directory/'proposed_trajectory.npz'
    graph_path = directory/'proposed_pose_graph.json'
    summary_path = directory/'summary.json'
    with np.load(trajectory) as data:
        current, proposed, keys = data['current_final'], data['proposed'], data['key_ids']
        seconds = (data['times_ns']-data['times_ns'][0]).astype(float)*1e-9
    graph = json.loads(graph_path.read_text())
    loops = [edge for edge in graph['edges'] if edge['loop']]
    summary = json.loads(summary_path.read_text())
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    axes[0].plot(current[:, 0], current[:, 1], label='Delivered one-loop trajectory', color='#71838a', linewidth=1.)
    axes[0].plot(proposed[:, 0], proposed[:, 1], label='Refinement proposal', color='#c36528', linewidth=1.)
    labels = []
    for n, edge in enumerate(loops):
        i, j = edge['i'], edge['j']; pair = proposed[keys[[i, j]]]
        kind = 'existing' if edge['sensor'] == 'lidar_revisit' else 'new'
        label = f'{i}→{j} ({kind})\n{seconds[keys[i]]:.1f}→{seconds[keys[j]]:.1f} s'
        labels.append(label)
        axes[0].plot(pair[:, 0], pair[:, 1], '-o', color='#347858', linewidth=1., markersize=3)
        axes[0].annotate(label, pair[1, :2], xytext=(10, 15 if n%2 else -35), textcoords='offset points',
                         fontsize=8, bbox={'facecolor': 'white', 'alpha': .85, 'edgecolor': '#c8d0d0'},
                         arrowprops={'arrowstyle': '-', 'color': '#347858', 'lw': .7})
    axes[0].set_aspect('equal'); axes[0].set(xlabel='Map x (m)', ylabel='Map y (m)', title='Measured relative poses; first pose fixed')
    axes[0].legend(fontsize=8); axes[0].grid(alpha=.15)
    before = [np.linalg.norm(e['residuals']['current_delivered_graph'][:2])*1000 for e in summary['loop_residuals']]
    after = [np.linalg.norm(e['residuals']['proposed_graph'][:2])*1000 for e in summary['loop_residuals']]
    x = np.arange(len(loops))
    axes[1].bar(x-.18, before, width=.36, color='#71838a', label='Delivered graph')
    axes[1].bar(x+.18, after, width=.36, color='#c36528', label='Refinement graph')
    axes[1].set_xticks(x, labels, fontsize=8)
    axes[1].set(ylabel='Fitted translation residual (mm)', title='Training constraint fit — not physical accuracy')
    axes[1].legend(fontsize=8); axes[1].grid(axis='y', alpha=.15)
    fig.suptitle(f'{summary["profile"]}: {summary["additional_loops"]} additional loop; '
                 f'maximum trajectory change {100*summary["maximum_change_from_delivered_trajectory_m"]:.2f} cm\n'
                 'No endpoint coincidence is imposed; localization evaluation is separate', fontsize=12)
    fig.tight_layout()
    destination = directory/'graph_constraint_review.png'
    fig.savefig(destination, dpi=170); plt.close(fig)
    paths = [trajectory, graph_path, summary_path, Path(__file__), destination]
    (directory/'graph_plot_provenance.json').write_text(json.dumps({'sha256': {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}}, indent=2)+'\n')
    print(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='*', type=Path,
                        default=[Path('outputs/fin301/loop_refinement/all-heading'), Path('outputs/fin301/loop_refinement/multikeyframe')])
    for directory in parser.parse_args().directories:
        plot(directory)
