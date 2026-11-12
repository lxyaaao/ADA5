"""Figures for checking each stage and for the report."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from .geometry import transform


def _grid_rgb(grid):
    st = grid.states()
    img = np.full(st.shape + (3,), 0.78)
    img[st == 0] = 1.0
    img[st == 1] = 0.0
    return img


def _scale_bar(ax, length=5.0):
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    bx, by = x0 + 0.04 * (x1 - x0), y0 + 0.05 * (y1 - y0)
    ax.plot([bx, bx + length], [by, by], 'k-', lw=3)
    ax.text(bx + length / 2, by + 0.02 * (y1 - y0), f'{length:g} m', ha='center', fontsize=9)


def plot_frontend(recs, keyframes, path, title='LiDAR odometry (front end only, no loop closure)'):
    poses = np.array([r['pose'] for r in recs])
    t = np.array([r['t'] for r in recs])
    lost = np.array([r['status'] == 'lost' for r in recs])
    degen = np.array([r['status'] == 'degenerate' for r in recs])
    pts = np.vstack([transform(k['pose'], k['pts']) for k in keyframes])
    fig, ax = plt.subplots(figsize=(14, 10))
    ax.plot(pts[:, 0], pts[:, 1], ',', color='0.3', alpha=0.25, rasterized=True)
    sc = ax.scatter(poses[:, 0], poses[:, 1], c=t / 60, s=2, cmap='viridis', zorder=3)
    ax.plot(poses[degen, 0], poses[degen, 1], 'o', ms=3, mfc='none', mec='orange', zorder=4,
            label=f'degenerate geometry ({degen.sum()})')
    ax.plot(poses[lost, 0], poses[lost, 1], 'x', ms=4, color='red', zorder=5,
            label=f'match rejected, prediction used ({lost.sum()})')
    ax.plot(*poses[0, :2], 'g^', ms=12, zorder=6, label='start')
    ax.plot(*poses[-1, :2], 'rs', ms=9, zorder=6, label='end')
    fig.colorbar(sc, ax=ax, label='minutes since start')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_heading_check(d_lidar, d_cam, path, interval_s=0.2):
    dl, dc = np.degrees(d_lidar), np.degrees(d_cam)
    fig, axs = plt.subplots(1, 2, figsize=(13, 5))
    axs[0].plot(dl, dc, '.', ms=2, alpha=0.4)
    lim = np.percentile(np.abs(np.r_[dl, dc]), 99.5)
    axs[0].plot([-lim, lim], [-lim, lim], 'k--', lw=0.8)
    axs[0].set_xlim(-lim, lim)
    axs[0].set_ylim(-lim, lim)
    axs[0].set_aspect('equal')
    axs[0].set_xlabel('LiDAR odometry heading change (deg)')
    axs[0].set_ylabel('camera essential-matrix heading change (deg)')
    axs[0].set_title(f'Heading change between the camera frames used ({interval_s:g} s apart)')
    axs[0].grid(alpha=0.3)
    axs[1].hist(np.clip(dc - dl, -3, 3), bins=120)
    axs[1].set_xlabel('camera minus LiDAR (deg, clipped at ±3)')
    axs[1].set_ylabel('frame pairs')
    axs[1].set_title('Disagreement')
    axs[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def plot_map(grid, traj, t, kf, loops, path):
    fig, ax = plt.subplots(figsize=(15, 11))
    ax.imshow(_grid_rgb(grid), origin='lower', extent=grid.extent(), interpolation='nearest')
    sc = ax.scatter(traj[:, 0], traj[:, 1], c=np.asarray(t) / 60, s=0.6, cmap='viridis', zorder=3)
    for l in loops:
        ax.plot([kf[l['j'], 0], kf[l['i'], 0]], [kf[l['j'], 1], kf[l['i'], 1]], '-', color='orangered',
                lw=1.2, zorder=4)
    ax.plot([], [], '-', color='orangered', label=f'camera-proposed, LiDAR-verified loop closures ({len(loops)})')
    ax.plot(*traj[0, :2], 'g^', ms=12, zorder=5, label='start (map origin)')
    ax.plot(*traj[-1, :2], 'rs', ms=9, zorder=5, label='end')
    fig.colorbar(sc, ax=ax, shrink=0.7, label='trajectory: minutes since start')
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='upper right', fontsize=9)
    ax.set_title('Occupancy map of the explored area (black occupied, white free, grey unknown; 0.05 m cells)\n'
                 'Map frame: origin at the vehicle start pose, x forward, y left. The map is the LiDAR scan plane '
                 '(0.034 m above base_link, 0.087 m above the camera).', fontsize=10)
    _scale_bar(ax)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def plot_before_after(grid_before, grid_after, kf_before, kf_after, loops, path):
    fig, axs = plt.subplots(1, 2, figsize=(20, 9))
    for ax, g, kf, name in ((axs[0], grid_before, kf_before, 'LiDAR odometry only'),
                            (axs[1], grid_after, kf_after, 'after loop closure (pose graph)')):
        ax.imshow(_grid_rgb(g), origin='lower', extent=g.extent(), interpolation='nearest')
        ax.plot(kf[:, 0], kf[:, 1], '-', color='tab:blue', lw=0.6)
        for l in loops:
            ax.plot([kf[l['j'], 0], kf[l['i'], 0]], [kf[l['j'], 1], kf[l['i'], 1]], '-', color='orangered', lw=1)
        ax.set_aspect('equal')
        ax.set_title(f'{name}: {(g.states() == 1).sum()} occupied cells')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def plot_tracking(recs, image_t, yaw_cam, cam_inliers, loops, path):
    t = np.array([r['t'] for r in recs]) / 60
    inl = np.array([r['inlier'] for r in recs])
    st = np.array([r['status'] for r in recs])
    fig, axs = plt.subplots(3, 1, figsize=(15, 9), sharex=True)
    axs[0].plot(t, inl, lw=0.5)
    for name, col in (('lost', 'red'), ('degenerate', 'orange')):
        m = st == name
        axs[0].plot(t[m], inl[m], 'o', ms=3, color=col, label=f'{name} ({m.sum()})')
    axs[0].set_ylabel('ICP inlier share')
    axs[0].set_title('LiDAR scan matching quality (share of points within 0.1 m of the local map)')
    axs[0].legend(loc='lower right', fontsize=8)
    ok = np.isfinite(yaw_cam)
    axs[1].plot(image_t[ok] / 60, cam_inliers[ok], '.', ms=1.5, label='heading estimated')
    axs[1].plot(image_t[~ok] / 60, cam_inliers[~ok], 'x', ms=3, color='red',
                label=f'no verified estimate ({(~ok[1:]).sum()} of {len(ok) - 1} pairs)')
    axs[1].set_ylabel('essential-matrix inliers')
    axs[1].set_title('Camera heading between consecutive frames')
    axs[1].legend(loc='upper right', fontsize=8)
    for l in loops:
        axs[2].plot([l['t_j'] / 60, l['t_i'] / 60], [0, 1], '-', color='orangered', lw=0.8)
    axs[2].set_yticks([0, 1], ['earlier visit', 'revisit'])
    axs[2].set_title(f'Accepted loop closures ({len(loops)}): each line joins a revisit to the earlier keyframe')
    axs[2].set_xlabel('minutes since start')
    for ax in axs:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)
