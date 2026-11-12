#!/usr/bin/env python3
"""Figures and summary numbers from describe_bag.py output.

Writes into <describe_dir>: timeline.png, route_context.png, start_end.png,
frames.jpg, scan_filter.png and summary.json. Odometry and drive commands
appear only to describe the drive; they are not estimates.

Usage: python3 -I plot_content.py <describe_dir>
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

STOP_MPS = 0.05


def rolling_median(x, n):
    x = np.asarray(x, dtype=float)
    if x.size < n:
        return x
    xp = np.pad(x, (n // 2, n - 1 - n // 2), mode='edge')
    return np.nanmedian(np.lib.stride_tricks.sliding_window_view(xp, n), axis=1)


def intervals(t, mask, min_len):
    """(start, end) times of runs where mask holds for at least min_len seconds."""
    out, i = [], 0
    while i < mask.size:
        if mask[i]:
            j = i
            while j + 1 < mask.size and mask[j + 1]:
                j += 1
            if t[j] - t[i] >= min_len:
                out.append((float(t[i]), float(t[j])))
            i = j + 1
        else:
            i += 1
    return out


def spans(transitions, t_end):
    """[(start, end, value)] from [(t, value)] change points."""
    return [(t, transitions[k + 1][0] if k + 1 < len(transitions) else t_end, v)
            for k, (t, v) in enumerate(transitions)]


def mmss(sec):
    s = int(sec)
    return f'{s // 60}:{s % 60:02d}'


def main():
    d = sys.argv[1]
    S = np.load(os.path.join(d, 'series.npz'))
    inv = json.load(open(os.path.join(d, 'inventory.json')))
    st = json.load(open(os.path.join(d, 'states.json')))
    sf = json.load(open(os.path.join(d, 'scan_filter.json')))
    T = inv['duration_s']
    summary = {'duration_s': T}

    groups = {}
    for t in inv['topics']:
        g = groups.setdefault(t['group'], {'topics': 0, 'with_data': 0, 'megabytes': 0.0})
        g['topics'] += 1
        g['with_data'] += int(t['count'] > 0)
        g['megabytes'] = round(g['megabytes'] + t['megabytes'], 2)
    summary['groups'] = groups

    # Drive, from vehicle odometry and commands (context only).
    od = S['odom'][np.argsort(S['odom'][:, 0], kind='stable')]
    od = od[np.r_[True, np.diff(od[:, 0]) > 0]]
    to, x, y = od[:, 0], od[:, 1], od[:, 2]
    yaw = np.unwrap(od[:, 3])
    speed = rolling_median(np.abs(od[:, 4]), 5)
    if np.nanstd(speed) < 1e-6:  # twist not filled: differentiate the positions
        speed = rolling_median(np.r_[0, np.hypot(np.diff(x), np.diff(y)) / np.diff(to)], 5)
    yaw_rate = rolling_median(np.degrees(np.gradient(yaw, to)), 5)
    path = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]
    moving = speed >= STOP_MPS
    stops = intervals(to, ~moving, 2.0)
    drive = {
        'odom_path_m': round(float(path[-1]), 1),
        'odom_start_end_gap_m': round(float(np.hypot(x[-1] - x[0], y[-1] - y[0])), 2),
        'first_motion_s': round(float(to[np.argmax(moving)]), 1),
        'last_motion_s': round(float(to[moving.size - 1 - np.argmax(moving[::-1])]), 1),
        'stops_over_2s': len(stops),
        'stopped_s_total': round(sum(b - a for a, b in stops), 1),
        'stops': [[round(a, 1), round(b, 1)] for a, b in stops],
        'median_moving_speed_mps': round(float(np.median(speed[moving])), 3),
        'p95_speed_mps': round(float(np.percentile(speed, 95)), 3),
        'max_speed_mps': round(float(speed.max()), 3),
        'p95_abs_yaw_rate_dps': round(float(np.percentile(np.abs(yaw_rate), 95)), 1),
        'share_of_time_turning_over_30dps': round(float((np.abs(yaw_rate) > 30).mean()), 3),
        'net_heading_change_deg': round(float(np.degrees(yaw[-1] - yaw[0])), 1),
    }
    ts = np.arange(0.0, to[-1], 1.0)
    P = np.column_stack([np.interp(ts, to, x), np.interp(ts, to, y)])
    sp = np.interp(ts, to, path)
    D = np.hypot(P[:, None, 0] - P[None, :, 0], P[:, None, 1] - P[None, :, 1])
    hit = (D < 1.0) & (sp[:, None] - sp[None, :] > 10.0)  # row: later time, column: earlier
    episodes, rows = [], np.where(hit.any(axis=1))[0]
    if rows.size:
        for g in np.split(rows, np.where(np.diff(rows) > 3)[0] + 1):
            cols = np.where(hit[g].any(axis=0))[0]
            episodes.append({'later_s': [float(ts[g[0]]), float(ts[g[-1]])],
                             'earlier_s': [float(ts[cols.min()]), float(ts[cols.max()])],
                             'closest_m': round(float(D[np.ix_(g, cols)].min()), 2)})
    drive['revisits_by_odometry'] = episodes
    summary['drive_context_only'] = drive

    cmd = {}
    for name in ('ackermann_cmd', 'teleop'):
        if name in S.files and S[name].size:
            a = S[name]
            tt = a[:, 0]
            gaps = np.where(np.diff(tt) > 0.5)[0]
            starts, ends = np.r_[tt[0], tt[gaps + 1]], np.r_[tt[gaps], tt[-1]]
            cmd[name] = {
                'messages': int(a.shape[0]),
                'active_spans_s': [[round(float(s0), 1), round(float(e0), 1)]
                                   for s0, e0 in zip(starts, ends) if e0 - s0 >= 0.5][:40],
                'speed_mps_p5_p50_p95': np.percentile(a[:, 2], [5, 50, 95]).round(3).tolist(),
                'share_nonzero_speed': round(float((np.abs(a[:, 2]) > 1e-3).mean()), 3),
                'steering_deg_min_max': np.degrees([a[:, 1].min(), a[:, 1].max()]).round(1).tolist(),
            }
    summary['commands'] = cmd
    if 'rc' in S.files and S['rc'].size:
        rc = S['rc']
        summary['rc_channels'] = [
            {'channel': i, 'min': float(np.nanmin(c)), 'median': float(np.nanmedian(c)),
             'max': float(np.nanmax(c)), 'distinct': int(np.unique(c[np.isfinite(c)]).size)}
            for i, c in enumerate(rc[:, 1:].T)]
    if 'battery' in S.files and S['battery'].size:
        b = S['battery']
        summary['battery'] = {'voltage_start_end': [round(float(b[0, 1]), 2), round(float(b[-1, 1]), 2)],
                              'percentage_start_end': [round(float(b[0, 2]), 3), round(float(b[-1, 2]), 3)]}
    modes = {}
    for topic, info in st['strings'].items():
        if not info['transitions']:
            continue
        dur = {}
        for a, b_, v in spans(info['transitions'], T):
            dur[v[:80]] = dur.get(v[:80], 0.0) + (b_ - a)
        modes[topic] = {'n_transitions': info['n_transitions'],
                        'time_by_value_s': dict(sorted(((k, round(v, 1)) for k, v in dur.items()),
                                                       key=lambda kv: -kv[1])[:10]),
                        'first_transitions': info['transitions'][:6]}
    summary['modes'] = modes

    # Camera quality.
    cam = S['cam']
    tc, mean_g, sat, dark, lap, corners = (cam[:, i] for i in range(6))
    med = float(np.nanmedian(lap))
    lap_s = rolling_median(lap, 10)
    yr_c, sp_c = np.interp(tc, to, np.abs(yaw_rate)), np.interp(tc, to, speed)

    def med_where(mask):
        return round(float(np.nanmedian(lap[mask])), 1) if mask.any() else None

    summary['camera'] = {
        'frames': int(cam.shape[0]),
        'mean_gray_median': round(float(np.nanmedian(mean_g)), 1),
        'laplacian_var_median': round(med, 1),
        'laplacian_median_stopped': med_where(sp_c < STOP_MPS),
        'laplacian_median_moving_straight': med_where((sp_c >= STOP_MPS) & (yr_c < 10)),
        'laplacian_median_turning_over_30dps': med_where(yr_c > 30),
        'share_frames_blurry_under_quarter_median': round(float((lap < 0.25 * med).mean()), 3),
        'blurry_spans_over_1s': [[round(a, 1), round(b, 1)]
                                 for a, b in intervals(tc, lap_s < 0.25 * med, 1.0)],
        'share_frames_overexposed_5pct': round(float((sat > 0.05).mean()), 3),
        'share_frames_mostly_dark': round(float((dark > 0.5).mean()), 3),
        'fast_corners_p10_p50_p90': np.nanpercentile(corners, [10, 50, 90]).round(0).tolist(),
        'share_frames_under_100_corners': round(float((corners < 100).mean()), 3),
    }

    # LiDAR quality and the /scan filter.
    sr = S['scan_raw']
    summary['lidar'] = {
        'scans': int(sr.shape[0]),
        'valid_frac_mean': round(float(np.nanmean(sr[:, 1])), 3),
        'valid_frac_p5': round(float(np.nanpercentile(sr[:, 1], 5)), 3),
        'nonfinite_frac_mean': round(float(np.nanmean(sr[:, 4])), 3),
        'zero_frac_mean': round(float(np.nanmean(sr[:, 5])), 4),
        'median_range_m_p10_p50_p90': np.nanpercentile(sr[:, 2], [10, 50, 90]).round(2).tolist(),
        'frac_valid_under_1m_p50_p95': np.nanpercentile(sr[:, 3], [50, 95]).round(3).tolist(),
        'filtered_valid_frac_mean': round(float(np.nanmean(S['scan'][:, 1])), 3),
    }
    summary['scan_filter'] = {k: v for k, v in sf.items()
                              if k not in ('removal_rate_by_beam', 'beam_angles_deg')}
    summary['rosout_top'] = st['rosout'][:15]
    summary['diagnostics'] = [{k: r[k] for k in ('name', 'level', 'count', 'first_s', 'message')}
                              for r in st['diagnostics'][:25]]
    with open(os.path.join(d, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)

    def tm(t):
        return np.asarray(t, dtype=float) / 60.0

    # Timeline.
    fig, axes = plt.subplots(6, 1, figsize=(15, 16), sharex=True,
                             gridspec_kw={'height_ratios': [0.6, 1, 1, 1, 1, 1]})
    ax = axes[0]
    cmap = plt.get_cmap('tab10')
    strip = [('main_state', '/control_center/main_state'),
             ('selected_cmd', '/control_center/selected_cmd'),
             ('behavior', '/behavior/state')]
    for row, (label, topic) in enumerate(strip):
        tr = st['strings'].get(topic, {}).get('transitions', [])
        sp_ = spans(tr, T)
        vals = sorted({v for _, _, v in sp_})
        for a, b_, v in sp_:
            ax.barh(row, tm(b_ - a), left=tm(a), height=0.8, color=cmap(vals.index(v) % 10))
            if b_ - a > 25:
                ax.text(tm(a) + 0.05, row, v[:28], fontsize=7, va='center')
    ax.set_yticks(range(len(strip)), [s[0] for s in strip])
    ax.invert_yaxis()
    ax.set_title('Vehicle software state (from /control_center and /behavior strings)')
    for a_ in axes[1:]:
        for s0, e0 in stops:
            a_.axvspan(tm(s0), tm(e0), color='0.88', zorder=0, lw=0)
    ax = axes[1]
    ax.plot(tm(to), speed, lw=0.8, label='odometry speed')
    if 'ackermann_cmd' in S.files:
        ax.plot(tm(S['ackermann_cmd'][:, 0]), S['ackermann_cmd'][:, 2], lw=0.6, alpha=0.7,
                label='commanded speed (/ackermann_cmd)')
    ax.set_ylabel('m/s')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title('Speed (context only; grey bands: stopped for 2 s or more)')
    ax = axes[2]
    ax.plot(tm(to), yaw_rate, lw=0.6, label='yaw rate from odometry')
    ax.set_ylabel('deg/s')
    if 'ackermann_cmd' in S.files:
        ax2 = ax.twinx()
        ax2.plot(tm(S['ackermann_cmd'][:, 0]), np.degrees(S['ackermann_cmd'][:, 1]), color='C3',
                 lw=0.5, alpha=0.6)
        ax2.set_ylabel('steering command (deg)', color='C3')
    ax.set_title('Turning (context only)')
    ax = axes[3]
    ax.semilogy(tm(tc), lap, ',', color='0.6')
    ax.semilogy(tm(tc), lap_s, lw=0.8)
    ax.axhline(0.25 * med, ls='--', lw=0.8, color='C3')
    ax.set_ylabel('Laplacian variance')
    ax.set_title('Camera sharpness (dashed: 25% of the median, used as the blur threshold)')
    ax = axes[4]
    ax.plot(tm(tc), rolling_median(corners, 10), lw=0.8)
    ax.set_ylabel('FAST corners per frame')
    ax2 = ax.twinx()
    ax2.plot(tm(tc), 100 * rolling_median(sat, 10), color='C1', lw=0.8)
    ax2.set_ylabel('% pixels at 250 or above', color='C1')
    ax.set_title('Camera texture (blue) and over-exposure (orange)')
    ax = axes[5]
    ax.plot(tm(sr[:, 0]), 100 * rolling_median(sr[:, 1], 30), lw=0.8, label='/scan/raw')
    ax.plot(tm(S['scan'][:, 0]), 100 * rolling_median(S['scan'][:, 1], 30), lw=0.8, label='/scan')
    ax.set_ylabel('% beams with a return')
    ax.legend(loc='lower left', fontsize=8)
    ax2 = ax.twinx()
    ax2.plot(tm(sr[:, 0]), 100 * rolling_median(sr[:, 3], 30), color='C3', lw=0.8)
    ax2.set_ylabel('% returns closer than 1 m', color='C3')
    ax.set_title('LiDAR returns')
    axes[-1].set_xlabel('minutes since the start of the recording')
    for a_ in axes:
        a_.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'timeline.png'), dpi=100)
    plt.close(fig)

    # Route with LiDAR returns placed by odometry.
    sc = np.load(os.path.join(d, 'scans_every10.npz'))
    R_all, t_all = sc['ranges'].astype(np.float32), sc['t']
    ang = float(sc['angle_min']) + float(sc['angle_increment']) * np.arange(R_all.shape[1])
    lx, ly = st['tf_static'].get('base_link -> laser', [0.0, 0.0])[:2]
    inside = (t_all >= to[0]) & (t_all <= to[-1])
    R, tsc = R_all[inside], t_all[inside]
    px, py, pyaw = np.interp(tsc, to, x), np.interp(tsc, to, y), np.interp(tsc, to, yaw)
    bx, by = R * np.cos(ang) + lx, R * np.sin(ang) + ly
    c, s_ = np.cos(pyaw)[:, None], np.sin(pyaw)[:, None]
    wx, wy = px[:, None] + c * bx - s_ * by, py[:, None] + s_ * bx + c * by
    m = np.isfinite(wx) & (R < 15)
    fig, ax = plt.subplots(figsize=(14, 10))
    ax.plot(wx[m], wy[m], ',', color='0.35', alpha=0.15, rasterized=True)
    sca = ax.scatter(x, y, c=tm(to), s=3, cmap='viridis', zorder=3)
    for mnt in range(int(T // 60) + 1):
        i = int(np.searchsorted(to, 60.0 * mnt))
        if i < to.size:
            ax.plot(x[i], y[i], 'k.', ms=5, zorder=4)
            ax.annotate(str(mnt), (x[i], y[i]), xytext=(3, 3), textcoords='offset points',
                        fontsize=9, zorder=4)
    ax.plot(x[0], y[0], 'g^', ms=13, zorder=5, label='start')
    ax.plot(x[-1], y[-1], 'rs', ms=10, zorder=5, label='end')
    fig.colorbar(sca, ax=ax, label='minutes since start (black dots: whole minutes)')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right')
    ax.set_xlabel('odom x (m)')
    ax.set_ylabel('odom y (m)')
    ax.set_title('Drive path and LiDAR returns placed with the vehicle odometry\n'
                 '(context only: shows the route and the odometry drift; not our map)')
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'route_context.png'), dpi=110)
    plt.close(fig)

    # Start versus end.
    frames = sorted(glob.glob(os.path.join(d, 'frames', 'frame_*s.jpg')))
    fig, axs = plt.subplots(1, 3, figsize=(18, 5.2))
    axs[0].imshow(mpimg.imread(frames[0]))
    axs[0].set_title(f'first camera frame ({mmss(float(os.path.basename(frames[0])[6:-5]))})')
    axs[1].imshow(mpimg.imread(os.path.join(d, 'frames', 'frame_last.jpg')))
    axs[1].set_title(f'last camera frame ({mmss(tc[-1])})')
    for a_ in axs[:2]:
        a_.axis('off')
    ax = axs[2]
    for idx, col in ((0, 'C2'), (-1, 'C3')):
        r = R_all[idx]
        ok = np.isfinite(r)
        ax.plot(r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]), '.', ms=1.5, color=col,
                label=f'scan at {mmss(t_all[idx])}')
    ax.plot(0, 0, 'k^', ms=8)
    ax.set_xlim(-12, 12)
    ax.set_ylim(-9, 9)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, markerscale=8)
    ax.set_title('first and last scan, laser frame (x forward, y left)')
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'start_end.png'), dpi=100)
    plt.close(fig)

    # One frame every 30 s.
    cols = 5
    rows_n = int(np.ceil(len(frames) / cols))
    fig, axs = plt.subplots(rows_n, cols, figsize=(cols * 3.6, rows_n * 2.95))
    for a_ in np.ravel(axs):
        a_.axis('off')
    for a_, f in zip(np.ravel(axs), frames):
        a_.imshow(mpimg.imread(f))
        a_.set_title(mmss(float(os.path.basename(f)[6:-5])), fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(d, 'frames.jpg'), dpi=85)
    plt.close(fig)

    # What /scan removes.
    if sf.get('pairs'):
        fig, axs = plt.subplots(1, 2, figsize=(15, 4.2))
        axs[0].plot(sf['beam_angles_deg'], 100 * np.asarray(sf['removal_rate_by_beam']), lw=0.8)
        axs[0].set_xlabel('beam angle (deg; 0 = laser +x, positive to the left)')
        axs[0].set_ylabel('% of raw returns removed')
        axs[0].grid(alpha=0.3)
        axs[0].set_title('Returns present in /scan/raw but missing from /scan, by angle')
        bins = sf['range_bins_m']
        labels = [f'{bins[i]:g}-{bins[i + 1]:g}' for i in range(len(bins) - 1)]
        rem, kept = np.asarray(sf['removed_by_range']), np.asarray(sf['kept_by_range'])
        axs[1].bar(range(len(labels)), 100 * rem / np.maximum(rem + kept, 1))
        axs[1].set_xticks(range(len(labels)), labels, rotation=45, ha='right', fontsize=8)
        axs[1].set_xlabel('raw range (m)')
        axs[1].set_ylabel('% removed')
        axs[1].set_title('...and by range')
        fig.tight_layout()
        fig.savefig(os.path.join(d, 'scan_filter.png'), dpi=100)
        plt.close(fig)

    print(json.dumps({k: summary[k] for k in ('drive_context_only', 'commands', 'camera', 'lidar')},
                     indent=1))


if __name__ == '__main__':
    main()
