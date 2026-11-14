"""Time-separated 2D visibility evidence; never classify occlusion as free space.

Fixed parameters were declared before examining candidate outcomes. Persistent
support is evidence during this recording, not a semantic or permanence label.
"""
from __future__ import annotations
import numpy as np
from scipy.ndimage import minimum_filter1d
from scipy.spatial import cKDTree


PARAMETERS = {
    'epoch_seconds': 2., 'hit_distance_m': .12, 'free_guard_m': .20,
    'free_fan_lateral_uncertainty_m': .10, 'minimum_range_m': .12, 'maximum_range_m': 12.,
    'conflict_minimum_free_epochs': 3, 'conflict_minimum_free_span_s': 4.,
    'conflict_minimum_free_fraction': .70, 'static_minimum_hit_epochs': 3,
    'static_minimum_hit_span_s': 10., 'static_maximum_free_fraction': .30,
}


def point_visibility(local_points, ranges, angle_min, angle_increment):
    """Return hit/free/occluded boolean arrays for finite local XY candidates.

    Free requires every beam in a conservative angular fan to have a finite
    valid return beyond candidate range +20cm. A short/invalid neighboring ray
    prevents free evidence. Hits use actual XY endpoint distance <=12cm.
    The current implementation explicitly requires a full-rotation scan.
    """
    points, ranges = np.asarray(local_points, float), np.asarray(ranges, float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
        raise ValueError('Expected finite Nx2 local points')
    if ranges.ndim != 1 or angle_increment <= 0 or abs(len(ranges)*angle_increment-2*np.pi) > 2*angle_increment:
        raise ValueError('Temporal range projection requires a full360-degree scan')
    radius = np.linalg.norm(points, axis=1)
    visible = (radius >= PARAMETERS['minimum_range_m']) & (radius < PARAMETERS['maximum_range_m'])
    hit = np.zeros(len(points), bool); free = hit.copy(); occluded = hit.copy()
    valid = np.isfinite(ranges) & (ranges >= PARAMETERS['minimum_range_m']) & (ranges < PARAMETERS['maximum_range_m'])
    candidates = np.flatnonzero(visible)
    if not len(candidates) or not valid.any():
        return hit, free, occluded
    angles = angle_min+np.arange(len(ranges))*angle_increment
    endpoints = ranges[valid, None]*np.column_stack([np.cos(angles[valid]), np.sin(angles[valid])])
    distances = cKDTree(endpoints).query(points[candidates], distance_upper_bound=PARAMETERS['hit_distance_m']+1e-12)[0]
    hit[candidates] = distances <= PARAMETERS['hit_distance_m']+1e-12
    beam = np.rint((np.arctan2(points[candidates, 1], points[candidates, 0])-angle_min)/angle_increment).astype(np.int64) % len(ranges)
    halfwidth = np.maximum(2, np.ceil(np.arctan2(PARAMETERS['free_fan_lateral_uncertainty_m'], radius[candidates])/angle_increment).astype(int))
    windows = np.array([2**k for k in range(1, 11) if 2**k < len(ranges)//2]+[len(ranges)//2-1])
    windows = np.unique(windows)
    which = np.minimum(np.searchsorted(windows, halfwidth), len(windows)-1)
    minimum_valid = np.full(len(candidates), np.inf)
    minimum_all = np.full(len(candidates), -np.inf)
    # Rounding the fan UP is conservative around thin foreground obstacles.
    for index in np.unique(which):
        selected = which == index; size = int(2*windows[index]+1)
        all_beams = minimum_filter1d(np.where(valid, ranges, -np.inf), size=size, mode='wrap')
        valid_beams = minimum_filter1d(np.where(valid, ranges, np.inf), size=size, mode='wrap')
        minimum_all[selected] = all_beams[beam[selected]]
        minimum_valid[selected] = valid_beams[beam[selected]]
    free[candidates] = minimum_all > radius[candidates]+PARAMETERS['free_guard_m']
    occluded[candidates] = minimum_valid < radius[candidates]-PARAMETERS['free_guard_m']
    free &= ~hit; occluded &= ~hit & ~free
    return hit, free, occluded


def classify_temporal(evidence):
    hits, free = evidence['hit_epochs'], evidence['free_epochs']
    fraction = np.divide(free, hits+free, out=np.zeros(len(hits), float), where=(hits+free)>0)
    conflict = ((free >= PARAMETERS['conflict_minimum_free_epochs'])
                & (evidence['free_span_s'] >= PARAMETERS['conflict_minimum_free_span_s'])
                & (fraction >= PARAMETERS['conflict_minimum_free_fraction']))
    supported = ((hits >= PARAMETERS['static_minimum_hit_epochs'])
                 & (evidence['hit_span_s'] >= PARAMETERS['static_minimum_hit_span_s'])
                 & (fraction <= PARAMETERS['static_maximum_free_fraction']))
    return {'conflicted': conflict, 'static_supported': supported,
            'insufficient': ~(conflict | supported), 'free_fraction': fraction}


def compute_temporal_evidence(pointsXYZ, raw_ranges, raw_times_ns, scan_ids, poses, angle_min, angle_increment):
    """Evaluate selected chronological raw scans and supplied corrected LiDAR poses.

    Each point gets at most one hit or free vote per2s epoch. Hit overrides free
    within an epoch. Raw conflicting epoch counts are retained. Support spans
    use actual first/last supporting scan timestamps, never bin-center times.
    """
    points, poses = np.asarray(pointsXYZ), np.asarray(poses, float)
    raw_times = np.asarray(raw_times_ns, np.int64); scan_ids = np.asarray(scan_ids, np.int64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)) or np.any(points[:, 2] != 0):
        raise ValueError('Expected finite planar LiDAR Nx3, z=0')
    if poses.shape != (len(scan_ids), 3) or not np.all(np.isfinite(poses)) or not len(scan_ids):
        raise ValueError('One finite corrected SE2 pose is required per selected raw scan')
    if len(raw_ranges) != len(raw_times) or np.any(scan_ids < 0) or np.any(scan_ids >= len(raw_times)):
        raise ValueError('Raw scan arrays and selected IDs are inconsistent')
    times = raw_times[scan_ids]
    if np.any(np.diff(times) <= 0): raise ValueError('Selected timestamps must be strictly increasing')
    epoch_ids = ((times-times[0])//2_000_000_000).astype(int)
    count, epochs = len(points), int(epoch_ids[-1])+1
    states = np.zeros((epochs, count), np.uint8)
    names = ('hit_epochs', 'free_epochs', 'raw_free_epochs', 'occluded_epochs', 'within_epoch_hit_free_conflicts')
    evidence = {name: np.zeros(count, np.uint32) for name in names}
    first = {kind: np.full(count, np.inf) for kind in ('hit', 'free')}
    last = {kind: np.full(count, -np.inf) for kind in ('hit', 'free')}
    first_id = {kind: np.full(count, -1, np.int32) for kind in ('hit', 'free')}
    last_id = {kind: np.full(count, -1, np.int32) for kind in ('hit', 'free')}
    local_first = {kind: np.full(count, np.inf) for kind in ('hit', 'free')}
    local_last = {kind: np.full(count, -np.inf) for kind in ('hit', 'free')}
    local_first_id = {kind: np.full(count, -1, np.int32) for kind in ('hit', 'free')}
    local_last_id = {kind: np.full(count, -1, np.int32) for kind in ('hit', 'free')}

    def flush(epoch):
        state = states[epoch]; h = (state & 1) != 0; raw_f = (state & 2) != 0
        f = raw_f & ~h; o = ((state & 4) != 0) & ~h & ~f
        for name, mask in [('hit_epochs', h), ('free_epochs', f), ('raw_free_epochs', raw_f),
                           ('occluded_epochs', o), ('within_epoch_hit_free_conflicts', h & raw_f)]:
            evidence[name] += mask
        for kind, mask in [('hit', h), ('free', f)]:
            earlier = mask & (local_first[kind] < first[kind]); later = mask & (local_last[kind] > last[kind])
            first[kind][earlier] = local_first[kind][earlier]; first_id[kind][earlier] = local_first_id[kind][earlier]
            last[kind][later] = local_last[kind][later]; last_id[kind][later] = local_last_id[kind][later]
            local_first[kind].fill(np.inf); local_last[kind].fill(-np.inf)
            local_first_id[kind].fill(-1); local_last_id[kind].fill(-1)

    previous_epoch = int(epoch_ids[0]); per_scan = []
    for number, (raw_id, pose, epoch) in enumerate(zip(scan_ids, poses, epoch_ids)):
        if epoch != previous_epoch:
            flush(previous_epoch); previous_epoch = int(epoch)
        dx, dy = points[:, 0]-pose[0], points[:, 1]-pose[1]
        c, s = np.cos(pose[2]), np.sin(pose[2])
        local = np.column_stack([c*dx+s*dy, -s*dx+c*dy])
        hit, free, occluded = point_visibility(local, raw_ranges[raw_id], angle_min, angle_increment)
        states[epoch, hit] |= 1; states[epoch, free] |= 2; states[epoch, occluded] |= 4
        seconds = (int(raw_times[raw_id])-int(times[0]))*1e-9
        for kind, mask in [('hit', hit), ('free', free)]:
            first_here = mask & ~np.isfinite(local_first[kind])
            local_first[kind][first_here] = seconds; local_first_id[kind][first_here] = raw_id
            local_last[kind][mask] = seconds; local_last_id[kind][mask] = raw_id
        per_scan.append([int(raw_id), int(raw_times[raw_id]), int(epoch), int(hit.sum()), int(free.sum()), int(occluded.sum())])
        if number % 250 == 0:
            print(f'Temporal evidence {number+1}/{len(scan_ids)} scans; epoch {epoch+1}/{epochs}', flush=True)
    flush(previous_epoch)
    for kind in ('hit', 'free'):
        observed = np.isfinite(first[kind])
        span = np.zeros(count); span[observed] = last[kind][observed]-first[kind][observed]
        evidence[kind+'_span_s'] = span
        evidence['first_'+kind+'_raw_scan'] = first_id[kind]
        evidence['last_'+kind+'_raw_scan'] = last_id[kind]
        evidence['first_'+kind+'_seconds'] = np.where(observed, first[kind], np.nan)
        evidence['last_'+kind+'_seconds'] = np.where(observed, last[kind], np.nan)
    evidence.update(classify_temporal(evidence))
    evidence.update(epoch_states=states, selected_raw_scan_ids=scan_ids, selected_times_ns=times,
                    selected_epoch_ids=epoch_ids, epoch_origin_timestamp_ns=np.array(times[0]),
                    per_scan_counts=np.asarray(per_scan, np.int64))
    stats = {'parameters': dict(PARAMETERS), 'input_points': count, 'evidence_scans': len(scan_ids),
             'epoch_count': epochs, 'evidence_duration_s': (int(times[-1])-int(times[0]))*1e-9,
             'static_supported_points': int(evidence['static_supported'].sum()),
             'conflicted_points': int(evidence['conflicted'].sum()),
             'insufficient_points': int(evidence['insufficient'].sum()),
             'conflicted_with_two_or_more_hit_epochs': int((evidence['conflicted'] & (evidence['hit_epochs'] >= 2)).sum()),
             'conflicted_with_three_or_more_hit_epochs': int((evidence['conflicted'] & (evidence['hit_epochs'] >= 3)).sum()),
             'points_without_hit_epochs': int((evidence['hit_epochs'] == 0).sum()),
             'epoch_state_bits': {'1': 'any hit', '2': 'any free-ray conflict', '4': 'any closer-return occlusion'},
             'epoch_precedence': 'hit overrides free; hit or free overrides occluded; raw flags and conflicts retained',
             'span_definition': 'Actual first/last supporting scan timestamps in contributing epochs',
             'visibility_rule': 'Every valid neighboring beam must pass beyond point range+0.20m; shorter/invalid rays block free',
             'limitations': ['Evidence uses the estimated trajectory; registration error can resemble temporal change.',
                            'Repeated hit epochs do not imply independent passes or permanent/semantic static objects.',
                            'Stationary people can be persistent-supported; removed points are not all people.',
                            'Insufficient evidence is preserved by the main conservative preset and is not free space.',
                            'Strict static-only output excludes weakly observed one-pass walls as well as conflicts.']}
    return evidence, stats
