"""Bounded all-heading revisit experiments using unchanged registration gates."""
from dataclasses import asdict, replace
import numpy as np

from .geometry import pose_between, pose_compose, pose_inverse, register_scan, transform_points, voxel_downsample, wrap_angle
from .lidar_revisits import RevisitParameters, verify_revisit, geometric_coverage, _fit_good, _fit_summary


def earlier_neighbor_constraint(b_from_neighbor, a_from_neighbor):
    return pose_inverse(pose_compose(b_from_neighbor, pose_inverse(a_from_neighbor)))


def transport_closure(old_measurement, target_from_old_target, old_source_from_source):
    return pose_compose(pose_compose(target_from_old_target, old_measurement), old_source_from_source)


def submap_members(key_index, keys, count, profile):
    anchor = int(keys[key_index])
    if profile == 'all-heading':
        return list(range(max(0, anchor-2), min(count, anchor+3)))
    if profile != 'multikeyframe':
        raise ValueError('Unknown bounded refinement profile')
    members = set()
    for index in range(max(0, key_index-1), min(len(keys), key_index+2)):
        members.update(range(max(0, int(keys[index])-1), min(count, int(keys[index])+2)))
    return sorted(members)


def refinement_parameters():
    return replace(RevisitParameters(), max_candidate_yaw_rad=np.pi+1e-6,
                   min_geometric_coverage=.50, min_mean_geometric_coverage=.65,
                   max_odometry_correction_m=2., min_converged_initializations=3)


def propose_refinement(scans, poses, keys, times, existing_edges, profile='all-heading'):
    """Propose additional measured loops; retain existing edges exactly once.

    Only proposal heading coverage / local submap scope change from the earlier
    partial-overlap experiment. Every pair retains its existing fit, reciprocal,
    geometric coverage, conditioning and supported-multistart gates. Both passes
    gain temporal checks on scans outside their construction submaps. Anchors
    must be at least eight seconds apart on BOTH passes from selected edges.
    """
    poses, keys, times = np.asarray(poses), np.asarray(keys, int), np.asarray(times, np.int64)
    parameters = refinement_parameters()
    key_poses, key_times = poses[keys], times[keys]
    used, cache, members_cache = set(), {}, {}
    events, selected, skipped = [], [dict(edge) for edge in existing_edges], []

    def submap(k):
        if k not in cache:
            members = submap_members(k, keys, len(poses), profile)
            members_cache[k] = members; used.update(members)
            cache[k] = voxel_downsample(np.vstack([
                transform_points(scans[n], pose_between(poses[keys[k]], poses[n])) for n in members]), .07)
        return cache[k]

    def temporal_side(event, side):
        i, j = event['i'], event['j']; a, b = int(keys[i]), int(keys[j])
        anchor = a if side == 'earlier' else b
        k = i if side == 'earlier' else j
        own_members = set(members_cache[k])
        if profile == 'all-heading':
            neighbors = [anchor-6, anchor+6]
        else:
            neighbors = [min(own_members)-4, max(own_members)+4]
        opposite = submap(j if side == 'earlier' else i)
        checks = []
        for n in neighbors:
            if n < 0 or n >= len(poses) or n in own_members:
                checks.append({'accepted': False, 'reason': 'validation_scan_not_available_or_not_disjoint', 'dense_index': n})
                continue
            used.add(n)
            anchor_from_neighbor = pose_between(poses[anchor], poses[n])
            initial = pose_compose(pose_inverse(event['pose']) if side == 'earlier' else event['pose'], anchor_from_neighbor)
            fit = register_scan(scans[n], opposite, initial=initial, max_distance=.35, iterations=36)
            transported = (earlier_neighbor_constraint(fit['pose'], anchor_from_neighbor) if side == 'earlier' else
                           pose_compose(fit['pose'], pose_inverse(anchor_from_neighbor)))
            difference = pose_between(event['pose'], transported)
            coverage = geometric_coverage(scans[n], opposite, fit['pose'], .15)
            good = (_fit_good(fit, parameters, neighbor=True)
                    and coverage['source_fraction'] >= parameters.neighbor_min_geometric_coverage
                    and np.linalg.norm(difference[:2]) <= parameters.max_neighbor_disagreement_m
                    and abs(difference[2]) <= parameters.max_neighbor_disagreement_rad)
            checks.append({'accepted': bool(good), 'dense_index': n, 'timestamp_ns': int(times[n]),
                           'construction_disjoint': True, 'fit': _fit_summary(fit), 'coverage': coverage,
                           'transported_measurement': transported, 'difference_from_anchor': difference})
        return checks

    for j in range(len(keys)):
        if any(abs(int(key_times[j])-int(key_times[e['j']])) < 8e9 for e in selected):
            skipped.append({'keyframe': j, 'reason': 'source_anchor_reserved_by_existing_or_selected_loop'})
            continue
        distances = np.linalg.norm(key_poses[:j, :2]-key_poses[j, :2], axis=1)
        elapsed = (key_times[j]-key_times[:j])*1e-9
        yaw = np.abs(wrap_angle(key_poses[:j, 2]-key_poses[j, 2]))
        eligible = np.flatnonzero((distances < 3.) & (elapsed > 35.))
        ranked = sorted(eligible, key=lambda i: distances[i]+.2*yaw[i])
        proposals = []
        for i in ranked:
            if np.linalg.norm(key_poses[i:j+1, :2]-key_poses[i, :2], axis=1).max() < 3.:
                continue
            if any(abs(int(key_times[i])-int(key_times[p])) < 3e9 for p in proposals):
                continue
            proposals.append(int(i))
            if len(proposals) == 3:
                break
        for i in proposals:
            event_base = {'i': i, 'j': j, 'dense_i': int(keys[i]), 'dense_j': int(keys[j]),
                          'timestamp_i_ns': int(key_times[i]), 'timestamp_j_ns': int(key_times[j]),
                          'time_gap_s': float(elapsed[i]), 'candidate_distance_m': float(distances[i]),
                          'candidate_yaw_rad': float(yaw[i]), 'previous_yaw_gate_would_exclude': bool(yaw[i] >= 1.2)}
            if any(abs(int(key_times[i])-int(key_times[e['i']])) < 8e9 for e in selected):
                events.append({**event_base, 'accepted': False, 'reason': 'correlated_target_anchor'})
                continue
            guess = pose_between(poses[keys[i]], poses[keys[j]])
            event = {**verify_revisit(submap(j), submap(i), guess, parameters), **event_base}
            event['construction_dense_indices_earlier'] = members_cache[i]
            event['construction_dense_indices_later'] = members_cache[j]
            if event['accepted']:
                event['earlier_temporal_support'] = temporal_side(event, 'earlier')
                event['later_temporal_support'] = temporal_side(event, 'later')
                event['accepted'] = all(check['accepted'] for side in ('earlier_temporal_support', 'later_temporal_support') for check in event[side])
                event['reason'] = 'bidirectional_temporal_supported' if event['accepted'] else 'bidirectional_temporal_inconsistency'
            if event['accepted']:
                cycles = []
                for previous in selected:
                    if abs(int(key_times[i])-int(key_times[previous['i']])) <= 20e9 and abs(int(key_times[j])-int(key_times[previous['j']])) <= 20e9:
                        prediction = transport_closure(previous['pose'], pose_between(key_poses[i], key_poses[previous['i']]), pose_between(key_poses[previous['j']], key_poses[j]))
                        difference = pose_between(prediction, event['pose'])
                        cycles.append({'other_edge': [previous['i'], previous['j']], 'difference': difference,
                                       'accepted': bool(np.linalg.norm(difference[:2]) <= .12 and abs(difference[2]) <= .05)})
                event['local_interloop_cycles'] = cycles
                event['accepted'] = all(cycle['accepted'] for cycle in cycles)
                event['reason'] = 'accepted' if event['accepted'] else 'local_interloop_cycle_inconsistency'
            events.append(event)
            if event['accepted']:
                selected.append({'i': i, 'j': j, 'pose': np.asarray(event['pose']).tolist(), 'loop': True, 'sensor': 'lidar_revisit_refinement'})
                print(f'{profile} added {i}->{j}: rmse={event["forward"]["rmse"]:.4f}m, yaw={yaw[i]:.3f}rad', flush=True)
                break
        if j % 50 == 0:
            print(f'{profile}: keyframe {j}/{len(keys)}, tested={len(events)}, added={len(selected)-len(existing_edges)}', flush=True)
    return selected, events, sorted(used), {'registration': asdict(parameters), 'profile': profile,
        'candidate_cap': 3*len(keys), 'anchor_separation_both_passes_s': 8.,
        'local_interloop_cycle_window_s': 20., 'local_interloop_cycle_translation_m': .12,
        'local_interloop_cycle_yaw_rad': .05, 'skipped_keyframes': skipped}
