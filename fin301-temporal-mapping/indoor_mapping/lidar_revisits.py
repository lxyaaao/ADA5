"""Conservative LiDAR revisit proposals for independent accuracy experiments.

This module proposes relative SE(2) measurements; it never assumes a revisit has
zero displacement and never changes the delivered pipeline or map on its own.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
from scipy.spatial import cKDTree

from .geometry import pose_between, pose_compose, pose_inverse, register_scan, transform_points, voxel_downsample, wrap_angle


@dataclass(frozen=True)
class RevisitParameters:
    min_elapsed_s: float = 35.0
    min_excursion_m: float = 3.0
    max_candidate_distance_m: float = 3.0
    max_candidate_yaw_rad: float = 1.2
    max_candidates_per_keyframe: int = 3
    min_loop_spacing_s: float = 8.0
    submap_half_width: int = 2
    submap_voxel_m: float = .07
    correspondence_gate_m: float = .35
    geometric_tolerance_m: float = .15
    min_geometric_coverage: float = .75
    min_mean_geometric_coverage: float = .75
    min_normal_support: float = .40
    max_line_rmse_m: float = .035
    max_euclidean_rmse_m: float = .08
    max_condition: float = 250.0
    min_matched_points: int = 120
    min_minor_spread_m: float = .25
    max_odometry_correction_m: float = .8
    max_odometry_correction_rad: float = .20
    max_cycle_translation_m: float = .04
    max_cycle_yaw_rad: float = .02
    max_initialization_spread_m: float = .05
    max_initialization_spread_rad: float = .025
    min_converged_initializations: int = 5
    neighbor_offset_scans: int = 6
    neighbor_min_normal_support: float = .35
    neighbor_min_geometric_coverage: float = .50
    neighbor_max_line_rmse_m: float = .045
    max_neighbor_disagreement_m: float = .08
    max_neighbor_disagreement_rad: float = .035


def anchor_constraint_from_neighbor(target_from_neighbor, anchor_from_neighbor):
    """Convert target-from-neighbor into target-from-anchor using short odometry."""
    return pose_compose(target_from_neighbor, pose_inverse(anchor_from_neighbor))


def _fit_summary(result):
    return {key: result[key] for key in ('pose', 'success', 'reason', 'rmse', 'euclidean_rmse', 'inlier_ratio', 'condition', 'degenerate')}


def _fit_good(result, parameters, neighbor=False):
    overlap = parameters.neighbor_min_normal_support if neighbor else parameters.min_normal_support
    residual = parameters.neighbor_max_line_rmse_m if neighbor else parameters.max_line_rmse_m
    return bool(result['success'] and not result['degenerate'] and result['inlier_ratio'] >= overlap
                and result['rmse'] <= residual and result['euclidean_rmse'] <= parameters.max_euclidean_rmse_m
                and result['condition'] <= parameters.max_condition)


def geometric_coverage(source, target, pose, tolerance=.15):
    """Measure actual nearest-counterpart coverage independently of PCA normals."""
    moved = transform_points(source, pose)
    forward, _ = cKDTree(target).query(moved)
    reverse, _ = cKDTree(moved).query(target)
    return {'source_fraction': float(np.mean(forward <= tolerance)),
            'target_fraction': float(np.mean(reverse <= tolerance)),
            'source_matched_points': int(np.count_nonzero(forward <= tolerance)),
            'target_matched_points': int(np.count_nonzero(reverse <= tolerance)),
            'tolerance_m': tolerance}


def _coverage_good(coverage, parameters):
    source, target = coverage['source_fraction'], coverage['target_fraction']
    return min(source, target) >= parameters.min_geometric_coverage and (source + target) / 2 >= parameters.min_mean_geometric_coverage


def verify_revisit(source, target, guess, parameters=None):
    """Require broad support, reciprocal registration, and five-start agreement."""
    parameters = parameters or RevisitParameters()
    source, target = np.asarray(source), np.asarray(target)
    guess = np.asarray(guess, dtype=float)
    result = register_scan(source, target, initial=guess,
                           max_distance=parameters.correspondence_gate_m, iterations=36)
    evidence = {'accepted': False, 'reason': 'initial_fit', 'pose': result['pose'],
                'odometry_relative_pose': guess, 'forward': _fit_summary(result), 'initialization_trials': []}
    if not _fit_good(result, parameters):
        return evidence
    evidence['geometric_coverage'] = geometric_coverage(source, target, result['pose'], parameters.geometric_tolerance_m)
    if not _coverage_good(evidence['geometric_coverage'], parameters):
        evidence['reason'] = 'insufficient_geometric_coverage'
        return evidence
    correction = pose_between(guess, result['pose'])
    evidence['odometry_correction'] = correction
    if np.linalg.norm(correction[:2]) > parameters.max_odometry_correction_m or abs(correction[2]) > parameters.max_odometry_correction_rad:
        evidence['reason'] = 'large_odometry_disagreement'
        return evidence
    moved = transform_points(source, result['pose'])
    distances, _ = cKDTree(target).query(moved)
    matched = moved[distances <= .2]
    minor = float(np.sqrt(max(0., np.linalg.eigvalsh(np.cov(matched.T))[0]))) if len(matched) >= 3 else 0.
    evidence.update(matched_points=len(matched), minor_spread_m=minor)
    if len(matched) < parameters.min_matched_points or minor < parameters.min_minor_spread_m:
        evidence['reason'] = 'narrow_geometric_support'
        return evidence
    reverse = register_scan(target, source, initial=pose_inverse(result['pose']),
                            max_distance=parameters.correspondence_gate_m, iterations=36)
    cycle = pose_compose(result['pose'], reverse['pose'])
    evidence.update(reverse=_fit_summary(reverse), reciprocal_cycle=cycle)
    if not _fit_good(reverse, parameters) or np.linalg.norm(cycle[:2]) > parameters.max_cycle_translation_m or abs(cycle[2]) > parameters.max_cycle_yaw_rad:
        evidence['reason'] = 'reciprocal_inconsistency'
        return evidence
    # Fixed perturbations are specified before observing candidate outcomes.
    qualifying = 0
    competing_mode = False
    for perturbation in ([0., 0., 0.], [.18, 0., 0.], [-.18, 0., 0.], [0., .18, .04], [0., -.18, -.04]):
        trial = result if not np.any(perturbation) else register_scan(
            source, target, initial=guess + perturbation,
            max_distance=parameters.correspondence_gate_m, iterations=36)
        difference = pose_between(result['pose'], trial['pose'])
        coverage = geometric_coverage(source, target, trial['pose'], parameters.geometric_tolerance_m)
        good = _fit_good(trial, parameters) and _coverage_good(coverage, parameters)
        agrees = np.linalg.norm(difference[:2]) <= parameters.max_initialization_spread_m and abs(difference[2]) <= parameters.max_initialization_spread_rad
        qualifying += int(good and agrees)
        competing_mode |= bool(good and not agrees)
        evidence['initialization_trials'].append({**_fit_summary(trial), 'initial': guess + perturbation,
                                                 'difference_from_first': difference, 'geometric_coverage': coverage,
                                                 'qualifies': bool(good), 'agrees_with_first': bool(agrees)})
    evidence['qualifying_agreeing_initializations'] = qualifying
    if competing_mode or qualifying < parameters.min_converged_initializations:
        evidence['reason'] = 'competing_supported_mode' if competing_mode else 'insufficient_initialization_support'
        return evidence
    evidence.update(accepted=True, reason='pair_geometry_supported')
    return evidence


def propose_revisits(scans, poses, key_ids, times_ns, parameters=None):
    """Find independently spaced, temporally supported relative-pose constraints.

    All scan arrays and poses are aligned at accepted dense trajectory indices.
    Returns loop edges, per-candidate evidence, and training dense scan indices.
    Neighbor scans are separated from the five-scan center submap by about one
    second at the pipeline's usual accepted-scan cadence.
    """
    parameters = parameters or RevisitParameters()
    poses, key_ids = np.asarray(poses), np.asarray(key_ids, dtype=int)
    times = np.asarray(times_ns, dtype=np.int64)
    if len(scans) != len(poses) or len(times) != len(poses):
        raise ValueError('scans, poses and timestamps must share dense trajectory indices')
    key_poses, key_times = poses[key_ids], times[key_ids]
    submaps, used_indices = {}, set()

    def submap(anchor):
        if anchor not in submaps:
            members = range(max(0, anchor - parameters.submap_half_width),
                            min(len(poses), anchor + parameters.submap_half_width + 1))
            used_indices.update(members)
            submaps[anchor] = voxel_downsample(np.vstack([
                transform_points(scans[index], pose_between(poses[anchor], poses[index])) for index in members
            ]), parameters.submap_voxel_m)
        return submaps[anchor]

    events, edges = [], []
    last_accepted_time = -10**30
    for j in range(len(key_ids)):
        if int(key_times[j]) - last_accepted_time < parameters.min_loop_spacing_s * 1e9:
            continue
        delta = (key_times[j] - key_times[:j]) * 1e-9
        distance = np.linalg.norm(key_poses[:j, :2] - key_poses[j, :2], axis=1)
        yaw = np.abs(wrap_angle(key_poses[:j, 2] - key_poses[j, 2]))
        candidates = np.flatnonzero((delta > parameters.min_elapsed_s) & (distance < parameters.max_candidate_distance_m)
                                    & (yaw < parameters.max_candidate_yaw_rad))
        ranked = sorted(candidates, key=lambda i: distance[i] + .2 * yaw[i])
        selected = []
        for i in ranked:
            excursion = np.linalg.norm(key_poses[i:j+1, :2] - key_poses[i, :2], axis=1).max()
            if excursion < parameters.min_excursion_m or any(abs(int(key_times[i]) - int(key_times[previous])) < 3e9 for previous in selected):
                continue
            selected.append(int(i))
            if len(selected) >= parameters.max_candidates_per_keyframe:
                break
        for i in selected:
            a, b = int(key_ids[i]), int(key_ids[j])
            guess = pose_between(poses[a], poses[b])
            event = verify_revisit(submap(b), submap(a), guess, parameters)
            event.update(i=i, j=j, dense_i=a, dense_j=b, timestamp_i_ns=int(times[a]), timestamp_j_ns=int(times[b]),
                         time_i_s=float((times[a] - times[0]) * 1e-9), time_j_s=float((times[b] - times[0]) * 1e-9),
                         candidate_distance_m=float(distance[i]), candidate_yaw_rad=float(yaw[i]))
            event['pair_geometry_accepted'] = event['accepted']
            if event['accepted']:
                supports = []
                for direction in (-1, 1):
                    neighbor = b + direction * parameters.neighbor_offset_scans
                    if not 0 <= neighbor < len(poses):
                        supports.append({'accepted': False, 'reason': 'neighbor_outside_trajectory'})
                        continue
                    used_indices.add(neighbor)
                    anchor_from_neighbor = pose_between(poses[b], poses[neighbor])
                    predicted = pose_compose(event['pose'], anchor_from_neighbor)
                    fit = register_scan(scans[neighbor], submap(a), initial=predicted,
                                        max_distance=parameters.correspondence_gate_m, iterations=36)
                    transported = anchor_constraint_from_neighbor(fit['pose'], anchor_from_neighbor)
                    difference = pose_between(event['pose'], transported)
                    coverage = geometric_coverage(scans[neighbor], submap(a), fit['pose'], parameters.geometric_tolerance_m)
                    good = _fit_good(fit, parameters, neighbor=True) and coverage['source_fraction'] >= parameters.neighbor_min_geometric_coverage and np.linalg.norm(difference[:2]) <= parameters.max_neighbor_disagreement_m and abs(difference[2]) <= parameters.max_neighbor_disagreement_rad
                    supports.append({'accepted': bool(good), 'dense_index': neighbor, 'timestamp_ns': int(times[neighbor]),
                                     'fit': _fit_summary(fit), 'transported_anchor_measurement': transported,
                                     'difference_from_anchor': difference, 'geometric_coverage': coverage})
                event['temporal_support'] = supports
                event['accepted'] = bool(all(support['accepted'] for support in supports))
                event['reason'] = 'accepted' if event['accepted'] else 'temporal_inconsistency'
            events.append(event)
            if event['accepted']:
                edges.append({'i': i, 'j': j, 'pose': event['pose'].tolist(), 'loop': True, 'sensor': 'lidar_revisit'})
                last_accepted_time = int(key_times[j])
                print(f'LiDAR revisit proposal {i}->{j}: line RMSE {event["forward"]["rmse"]:.4f} m; overlap {event["forward"]["inlier_ratio"]:.3f}', flush=True)
                break
        if j % 50 == 0:
            print(f'Revisit experiment keyframe {j}/{len(key_ids)}; tested {len(events)}, proposed {len(edges)}', flush=True)
    return edges, events, sorted(used_indices), asdict(parameters)
