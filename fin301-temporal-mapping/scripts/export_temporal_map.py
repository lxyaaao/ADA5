"""Export temporal PCD subsets with conservative, matched YAML/PGM companions.

Only unsupported occupied grid cells change, to unknown. Existing free/unknown
observations are preserved; removing a point never creates navigable free space.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from indoor_mapping.map_filter import read_ascii_pcd
from indoor_mapping.visual_landmarks import save_xyz_pcd


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def companion_grid(image, metadata, retained, support_distance=.10):
    """Keep occupied centers within 10cm of retained PCD; mask others unknown.

    This raster is a conservative derivative of the original occupancy map,
    not a new occupancy reconstruction or a new estimate of free space.
    """
    image = np.asarray(image)
    points = np.asarray(retained, float)
    origin = np.asarray(metadata['origin'], float)
    resolution = float(metadata['resolution'])
    if image.ndim != 2 or image.dtype != np.uint8:
        raise ValueError('Expected an 8-bit grayscale grid')
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)) or np.any(points[:, 2] != 0):
        raise ValueError('Expected finite planar Nx3 retained points')
    if origin.shape != (3,) or not np.all(np.isfinite(origin)) or not np.isfinite(resolution) or resolution <= 0:
        raise ValueError('Invalid YAML coordinate definition')
    free_threshold, occupied_threshold = float(metadata['free_thresh']), float(metadata['occupied_thresh'])
    negate = metadata.get('negate', 0)
    if metadata.get('mode', 'trinary') != 'trinary' or negate not in (0, 1) or not 0 <= free_threshold < occupied_threshold <= 1:
        raise ValueError('Invalid trinary thresholds/mode')
    gray = image.astype(float)/255.
    probability = gray if negate else 1-gray
    occupied = probability > occupied_threshold
    free = probability < free_threshold
    unknown = ~(occupied | free)
    # Choose an exactly representable unknown pixel, honoring negate/thresholds.
    possible = np.arange(256)
    probs = possible/255. if negate else 1-possible/255.
    choices = possible[(probs >= free_threshold) & (probs <= occupied_threshold)]
    if not len(choices):
        raise ValueError('No representable unknown pixel at these thresholds')
    unknown_value = int(205 if 205 in choices else choices[len(choices)//2])
    rows, cols = np.nonzero(occupied)
    local = np.column_stack(((cols+.5)*resolution, (image.shape[0]-rows-.5)*resolution))
    c, s = np.cos(origin[2]), np.sin(origin[2])
    centers = local @ np.array([[c, s], [-s, c]]) + origin[:2]
    distances = cKDTree(points[:, :2]).query(centers)[0] if len(points) else np.full(len(rows), np.inf)
    removed = distances > support_distance+1e-12
    result = image.copy()
    result[rows[removed], cols[removed]] = unknown_value
    assert np.array_equal(result[free | unknown], image[free | unknown])
    return result, {
        'method': 'Original occupied cell centers without retained LiDAR point support are masked to unknown',
        'support_distance_m': support_distance, 'unknown_pixel': unknown_value,
        'occupied_cells_original': int(occupied.sum()), 'occupied_cells_retained': int((~removed).sum()),
        'occupied_cells_masked_to_unknown': int(removed.sum()),
        'free_cells_preserved': int(free.sum()), 'original_unknown_cells_preserved': int(unknown.sum()),
        'new_free_cells': 0, 'new_occupied_cells': 0,
        'interpretation': 'Conservative raster companion; no free-space inference from removed point endpoints',
    }


def export(source, experiment, profile):
    output = experiment/profile/'result'
    if output.resolve().is_relative_to(source.resolve()):
        raise ValueError('Output must be outside original result')
    candidate = experiment/f'lidar_scan_map_clean_temporal_{profile}.pcd'
    report = json.loads((experiment/'temporal_filter_report.json').read_text(encoding='utf-8'))
    expected = report['candidates'][profile]
    if digest(candidate) != expected['sha256'] or digest(source/'lidar_scan_map.pcd') != report['source_sha256']['source_pcd']:
        raise ValueError('Temporal source/candidate checksum mismatch')
    lidar, _, _ = read_ascii_pcd(candidate)
    visual, _, _ = read_ascii_pcd(source/'visual_landmarks.pcd')
    if len(lidar) != expected['retained_points']:
        raise ValueError('Unexpected candidate point count')
    metadata = json.loads((source/'map_metadata.json').read_text(encoding='utf-8'))
    grid_metadata = yaml.safe_load((source/'map.yaml').read_text(encoding='utf-8'))
    grid_path = (source/grid_metadata['image']).resolve()
    if not grid_path.is_relative_to(source.resolve()):
        raise ValueError('Source grid escapes result directory')
    original_grid = np.asarray(Image.open(grid_path).convert('L'))
    filtered_grid, grid_report = companion_grid(original_grid, grid_metadata, lidar)
    output.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(candidate, output/'lidar_scan_map.pcd')
    for name in ('visual_landmarks.pcd', 'visual_landmark_stats.json', 'visual_landmark_evidence.csv',
                 'trajectory.csv', 'trajectory_arrays.npz', 'odometry.npz', 'pose_graph.json'):
        shutil.copyfile(source/name, output/name)
    # Match the established float32 PCD export convention for the combined map.
    save_xyz_pcd(np.vstack([lidar, visual]).astype(np.float32), output/'map.pcd')
    Image.fromarray(filtered_grid).save(output/'map.pgm')
    Image.fromarray(filtered_grid).save(output/'map.png')
    grid_metadata['image'] = 'map.pgm'
    (output/'map.yaml').write_text(yaml.safe_dump(grid_metadata, sort_keys=False), encoding='utf-8')
    description = ('Conservative temporal filter: removes strong visible-free conflicts; retains insufficient evidence'
                   if profile == 'conservative' else 'Strict temporal support: retains repeated hits; also excludes insufficient evidence')
    metadata.update(points=len(lidar)+len(visual), lidar_point_count=len(lidar), visual_point_count=len(visual),
                    viewer_title=f'fin301 - {profile.capitalize()} temporal map',
                    temporal_filter=description, camera_landmarks_temporally_filtered=False,
                    trajectory_note='Original delivered one-loop trajectory unchanged',
                    grid_derivation=grid_report,
                    temporal_evidence='../../temporal_filter_report.json')
    (output/'map_metadata.json').write_text(json.dumps(metadata, indent=2)+'\n', encoding='utf-8')
    metrics = json.loads((source/'metrics.json').read_text(encoding='utf-8'))
    metrics.update(pcd_points=len(lidar)+len(visual), lidar_point_count=len(lidar), visual_point_count=len(visual),
                   temporal_filter=expected, inherited_tracking_metrics='Unchanged original one-loop run; no new pose estimation',
                   absolute_accuracy_validated=False)
    (output/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n', encoding='utf-8')
    source_names = ['lidar_scan_map.pcd', 'map.yaml', 'map.pgm', 'trajectory.csv', 'trajectory_arrays.npz', 'visual_landmarks.pcd']
    provenance = {'profile': profile, 'description': description, 'source_result': str(source),
                  'source_sha256': {n:digest(source/n) for n in source_names},
                  'candidate_sha256': digest(candidate), 'filter_report_sha256': digest(experiment/'temporal_filter_report.json'),
                  'grid': grid_report, 'exact_subset_pcd': digest(output/'lidar_scan_map.pcd') == digest(candidate),
                  'original_trajectory_and_camera_unchanged': True, 'absolute_accuracy_validated': False,
                  'output_sha256': {p.name:digest(p) for p in output.iterdir() if p.is_file() and p.name != 'export_provenance.json'}}
    (output/'export_provenance.json').write_text(json.dumps(provenance, indent=2)+'\n', encoding='utf-8')
    return {'profile':profile, 'output':str(output), 'points':len(lidar), 'grid':grid_report}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT/'outputs/fin301/result')
    parser.add_argument('--experiment', type=Path, default=ROOT/'outputs/fin301/temporal_filter')
    args = parser.parse_args()
    print(json.dumps([export(args.source, args.experiment, p) for p in ('conservative', 'static')], indent=2))


if __name__ == '__main__':
    main()
