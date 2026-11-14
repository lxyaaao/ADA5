"""Experimental exact-subset PCD agreement with an existing occupancy grid.

The grid and point cloud share sensor observations. This is a consistency mask,
not independent obstacle truth, and excluded points do not establish free space.
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import distance_transform_edt


def filter_occupancy_supported(pointsXYZ, grid_image, metadata):
    """Retain mapped cells within a fixed 0.10m of an occupied cell center.

    ``grid_image`` is the original top-row-first uint8 grayscale PGM array.
    YAML's origin includes yaw; its XY describes the lower-left grid corner.
    Distances are between *cell centers*, not each point and an occupied center.
    Returns exact original ``(retainedXYZ, statistics, Boolean keepMask)``.
    No smoothing, grid mutation, interpolation, or coordinate changes occur.
    """
    points = np.asarray(pointsXYZ)
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
        raise ValueError('Expected finite Nx3 pointsXYZ')
    if np.any(points[:, 2] != 0):
        raise ValueError('Expected planar LiDAR z=0; visual landmarks must remain separate')
    image = np.asarray(grid_image)
    if image.ndim != 2 or not image.size or not np.all(np.isfinite(image)) or np.any((image < 0) | (image > 255)):
        raise ValueError('Expected nonempty grayscale image values in [0,255]')
    origin = np.asarray(metadata['origin'], float)
    resolution = float(metadata['resolution'])
    occupied_threshold, free_threshold = float(metadata['occupied_thresh']), float(metadata['free_thresh'])
    negate = metadata.get('negate', 0)
    if origin.shape != (3,) or not np.all(np.isfinite(origin)):
        raise ValueError('YAML origin must be finite [x,y,yaw]')
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError('YAML resolution must be positive')
    if metadata.get('mode', 'trinary') != 'trinary' or negate not in (0, 1, False, True):
        raise ValueError('Expected trinary YAML mode and negate 0 or 1')
    if not 0 <= free_threshold < occupied_threshold <= 1:
        raise ValueError('Invalid YAML occupancy thresholds')
    gray = image.astype(float)/255.
    probability = gray if negate else 1.-gray
    occupied = probability > occupied_threshold
    free = probability < free_threshold
    unknown = ~(occupied | free)
    height, width = image.shape
    delta = np.asarray(points[:, :2], float)-origin[:2]
    c, s = np.cos(origin[2]), np.sin(origin[2])
    local = np.column_stack([c*delta[:, 0]+s*delta[:, 1],
                             -s*delta[:, 0]+c*delta[:, 1]])
    inside = (local[:, 0] >= 0) & (local[:, 1] >= 0) & (local[:, 0] < width*resolution) & (local[:, 1] < height*resolution)
    inside_ids = np.flatnonzero(inside)
    cells = np.floor(local[inside]/resolution).astype(np.int64)
    rows, columns = height-1-cells[:, 1], cells[:, 0]
    no_occupied = not np.any(occupied)
    distance_grid = (np.full(image.shape, np.inf) if no_occupied else
                     distance_transform_edt(~occupied, sampling=resolution))
    distances = np.full(len(points), np.inf)
    distances[inside] = distance_grid[rows, columns]
    keep = inside & (distances <= .10 + 1e-12)
    categories = np.full(len(points), 'outside_grid', dtype='<U12')
    categories[inside] = 'unknown'
    categories[inside_ids[occupied[rows, columns]]] = 'occupied'
    categories[inside_ids[free[rows, columns]]] = 'free'
    retention = {}
    for name in ('occupied', 'free', 'unknown', 'outside_grid'):
        selected = categories == name
        retention[name] = {'input': int(selected.sum()), 'retained': int((selected & keep).sum()),
                           'removed': int((selected & ~keep).sum())}
    finite_distances = distances[np.isfinite(distances)]
    stats = {
        'candidate': 'occupancy_supported',
        'status': 'experimental; requires exact-exported-PCD development evaluation',
        'parameters': {'maximum_cell_center_distance_m': .10,
                       'method': 'Euclidean distance transform from original occupied cell centers',
                       'occupied_comparison': 'probability > occupied_thresh (strict)',
                       'free_comparison': 'probability < free_thresh (strict)',
                       'image_row_orientation': 'row zero is highest local grid Y',
                       'point_to_cell': 'inverse YAML origin/yaw, floor(localXY/resolution); no interpolation'},
        'grid_metadata': {'width': width, 'height': height, 'resolution': resolution,
                          'origin': origin.tolist(), 'negate': int(negate), 'mode': 'trinary',
                          'occupied_thresh': occupied_threshold, 'free_thresh': free_threshold},
        'grid_cell_counts': {'occupied': int(occupied.sum()), 'free': int(free.sum()), 'unknown': int(unknown.sum())},
        'input_points': len(points), 'retained_points': int(keep.sum()),
        'removed_points': int((~keep).sum()),
        'removed_fraction': float((~keep).mean()) if len(points) else 0.,
        'retention_by_original_cell_class': retention,
        'no_occupied_cells_all_rejected': bool(no_occupied),
        'mapped_cell_distance_quantiles_m': dict(zip(('min', 'median', 'p95', 'max'),
                                                    np.quantile(finite_distances, [0, .5, .95, 1.]).tolist())) if len(finite_distances) else {},
        'exact_subset_no_coordinate_changes': True,
        'limitations': [
            'Occupancy grid and PCD come from the same observations; grid agreement is not independent truth.',
            'Valid sparse or transient surfaces can be excluded, including points in indeterminate cells.',
            'Unknown is the trinary intermediate class; it does not distinguish unobserved from weakly observed cells.',
            'Excluded points are masked out, not newly declared free; original occupancy context is unchanged.',
            'This bounded 0.10m candidate is not a parameter sweep or a physical-accuracy claim.',
        ],
    }
    return points[keep], stats, keep
