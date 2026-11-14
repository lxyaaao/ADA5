"""Conservative density cleanup of a planar LiDAR map, without moving points.

The radius test is supplemented by local straight-segment protection so sparse
walls, endpoints, and corner branches are not rejected merely for low density.
This is not semantic obstacle removal: dense clutter and coherent ghost segments
can survive, and deleting a point does not establish free space.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class FilterParameters:
    density_radius_m: float = .12
    minimum_other_neighbors: int = 3
    enable_line_protection: bool = True
    line_radius_m: float = .45
    line_max_neighbors: int = 24
    line_minimum_other_support: int = 4
    line_max_perpendicular_distance_m: float = .025
    line_minimum_span_m: float = .18


PRESETS = {
    'conservative': FilterParameters(minimum_other_neighbors=3),
    'balanced': FilterParameters(minimum_other_neighbors=6),
    'strong': FilterParameters(minimum_other_neighbors=10),
    'radius_conservative': FilterParameters(minimum_other_neighbors=3, enable_line_protection=False),
    'radius_balanced': FilterParameters(minimum_other_neighbors=6, enable_line_protection=False),
}
QUANTILES = np.array([0, .01, .05, .25, .5, .75, .95, .99, 1.])


def _quantiles(values):
    return dict(zip((f'{q:g}' for q in QUANTILES),
                    np.quantile(values, QUANTILES).tolist())) if len(values) else {}


def _line_supported(offsets, parameters):
    """Test actual neighbor segments; never fit/move the candidate coordinate."""
    norms = np.linalg.norm(offsets, axis=1)
    offsets = offsets[(norms > 1e-10) & (norms <= parameters.line_radius_m)]
    if len(offsets) < parameters.line_minimum_other_support:
        return False
    directions = offsets / np.linalg.norm(offsets, axis=1)[:, None]
    # Each neighbor supplies one deterministic line hypothesis through the point.
    perpendicular = np.abs(directions[:, 0, None]*offsets[None, :, 1]
                           - directions[:, 1, None]*offsets[None, :, 0])
    aligned = perpendicular <= parameters.line_max_perpendicular_distance_m
    for direction, support in zip(directions, aligned):
        if np.count_nonzero(support) < parameters.line_minimum_other_support:
            continue
        tangent = offsets[support] @ direction
        # Include the actual candidate in the segment's extent; one-sided wall
        # endpoints remain valid, as do either branch of a sparse true corner.
        span = max(0., tangent.max()) - min(0., tangent.min())
        if span >= parameters.line_minimum_span_m:
            return True
    return False


def filter_points(pointsXYZ, preset='conservative'):
    """Return ``(retainedXYZ, stats, keepMask)`` with exact original coordinates.

    Input is finite Nx3 planar LiDAR geometry (z exactly zero), normally the
    project's 0.03 m voxel-centroid map. The function itself does not resample.
    For protocol evaluation, build the allowed reference map at the same 0.03 m
    spacing before filtering, then apply the evaluator's registration sampling.
    All counts exclude the candidate itself. Presets are deterministic; each
    family is nested. ``radius_*`` presets disable the segment protection for
    the bounded plain radius-outlier-removal comparison.
    """
    if preset not in PRESETS:
        raise ValueError(f'Unknown filter preset: {preset}; choose {tuple(PRESETS)}')
    points = np.asarray(pointsXYZ)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('pointsXYZ must have shape (N,3)')
    if not np.all(np.isfinite(points)):
        raise ValueError('pointsXYZ must contain only finite coordinates')
    if np.any(points[:, 2] != 0):
        raise ValueError('This filter accepts planar LiDAR z=0 only; do not project visual landmarks')
    parameters = PRESETS[preset]
    counts = np.zeros(len(points), dtype=np.int64)
    protected = np.zeros(len(points), dtype=bool)
    nearest = np.empty(0)
    if len(points):
        xy = np.asarray(points[:, :2], dtype=float)
        tree = cKDTree(xy)
        counts = tree.query_ball_point(xy, parameters.density_radius_m,
                                      return_length=True, workers=-1)-1
        dense = counts >= parameters.minimum_other_neighbors
        candidate_ids = np.flatnonzero(~dense)
        if len(points) > 1:
            nearest = tree.query(xy, k=2, workers=-1)[0][:, 1]
        if len(candidate_ids) and parameters.enable_line_protection:
            k = min(len(points), parameters.line_max_neighbors+1)
            distances, indices = tree.query(xy[candidate_ids], k=k, workers=-1)
            if k == 1:
                distances, indices = distances[:, None], indices[:, None]
            for row, index in enumerate(candidate_ids):
                neighbor_ids = indices[row][(indices[row] != index)
                                            & (distances[row] <= parameters.line_radius_m)]
                protected[index] = _line_supported(xy[neighbor_ids]-xy[index], parameters)
        keep = dense | protected
    else:
        dense = np.zeros(0, dtype=bool)
        keep = dense.copy()
    stats = {
        'preset': preset,
        'parameters': asdict(parameters),
        'input_representation': {'production_voxel_m': .03,
                                 'resampling_in_filter': False,
                                 'units': 'metres', 'plane': 'z=0'},
        'input_points': len(points), 'retained_points': int(keep.sum()),
        'removed_points': int((~keep).sum()),
        'removed_fraction': float((~keep).mean()) if len(keep) else 0.,
        'density_passing_points': int(dense.sum()),
        'density_failing_points': int((~dense).sum()),
        'line_protected_points': int(protected.sum()),
        'input_neighbor_count_quantiles': _quantiles(counts),
        'input_nearest_distance_m_quantiles': _quantiles(nearest),
        'exact_subset_no_coordinate_changes': True,
        'limitations': [
            'Dense clutter and geometrically coherent ghost segments can survive.',
            'Sparse true features may be removed; localization needs separate validation.',
            'A removed point is not evidence of free space; the original occupancy grid is unchanged.',
            'Density differs between full-map and separated-reference evaluation subsets.',
        ],
    }
    return points[keep], stats, keep


def read_ascii_pcd(path):
    """Read this project's ASCII XYZ PCD and retain every data line verbatim."""
    lines = Path(path).read_text(encoding='ascii').splitlines()
    data_index = next((i for i, line in enumerate(lines)
                       if line.strip().upper().startswith('DATA ')), None)
    if data_index is None or lines[data_index].strip().lower() != 'data ascii':
        raise ValueError('Only ASCII XYZ PCD is supported by this subset writer')
    header = lines[:data_index+1]
    fields = {}
    for line in header:
        parts = line.split()
        if parts and not parts[0].startswith('#'):
            fields[parts[0].upper()] = parts[1:]
    if fields.get('FIELDS') != ['x', 'y', 'z'] or fields.get('COUNT', ['1']*3) != ['1']*3:
        raise ValueError('PCD must contain exactly scalar x y z fields')
    rows = [line for line in lines[data_index+1:] if line.strip()]
    if any(len(row.split()) != 3 for row in rows):
        raise ValueError('PCD rows must contain exactly three coordinates')
    points = np.array([list(map(float, row.split())) for row in rows], dtype=float).reshape(-1, 3)
    if len(rows) != int(fields['POINTS'][0]):
        raise ValueError('PCD header point count does not match data rows')
    return points, header, rows


def write_pcd_subset(path, header, data_rows, keepMask):
    """Copy selected original ASCII rows, updating only count/shape metadata."""
    keep = np.asarray(keepMask)
    if keep.dtype != bool or keep.shape != (len(data_rows),):
        raise ValueError('keepMask must be one boolean per original data row')
    count = int(keep.sum())
    output_header = []
    for line in header:
        fields = line.split()
        key = fields[0].upper() if fields else ''
        output_header.append(f'WIDTH {count}' if key == 'WIDTH' else
                             'HEIGHT 1' if key == 'HEIGHT' else
                             f'POINTS {count}' if key == 'POINTS' else line)
    selected = [row for row, retained in zip(data_rows, keep) if retained]
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text('\n'.join(output_header+selected)+'\n', encoding='ascii', newline='\n')
    return target
