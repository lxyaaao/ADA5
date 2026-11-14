"""Observation-supported planar points for a bounded map representation comparison."""
import numpy as np


def persistent_points(scans_in_map, voxel_size=.10, minimum_scans=2):
    """Keep voxel centroids supported by distinct scans, without fitting surfaces.

    Repeated beams inside one scan count once. Support is not proof of a static
    physical surface; coherent motion, reflections and map drift can survive.
    """
    if voxel_size <= 0 or minimum_scans < 1:
        raise ValueError('Require positive voxel size and scan support')
    representatives = []
    for scan in scans_in_map:
        points = np.asarray(scan, float).reshape(-1, 2)
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            continue
        cells, inverse = np.unique(np.floor(points / voxel_size).astype(np.int64), axis=0, return_inverse=True)
        sums = np.zeros((len(cells), 2))
        np.add.at(sums, inverse, points)
        representatives.append(sums / np.bincount(inverse)[:, None])
    if not representatives:
        return np.empty((0, 2))
    points = np.vstack(representatives)
    cells, inverse, support = np.unique(np.floor(points / voxel_size).astype(np.int64),
                                        axis=0, return_inverse=True, return_counts=True)
    sums = np.zeros((len(cells), 2))
    np.add.at(sums, inverse, points)
    return (sums / support[:, None])[support >= minimum_scans]
