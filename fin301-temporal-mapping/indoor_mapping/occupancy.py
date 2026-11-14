"""Metric planar maps from LiDAR return endpoints and local SE(2) poses.

The PCD export is a 2D LiDAR map embedded in a constant-z plane, not a 3D
reconstruction. Occupancy rays contain only supplied finite return endpoints;
missing or infinite LaserScan beams provide no free-space evidence here.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image
import yaml

from .geometry import transform_points, voxel_downsample


def _validated_inputs(scans, poses, max_range):
    poses = np.asarray(poses, dtype=float)
    if poses.shape != (len(scans), 3) or not np.all(np.isfinite(poses)):
        raise ValueError('poses must be finite with shape (number of scans, 3)')
    if not np.isfinite(max_range) or max_range <= 0:
        raise ValueError('max_range must be finite and positive')
    return poses


def _valid_scan(scan):
    scan = np.asarray(scan, dtype=float)
    if scan.size == 0:
        return np.empty((0, 2)), np.empty(0)
    if scan.ndim != 2 or scan.shape[1] != 2:
        raise ValueError('each scan must have shape (N, 2)')
    norms = np.linalg.norm(scan, axis=1)
    valid = np.all(np.isfinite(scan), axis=1) & (norms > 0)
    return scan[valid], norms[valid]


def build_occupancy(scans, poses, resolution=0.05, max_range=12.0):
    """Raytrace valid supplied returns into a probabilistic occupancy grid.

    Returns a dict with bottom-to-top ``grid`` uint8 image intensities,
    ``occupancy`` int8 probabilities (0..100; -1 unknown), ``log_odds``,
    ``observed``, lower-left metric ``origin=[x,y,0]``, and ``resolution``.
    Returns at or beyond max_range only clear their clipped ray; they do not
    create an occupied endpoint. Invalid/no-return beams are ignored.

    Each scan contributes at most one occupied/free observation to a cell,
    preventing angular oversampling from multiplying confidence. Endpoints
    override free rays within one scan. Padding retains unknown border cells.
    """
    poses = _validated_inputs(scans, poses, max_range)
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError('resolution must be finite and positive')
    transformed = []
    bounds = [poses[:, :2]] if len(poses) else [np.zeros((1, 2))]
    for scan, pose in zip(scans, poses):
        local, distances = _valid_scan(scan)
        clipped = local * np.minimum(1.0, max_range / np.maximum(distances, 1e-12))[:, None]
        endpoints = transform_points(clipped, pose)
        hits = distances < max_range - 1e-6
        transformed.append((endpoints, hits))
        if len(endpoints):
            bounds.append(np.stack([endpoints.min(axis=0), endpoints.max(axis=0)]))
    bounds = np.vstack(bounds)
    padding = max(0.5, 2 * resolution)
    origin = np.floor((bounds.min(axis=0) - padding) / resolution) * resolution
    upper = np.ceil((bounds.max(axis=0) + padding) / resolution) * resolution
    width, height = np.ceil((upper - origin) / resolution).astype(int) + 1
    if int(width) * int(height) > 100_000_000:
        raise ValueError('map exceeds 100 million cells; check poses or increase resolution')
    log_odds = np.zeros((height, width), dtype=np.float32)
    observed = np.zeros((height, width), dtype=bool)
    flat_log, flat_seen = log_odds.ravel(), observed.ravel()
    for (endpoints, hits), pose in zip(transformed, poses):
        if not len(endpoints):
            continue
        start = np.floor((pose[:2] - origin) / resolution + 1e-9).astype(int)
        ends = np.floor((endpoints - origin) / resolution + 1e-9).astype(int)
        hit_ids = np.unique(ends[hits, 1] * width + ends[hits, 0])
        free_parts = []
        # Raster DDA in bounded batches keeps temporary ray arrays small.
        for offset in range(0, len(ends), 256):
            batch = ends[offset:offset + 256]
            batch_hits = hits[offset:offset + 256]
            difference = batch - start
            steps = np.max(np.abs(difference), axis=1)
            time = np.arange(int(steps.max()) + 1)[None, :]
            ratio = time / np.maximum(steps[:, None], 1)
            cells = start + np.rint(difference[:, None, :] * ratio[:, :, None]).astype(int)
            valid = time <= (steps - batch_hits.astype(int))[:, None]
            ids = cells[:, :, 1] * width + cells[:, :, 0]
            free_parts.append(ids[valid])
        free_ids = np.setdiff1d(np.unique(np.concatenate(free_parts)), hit_ids, assume_unique=True)
        flat_log[free_ids] -= 0.4
        flat_log[hit_ids] += 0.85
        flat_seen[free_ids] = True
        flat_seen[hit_ids] = True
        touched = np.concatenate([free_ids, hit_ids])
        flat_log[touched] = np.clip(flat_log[touched], -5.0, 5.0)
    probability = 1.0 / (1.0 + np.exp(-log_odds))
    occupancy = np.where(observed, np.rint(100 * probability), -1).astype(np.int8)
    grid = np.where(observed, np.rint(254 * (1.0 - probability)), 205).astype(np.uint8)
    return dict(grid=grid, occupancy=occupancy, log_odds=log_odds, observed=observed,
                origin=[float(origin[0]), float(origin[1]), 0.0], resolution=float(resolution))


def save_map(map_data, directory, basename='map'):
    """Write PGM, PNG, and ROS map-server YAML, flipping rows for image files."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if not basename or Path(basename).name != basename:
        raise ValueError('basename must be a filename without a directory')
    paths = {extension: directory / f'{basename}.{extension}' for extension in ('pgm', 'yaml', 'png')}
    image = Image.fromarray(np.asarray(map_data['grid'], dtype=np.uint8)[::-1])
    image.save(paths['pgm'])
    image.save(paths['png'])
    metadata = dict(image=paths['pgm'].name, mode='trinary', resolution=float(map_data['resolution']),
                    origin=[float(v) for v in map_data['origin']], negate=0,
                    occupied_thresh=0.65, free_thresh=0.196)
    paths['yaml'].write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    return paths


def save_pcd(scans, poses, path, voxel_size=0.03, z=0.0, max_range=12.0):
    """Save ASCII PCD v0.7 metric returns in the local map frame.

    Only finite positive returns strictly below max_range are emitted;
    range-censored endpoints cannot be interpreted as measured obstacles.
    Centroids of XY voxels reduce duplicates. The constant z is a documented
    plane embedding of planar LiDAR data, with no inferred height structure.
    """
    poses = _validated_inputs(scans, poses, max_range)
    if not np.isfinite(z):
        raise ValueError('z must be finite')
    world = []
    for scan, pose in zip(scans, poses):
        local, distances = _valid_scan(scan)
        local = local[distances < max_range - 1e-6]
        if len(local):
            world.append(transform_points(local, pose))
    points = voxel_downsample(np.vstack(world) if world else np.empty((0, 2)), voxel_size)
    xyz = np.column_stack([points, np.full(len(points), z)])
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        '# Planar LiDAR returns in local map frame; z is a constant plane embedding.\n'
        'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\nDATA ascii\n'
    )
    with path.open('w', encoding='ascii', newline='\n') as stream:
        stream.write(header)
        np.savetxt(stream, xyz, fmt='%.6f')
    return dict(path=path, point_count=len(points), voxel_size=float(voxel_size), z=float(z))
