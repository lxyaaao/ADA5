"""Occupancy grid from posed scans (Lecture 6: free space along rays, occupied at returns,
unknown where no ray passed), saved in the ROS map_server format (PGM + YAML)."""
import os

import numpy as np

from .geometry import transform

P_OCC, P_FREE = 0.65, 0.196   # thresholds written to the YAML (map_server defaults)


class OccupancyGrid:
    def __init__(self, resolution=0.05, l_occ=0.85, l_free=-0.4, l_min=-4.0, l_max=4.0):
        self.res, self.l_occ, self.l_free, self.l_min, self.l_max = resolution, l_occ, l_free, l_min, l_max
        self.origin, self.n_occ, self.n_free = None, None, None

    def build(self, poses, scans, laser_xy, margin=1.0, max_free=12.0):
        """poses: base_link poses in the map frame; scans: matching N_i x 2 endpoints in base_link.

        Beams without a return carry no evidence (glass and open space stay unknown).
        """
        ends = [transform(p, s) for p, s in zip(poses, scans)]
        allp = np.vstack(ends)
        lo, hi = allp.min(0) - margin, allp.max(0) + margin
        self._allocate(lo, (int(np.ceil((hi[1] - lo[1]) / self.res)), int(np.ceil((hi[0] - lo[0]) / self.res))))
        for p, e in zip(poses, ends):
            self._integrate(transform(p, np.asarray(laser_xy, float)[None])[0], e, max_free)
        return self

    def allocate_square(self, centre, size_m):
        """Fixed square grid for incremental (live) use."""
        n = int(np.ceil(size_m / self.res))
        self._allocate(np.asarray(centre, float) - size_m / 2, (n, n))
        return self

    def integrate(self, pose, pts_base, laser_xy, max_free=12.0):
        """Add one scan given the base_link pose and its endpoints in base_link."""
        self._integrate(transform(pose, np.asarray(laser_xy, float)[None])[0],
                        transform(pose, pts_base), max_free)

    def _allocate(self, origin, shape):
        self.origin, self.shape = np.asarray(origin, float), shape
        self.n_occ = np.zeros(shape, np.int32)
        self.n_free = np.zeros(shape, np.int32)

    def _integrate(self, o, e, max_free):
        step = 0.5 * self.res
        v = e - o
        d = np.linalg.norm(v, axis=1)
        keep = d > self.res
        v, d, e = v[keep], d[keep], e[keep]
        if not len(d):
            return
        n = ((np.minimum(d, max_free) - self.res) / step).astype(int)
        s = np.arange(max(n.max(), 1))
        frac = (s[None, :] * step) / d[:, None]
        mask = s[None, :] < n[:, None]
        free = self._index((o[0] + v[:, 0:1] * frac)[mask], (o[1] + v[:, 1:2] * frac)[mask])
        # a[idx] += 1 adds once per distinct index, so each scan counts a cell at most once
        flat_free, flat_occ = self.n_free.reshape(-1), self.n_occ.reshape(-1)
        flat_free[free[free >= 0]] += 1
        occ = self._index(e[:, 0], e[:, 1])
        flat_occ[occ[occ >= 0]] += 1

    def _index(self, x, y):
        ny, nx = self.shape
        ix = np.floor((x - self.origin[0]) / self.res).astype(int)
        iy = np.floor((y - self.origin[1]) / self.res).astype(int)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        return np.where(ok, iy * nx + ix, -1)

    def log_odds(self):
        return np.clip(self.n_occ * self.l_occ + self.n_free * self.l_free, self.l_min, self.l_max)

    def states(self):
        """1 occupied, 0 free, -1 unknown."""
        L = self.log_odds()
        st = np.full(L.shape, -1, np.int8)
        seen = (self.n_occ + self.n_free) > 0
        st[seen & (L > np.log(P_OCC / (1 - P_OCC)))] = 1
        st[seen & (L < np.log(P_FREE / (1 - P_FREE)))] = 0
        return st

    def save(self, folder, name):
        """Write <name>.pgm and <name>.yaml (trinary, origin at the lower-left cell)."""
        os.makedirs(folder, exist_ok=True)
        st = self.states()
        img = np.full(st.shape, 205, np.uint8)
        img[st == 0] = 254
        img[st == 1] = 0
        img = img[::-1]                                  # first image row = largest y
        with open(os.path.join(folder, f'{name}.pgm'), 'wb') as f:
            f.write(f'P5\n# CISC647 Project 2.2 Ada5 map, {self.res} m/cell\n'
                    f'{img.shape[1]} {img.shape[0]}\n255\n'.encode())
            f.write(img.tobytes())
        with open(os.path.join(folder, f'{name}.yaml'), 'w') as f:
            f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {self.res}\n'
                    f'origin: [{self.origin[0]:.3f}, {self.origin[1]:.3f}, 0.0]\n'
                    f'negate: 0\noccupied_thresh: {P_OCC}\nfree_thresh: {P_FREE}\n')
        return st

    def extent(self):
        ny, nx = self.shape
        return [self.origin[0], self.origin[0] + nx * self.res, self.origin[1], self.origin[1] + ny * self.res]
