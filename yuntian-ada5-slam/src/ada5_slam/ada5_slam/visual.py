"""Camera contribution (Lecture 6 slides 8-18).

1. Heading prior: ORB matches between consecutive frames, essential matrix
   with RANSAC, rotation converted to the vehicle's heading change. It
   replaces the constant-velocity heading prediction of the LiDAR odometry.
2. Place recognition: a bag of binary words over ORB descriptors proposes
   earlier keyframes that look like the current one; an essential matrix
   verifies the geometry. The monocular camera gives rotation and the
   direction of translation only; LiDAR scan matching supplies the metric
   relative pose (see loop_closure.py).
"""
import cv2
import numpy as np

from .geometry import yaw_of

cv2.setNumThreads(1)   # OpenCV's thread pool is several times slower on the course VM


class Camera:
    def __init__(self, K, D, R_base_cam, n_features=1000, ransac_px=1.5, ransac_iters=300):
        self.K, self.D = np.asarray(K, float), np.asarray(D, float)
        self.R_bc = np.asarray(R_base_cam, float)
        self.orb = cv2.ORB_create(nfeatures=n_features, fastThreshold=12)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.ransac_px, self.ransac_iters = ransac_px, ransac_iters

    def features(self, jpeg):
        """Pixel coordinates and ORB descriptors of one JPEG image."""
        g = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
        kp, des = self.orb.detectAndCompute(g, None)
        if des is None or len(kp) < 10:
            return np.zeros((0, 2), np.float32), None
        return np.float32([k.pt for k in kp]), des

    def match(self, d1, d2, ratio=0.75):
        if d1 is None or d2 is None or len(d1) < 2 or len(d2) < 2:
            return np.zeros((0, 2), int)
        good = [(m.queryIdx, m.trainIdx) for p in self.matcher.knnMatch(d1, d2, k=2)
                if len(p) == 2 for m, n in [p] if m.distance < ratio * n.distance]
        return np.array(good, int).reshape(-1, 2)

    def relative_motion(self, f1, f2, min_inliers=25, still_px=0.7):
        """Vehicle heading change from frame 1 to frame 2 (rad), essential-matrix inliers and
        the unit direction of travel in base_link (None when the frames barely moved).

        Returns (None, inliers, None) when the geometry cannot be verified.
        """
        (p1, d1), (p2, d2) = f1, f2
        m = self.match(d1, d2)
        if len(m) < min_inliers:
            return None, len(m), None
        a, b = p1[m[:, 0]], p2[m[:, 1]]
        if np.median(np.linalg.norm(a - b, axis=1)) < still_px:
            return 0.0, len(m), None               # no measurable motion between the frames
        n1 = cv2.undistortPoints(a.reshape(-1, 1, 2), self.K, self.D).reshape(-1, 2)
        n2 = cv2.undistortPoints(b.reshape(-1, 1, 2), self.K, self.D).reshape(-1, 2)
        E, mask = cv2.findEssentialMat(n1, n2, np.eye(3), method=cv2.RANSAC, prob=0.999,
                                       threshold=self.ransac_px / self.K[0, 0], maxIters=self.ransac_iters)
        if E is None or E.shape != (3, 3):
            return None, 0, None
        n_in, R, t, _ = cv2.recoverPose(E, n1, n2, np.eye(3), mask=mask)
        if n_in < min_inliers:
            return None, int(n_in), None
        R_motion = R.T                              # camera 2 orientation in camera 1
        yaw = yaw_of(self.R_bc @ R_motion @ self.R_bc.T)
        direction = self.R_bc @ (-R.T @ t.ravel())  # camera 2 centre seen from camera 1
        return yaw, int(n_in), direction / max(np.linalg.norm(direction), 1e-9)


class YawTrack:
    """Heading changes of consecutive frames chained into a track; a failed pair starts a new segment."""

    def __init__(self, times, yaw_inc, ok):
        self.t = np.asarray(times, float)
        ok = np.asarray(ok, bool).copy()
        ok[0] = True
        inc = np.where(ok, np.nan_to_num(np.asarray(yaw_inc, float)), 0.0)
        inc[0] = 0.0
        self.cum = np.cumsum(inc)
        self.seg = np.cumsum(~ok)

    def increment(self, t1, t2):
        """Heading change between times t1 and t2, or None if a failed pair lies between."""
        k1, k2 = np.searchsorted(self.t, [t1, t2])
        if k1 <= 0 or k2 >= len(self.t) or self.seg[k2] != self.seg[k1 - 1]:
            return None
        return float(np.interp(t2, self.t, self.cum) - np.interp(t1, self.t, self.cum))


class LiveYawTrack:
    """Incremental YawTrack for the ROS node; keeps the last `keep` frames."""

    def __init__(self, keep=200):
        self.keep, self.t, self.cum, self.seg = keep, [], [], []

    def add(self, t, inc):
        """Add a frame at time t; inc is the heading change from the previous frame or None."""
        if not self.t:
            self.t, self.cum, self.seg = [t], [0.0], [0]
            return
        ok = inc is not None
        self.t.append(t)
        self.cum.append(self.cum[-1] + (inc if ok else 0.0))
        self.seg.append(self.seg[-1] + (0 if ok else 1))
        del self.t[:-self.keep], self.cum[:-self.keep], self.seg[:-self.keep]

    def increment(self, t1, t2):
        if len(self.t) < 2:
            return None
        t = np.asarray(self.t)
        k1, k2 = np.searchsorted(t, [t1, t2])
        if k1 <= 0 or k2 >= len(t) or self.seg[k2] != self.seg[k1 - 1]:
            return None
        return float(np.interp(t2, t, self.cum) - np.interp(t1, t, self.cum))


def build_yaw_track(cam, images, times, every=1, log_every=500):
    """Heading change between frames `every` apart (every=2: 5 Hz from the 10 Hz camera).

    Returns the track, the frame indices used, and per used frame the heading
    change from the previous used frame (nan if unverified) and the inlier count.
    """
    idx = np.arange(0, len(images), every)
    yaw, inl = np.full(len(idx), np.nan), np.zeros(len(idx), int)
    prev = cam.features(images[idx[0]])
    for n in range(1, len(idx)):
        cur = cam.features(images[idx[n]])
        y, c, _ = cam.relative_motion(prev, cur)
        yaw[n], inl[n] = (np.nan if y is None else y), c
        prev = cur
        if log_every and n % log_every == 0:
            print(f'  camera heading {n}/{len(idx)}', flush=True)
    return YawTrack(np.asarray(times)[idx], yaw, np.isfinite(yaw)), idx, yaw, inl


class BagOfWords:
    """Binary visual vocabulary (k-majority over ORB bits) with TF-IDF weighting."""

    def __init__(self, n_words=200, sample=15000, iters=8, seed=0):
        self.n_words, self.sample, self.iters, self.seed = n_words, sample, iters, seed
        self.words, self.idf = None, None
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    def fit(self, descriptor_sets):
        """k-majority clustering in Hamming space (assign by Hamming distance, bitwise vote)."""
        rng = np.random.default_rng(self.seed)
        des = np.vstack([d for d in descriptor_sets if d is not None])
        des = des[rng.choice(len(des), min(self.sample, len(des)), replace=False)]
        bits = np.unpackbits(des, axis=1)
        self.words = des[rng.choice(len(des), self.n_words, replace=False)].copy()
        for _ in range(self.iters):
            assign = self._assign(des)
            for k in range(self.n_words):
                members = bits[assign == k]
                if len(members):
                    self.words[k] = np.packbits(members.mean(axis=0) >= 0.5)
        df = np.zeros(self.n_words)
        for d in descriptor_sets:
            df[np.unique(self._assign(d))] += 1
        self.idf = np.log(len(descriptor_sets) / (1.0 + df))
        return self

    def _assign(self, des):
        if des is None or len(des) == 0:
            return np.zeros(0, int)
        return np.array([m[0].trainIdx for m in self.matcher.knnMatch(des, self.words, k=1)], int)

    def vector(self, des):
        v = np.bincount(self._assign(des), minlength=self.n_words).astype(float)
        v = v / max(v.sum(), 1.0) * self.idf
        return v / max(np.linalg.norm(v), 1e-12)
