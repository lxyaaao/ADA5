"""LiDAR odometry: scan-to-local-map ICP with a motion prediction and a motion gate.

Without an IMU the prediction is constant velocity. When the camera heading
prior is available it replaces the predicted heading change; if it disagrees
with the constant-velocity heading by more than prior_check_deg, ICP is run
from both predictions and the better-supported result is kept. A camera
outlier once sent the odometry into an unrecoverable loss (fin301, 7:51), so
the prior is never trusted alone, and a strong match (inlier share at least
strong_inlier) is accepted even if it is far from the prediction.
"""
import numpy as np

from .geometry import between, compose, transform, wrap
from .scan_matching import LocalMap, icp, scan_to_base, translation_eigen, voxel_filter, weak_direction


class LidarOdometry:
    def __init__(self, angles, beam_dt, laser_pose, params):
        self.angles, self.beam_dt, self.laser_pose = angles, beam_dt, laser_pose
        self.p = params
        self.pose = np.zeros(3)   # base_link in the odometry frame (= map frame before loop closure)
        self.vel = np.zeros(3)    # body-frame velocity (vx, vy, w)
        self.t = None
        self.keyframes = []       # {'t', 'pose', 'pts' (base_link)}
        self.lmap = None

    def _rebuild_local_map(self):
        kfs = self.keyframes[-self.p['local_map_keyframes']:]
        pts = np.vstack([transform(k['pose'], k['pts']) for k in kfs])
        self.lmap = LocalMap(voxel_filter(pts, self.p['map_voxel']))

    def step(self, t, ranges, yaw_increment=None):
        """Process one scan at time t (s). Returns the pose and match diagnostics."""
        p = self.p
        pts = voxel_filter(scan_to_base(ranges, self.angles, self.beam_dt, self.laser_pose,
                                        p['range_min'], p['range_max'], self.vel), p['scan_voxel'])
        out = {'t': t, 'pts': pts, 'prior_used': yaw_increment is not None}
        if self.t is None:
            self.t = t
            self.keyframes.append({'t': t, 'pose': self.pose.copy(), 'pts': pts})
            self._rebuild_local_map()
            return dict(out, pose=self.pose.copy(), pred=self.pose.copy(), status='init', inlier=1.0,
                        rmse=0.0, iters=0, weak=0.0, keyframe=True, init_used='none', both_tried=False)

        dt = t - self.t
        pred_cv = compose(self.pose, self.vel * dt)
        preds = [('cv', pred_cv)]
        if yaw_increment is not None:
            inc = self.vel * dt
            inc[2] = yaw_increment
            preds = [('camera', compose(self.pose, inc))]
            if abs(wrap(yaw_increment - self.vel[2] * dt)) > np.radians(p['prior_check_deg']):
                preds.append(('cv', pred_cv))
        src = voxel_filter(pts, p['icp_voxel']) if p['icp_voxel'] > p['scan_voxel'] else pts
        # Longer gaps between processed scans (dropped messages in live use) allow larger
        # prediction errors, so the ICP search radius and the motion gate grow with dt.
        scale = max(1.0, dt / p['nominal_dt'])
        best = None
        for name, pr in preds:
            pose_i, q_i = icp(src, self.lmap, pr, p['icp_max_iter'], min(1.5, p['icp_gate_start'] * scale),
                              p['icp_gate_end'], p['icp_huber'], p['icp_inlier_dist'])
            if best is None or (q_i['ok'] and (not best[3]['ok'] or q_i['inlier'] > best[3]['inlier'])):
                best = (name, pr, pose_i, q_i)
        init_used, pred, pose, q = best
        out.update(init_used=init_used, both_tried=len(preds) > 1)
        status, weak_eig = 'tracked', np.nan
        if q['ok']:
            weak_eig = float(translation_eigen(q['H'])[0] / max(len(src), 1))
            weak = weak_direction(q['H'], len(src), p['degenerate_eig'])
            if weak is not None:   # keep the prediction along the unconstrained direction
                d = pose[:2] - pred[:2]
                pose[:2] = pred[:2] + d - weak * (d @ weak)
                status = 'degenerate'
        dev = between(pred, pose)
        near_prediction = (np.hypot(dev[0], dev[1]) <= p['max_jump_m'] * scale
                           and abs(dev[2]) <= np.radians(p['max_jump_deg']) * scale)
        good = (q['ok'] and q['inlier'] >= p['min_inlier']
                and (near_prediction or q['inlier'] >= p['strong_inlier']))
        if not good:
            pose, status = pred, 'lost'
        elif dt > 0:
            self.vel = between(self.pose, pose) / dt
        self.pose, self.t = pose, t
        moved = between(self.keyframes[-1]['pose'], pose)
        keyframe = good and (np.hypot(moved[0], moved[1]) >= p['kf_dist']
                             or abs(moved[2]) >= np.radians(p['kf_deg']))
        if keyframe:
            self.keyframes.append({'t': t, 'pose': pose.copy(), 'pts': pts})
            self._rebuild_local_map()
        return dict(out, pose=pose.copy(), pred=pred, status=status, inlier=q['inlier'], rmse=q['rmse'],
                    iters=q['iters'], weak=weak_eig, keyframe=keyframe)
