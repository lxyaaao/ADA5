"""Export vehicle poses and reproducible sensor-consistency diagnostics, not ground truth."""
import csv
import json
import os
import sqlite3
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from context import WORK, BAG
from scan_geometry import relative, rot, wrap, points, icp

out = Path(os.environ['MAP_RESULTS_DIR'])
out.mkdir(parents=True, exist_ok=True)
p = np.loadtxt(WORK/'backend/optimized_poses.csv', delimiter=',', skiprows=1, ndmin=2)
if len(p) < 2 or np.any(np.diff(p[:,0]) <= 0):
    raise RuntimeError('Need ordered, distinct optimized trajectory timestamps')
cal = json.loads((WORK/'camera/calibration.json').read_text())
# Each recorded TF is parent_from_child. Traverse the static graph from base_link.
graph = {}
for tf in cal['static_tf']:
    m = np.eye(4)
    m[:3,:3] = Rotation.from_quat(tf['quaternion']).as_matrix()
    m[:3,3] = tf['translation']
    graph.setdefault(tf['parent'], []).append((tf['child'],m))
    graph.setdefault(tf['child'], []).append((tf['parent'],np.linalg.inv(m)))
queue = [('base_link',np.eye(4))]; seen = set(); base_from_laser = None
while queue:
    frame,m = queue.pop(0)
    if frame in seen: continue
    seen.add(frame)
    if frame == 'laser': base_from_laser = m; break
    queue.extend((child,m@step) for child,step in graph.get(frame,[]) if child not in seen)
if base_from_laser is None: raise RuntimeError('Missing static base_link-to-laser transform')
laser_from_base = np.linalg.inv(base_from_laser)
vehicle = []
for t,x,y,a in p:
    m = np.eye(4);m[:3,:3] = Rotation.from_euler('z',a).as_matrix();m[:2,3] = [x,y]
    b = m@laser_from_base
    vehicle.append([t,b[0,3],b[1,3],np.arctan2(b[1,0],b[0,0])])
vehicle = np.asarray(vehicle)
np.savetxt(out/'vehicle_poses.csv',vehicle,delimiter=',',header='timestamp_s,x_m,y_m,yaw_rad',comments='')

def pose(t):
    if not p[0,0] <= t <= p[-1,0]: raise ValueError('Reference time outside optimized trajectory')
    return np.array([np.interp(t,p[:,0],p[:,1]),np.interp(t,p[:,0],p[:,2]),np.interp(t,p[:,0],np.unwrap(p[:,3]))])

revisits = []
for row in json.loads((WORK/'camera/revisits.json').read_text()):
    predicted = relative(pose(row['scan_time_a']),pose(row['scan_time_b']))
    error = relative(np.asarray(row['lidar_relative_pose']),predicted)
    revisits.append(dict(timestamp_a_s=row['scan_time_a'],timestamp_b_s=row['scan_time_b'],
        used_for_optimization=row['used_for_optimization'],translation_disagreement_m=float(np.linalg.norm(error[:2])),
        heading_disagreement_rad=float(abs(error[2])),registration_residual_m=row['scan_rmse_m'],
        overlap=row['scan_overlap']))

# Audit approximately one scan each second; these are warnings, not definitive tracking failures.
checks = []; first_scan = None; last_sample = -np.inf; previous = None; scan_count = 0
with sqlite3.connect('file:'+str(BAG)+'?mode=ro',uri=True) as conn:
    topic = conn.execute("select id from topics where name='/scan/raw'").fetchone()[0]
    for blob, in conn.execute('select data from messages where topic_id=? order by timestamp',(topic,)):
        msg = deserialize_message(blob,LaserScan); t = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        scan_count += 1
        if first_scan is None: first_scan = t
        if t-last_sample < 1 or not p[0,0] <= t <= p[-1,0]: continue
        last_sample = t
        ranges = np.asarray(msg.ranges);cloud = points(ranges,msg.angle_min+np.arange(len(ranges))*msg.angle_increment)
        valid = np.isfinite(ranges)&(ranges>=max(.22,msg.range_min))&(ranges<=min(12,msg.range_max))
        reasons = []
        if valid.mean()<.25 or len(cloud)<40: reasons.append('sparse_scan')
        item = dict(timestamp_s=t,x_m=float(pose(t)[0]),y_m=float(pose(t)[1]),valid_range_fraction=float(valid.mean()),filtered_points=len(cloud))
        if previous:
            pt,pc = previous; dt=t-pt; guess=relative(pose(pt),pose(t))
            fit,rmse,overlap=icp(cloud,pc,guess,.30,25)
            correction=relative(guess,fit)
            item.update(sample_gap_s=dt,registration_residual_m=rmse,overlap=overlap,
                correction_m=float(np.linalg.norm(correction[:2])),correction_rad=float(abs(correction[2])))
            if dt>2: reasons.append('scan_sample_gap')
            if overlap<.45 or rmse>.05: reasons.append('weak_scan_alignment')
            if np.linalg.norm(correction[:2])>.20 or abs(correction[2])>.15: reasons.append('trajectory_scan_disagreement')
            if np.linalg.norm(guess[:2])/dt>3 or abs(guess[2])/dt>3: reasons.append('pose_jump')
        item['warnings']=reasons;checks.append(item);previous=(t,cloud)

reference = dict(status='not_provided',absolute_accuracy_established=False,
    reason='No independently measured distances/checkpoints with uncertainty are supplied in the recordings.')
reference_path=os.environ.get('MAP_REFERENCE_MEASUREMENTS')
if reference_path:
    measurements=json.loads(Path(reference_path).read_text())
    results=[]
    for r in measurements:
        if not r.get('source') or r['uncertainty_m']<=0 or r['measured_distance_m']<0:
            raise ValueError('Each independent reference needs source, nonnegative distance and positive uncertainty_m')
        # Reference describes Euclidean separation of base_link positions at two timestamps.
        ts=[float(r[k]) for k in ['timestamp_a_s','timestamp_b_s']]
        for t in ts: pose(t)  # Reject extrapolation.
        xy=[np.array([np.interp(t,vehicle[:,0],vehicle[:,j]) for j in [1,2]]) for t in ts]
        estimate=float(np.linalg.norm(xy[1]-xy[0]));error=estimate-r['measured_distance_m']
        results.append(dict(**r,estimated_distance_m=estimate,error_m=error,
            within_stated_reference_uncertainty=abs(error)<=r['uncertainty_m']))
    reference=dict(status='evaluated' if results else 'empty',absolute_accuracy_established=False,checks=results,
        limitation='User-supplied physical references; uncertainty is the stated reference bound, not SLAM covariance or global accuracy.')

dt=np.diff(p[:,0]);warnings=[c for c in checks if c['warnings']]
summary=dict(frames=dict(map='map',vehicle='base_link',tracking='laser',units='metres and radians',
    origin='Cartographer first laser initialization; x/y axes inherited from first laser pose',base_from_laser=base_from_laser.tolist()),
    vehicle_pose_count=len(vehicle),raw_scan_count=scan_count,
    initialization=dict(first_scan_timestamp_s=first_scan,first_pose_timestamp_s=float(p[0,0]),
        unestimated_initial_duration_s=max(0.,float(p[0,0]-first_scan)),
        note='Before first optimized pose is unestimated; no unsupported fixed warm-up duration is assumed.'),
    trajectory_gaps=[dict(start_s=float(p[i,0]),end_s=float(p[i+1,0])) for i in np.flatnonzero(dt>max(2.,5*float(np.median(dt))))],
    reliability=dict(sample_period_s=1,sampled_scans=len(checks),warning_samples=len(warnings),
        warning_fraction=len(warnings)/max(1,len(checks)),checks=checks,
        limitation='Heuristic scan-plane consistency audit; missed failures, glass, occlusion and doubled walls remain possible. This does not certify map correctness.'),
    revisits=dict(checks=revisits,limitation='Camera-gated scan registration checks reuse sensor data; held-out added constraints are not independent ground truth. Registration residual is not reference uncertainty.'),
    camera=dict(associated_frames=cal['synchronized_camera_frames'],decode_failures=cal['camera_decode_failures']),
    physical_reference=reference)
(out/'quality_checks.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
print(f'Exported {len(vehicle)} base_link poses; {len(warnings)}/{len(checks)} reliability samples flagged; physical reference: {reference["status"]}')
