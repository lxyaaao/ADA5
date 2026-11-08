"""Camera-gated, range-verified place constraints from this recording only."""
import bisect
import json
import sqlite3
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import LaserScan, CameraInfo, CompressedImage
from tf2_msgs.msg import TFMessage
from cartographer_ros_msgs.msg import LandmarkList, LandmarkEntry
import rosbag2_py
from context import BAG, WORK, DATASET
from scan_geometry import wrap, rot, relative, points, icp

cv2.setNumThreads(1)
cv2.setRNGSeed(7)
folder = WORK / 'camera'
folder.mkdir(exist_ok=True)
poses = np.loadtxt(WORK / 'baseline/optimized_poses.csv', delimiter=',', skiprows=1)
poses[:, 3] = np.unwrap(poses[:, 3])
conn = sqlite3.connect('file:' + str(BAG) + '?mode=ro', uri=True)
topics = {n: i for i, n in conn.execute('select id,name from topics')}
def read(name, cls):
    for record_time, blob in conn.execute('select timestamp,data from messages where topic_id=? order by timestamp', (topics[name],)):
        yield record_time, deserialize_message(blob, cls), blob
def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
cal = next(read('/camera/camera/color/camera_info', CameraInfo))[1]
K = np.asarray(cal.k).reshape(3, 3)
D = np.asarray(cal.d)
mounts = []
for _, msg, _ in read('/tf_static', TFMessage):
    for tf in msg.transforms:
        v, q = tf.transform.translation, tf.transform.rotation
        mounts.append(dict(parent=tf.header.frame_id, child=tf.child_frame_id,
                           translation=[v.x,v.y,v.z], quaternion=[q.x,q.y,q.z,q.w]))
laser_mount = next(t for t in mounts if t['child'] == 'laser')
if not np.allclose(laser_mount['quaternion'], [0,0,0,1], atol=1e-6):
    raise RuntimeError('This planar base-pose conversion requires the recorded identity laser rotation')
scans, times, record_times = [], [], []
for record_time, msg, _ in read('/scan/raw', LaserScan):
    times.append(stamp(msg)); record_times.append(record_time); scans.append(msg)
times = np.asarray(times)
orb = cv2.ORB_create(nfeatures=1100)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
frames, previous = [], -1e12
decode_failures = 0
for _, msg, _ in read('/camera/camera/color/image_raw/compressed', CompressedImage):
    t = stamp(msg)
    if t - previous < .9:
        continue
    previous = t
    scan_id = int(np.argmin(abs(times-t)))
    if abs(times[scan_id]-t) > .05 or t < poses[0,0] or t > poses[-1,0]:
        continue
    image = cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_GRAYSCALE)
    if image is None or image.size == 0:
        decode_failures += 1
        continue
    image = cv2.undistort(image,K,D)
    keypoints, descriptors = orb.detectAndCompute(image,None)
    if descriptors is None or len(keypoints) < 50:
        continue
    pose = np.array([np.interp(times[scan_id], poses[:,0], poses[:,j]) for j in [1,2,3]])
    frames.append(dict(t=t, scan=scan_id, pose=pose, keypoints=keypoints, descriptors=descriptors))
print('Timestamp-associated camera frames',len(frames),flush=True)
xy = np.array([f['pose'][:2] for f in frames])
tree = cKDTree(xy)
accepted, attempts = [], 0
clouds = {}
def scan_cloud(index):
    if index not in clouds:
        scan = scans[index]
        angles = scan.angle_min + np.arange(len(scan.ranges))*scan.angle_increment
        clouds[index] = points(np.asarray(scan.ranges),angles)
    return clouds[index]
for b, fb in enumerate(frames):
    candidates = [a for a in tree.query_ball_point(xy[b],1.5) if frames[b]['t']-frames[a]['t'] > 25
                  and abs(wrap(fb['pose'][2]-frames[a]['pose'][2])) < .55]
    returns = []
    for a in candidates:
        route = poses[(poses[:,0]>=frames[a]['t']) & (poses[:,0]<=fb['t']),1:3]
        excursion = float(np.max(np.linalg.norm(route-xy[a],axis=1))) if len(route) else 0.
        # Exclude long stationary sequences: a revisit must follow a real departure.
        if excursion >= max(1.5, float(np.linalg.norm(xy[b]-xy[a]))+.5):
            returns.append((a,excursion))
    excursions = dict(returns)
    candidates = [a for a,_ in returns]
    # Spread nearby candidates in time instead of repeatedly matching one instant.
    candidates = sorted(candidates,key=lambda a:np.linalg.norm(xy[a]-xy[b]))[:32]
    candidates = candidates[::max(1,len(candidates)//8)][:8]
    for a in candidates:
        fa = frames[a]
        if any(abs(fb['t']-r['time_b']) < 2 and abs(fa['t']-r['time_a']) < 2 for r in accepted):
            continue
        pairs = matcher.knnMatch(fa['descriptors'],fb['descriptors'],k=2)
        matches = [m for pair in pairs if len(pair)==2 for m,n in [pair] if m.distance < .72*n.distance]
        if len(matches) < 35:
            continue
        pa = np.float32([fa['keypoints'][m.queryIdx].pt for m in matches])
        pb = np.float32([fb['keypoints'][m.trainIdx].pt for m in matches])
        _, mask = cv2.findHomography(pa,pb,cv2.RANSAC,3)
        count = int(mask.sum()) if mask is not None else 0
        if count < 35 or count/len(matches) < .55:
            continue
        attempts += 1
        guess = relative(fa['pose'],fb['pose'])
        estimate,error,overlap = icp(scan_cloud(fb['scan']),scan_cloud(fa['scan']),guess,.30,35)
        if error > .025 or overlap < .65 or np.linalg.norm(estimate[:2]) > 1.5 or abs(estimate[2]) > .5:
            continue
        delta = relative(guess,estimate)
        if np.linalg.norm(delta[:2]) > .30 or abs(delta[2]) > .12:
            continue
        row = dict(time_a=fa['t'],time_b=fb['t'],scan_a=fa['scan'],scan_b=fb['scan'],
                   scan_time_a=float(times[fa['scan']]),scan_time_b=float(times[fb['scan']]),
                   camera_scan_dt_a=float(abs(fa['t']-times[fa['scan']])),
                   camera_scan_dt_b=float(abs(fb['t']-times[fb['scan']])),
                   departure_excursion_m=excursions[a],visual_matches=len(matches),visual_inliers=count,scan_rmse_m=error,scan_overlap=overlap,
                   lidar_relative_pose=estimate.tolist(),used_for_optimization=len(accepted)%3!=2)
        accepted.append(row)
        print('Accepted camera/range revisit',len(accepted),'inliers',count,flush=True)
        break
    if len(accepted)>=30:
        break
if not accepted:
    raise RuntimeError(f'Only {len(accepted)} camera/range revisits passed; refuse a silent LiDAR-only fallback')
(folder/'revisits.json').write_text(json.dumps(accepted,indent=2))
(folder/'calibration.json').write_text(json.dumps(dict(K=K.tolist(),D=D.tolist(),static_tf=mounts,
    synchronized_camera_frames=len(frames),camera_decode_failures=decode_failures,raw_scans=len(scans),camera_role='Image-verified place identity gates metric scan constraints',
    transform_assumption='Image features are undistorted but not projected into LiDAR; metric loop geometry is registered in the laser frame'),indent=2))
events = []
for index,row in enumerate(accepted):
    if not row['used_for_optimization']:
        continue
    x,y,a = row['lidar_relative_pose']
    inverse_xy = -rot(a).T @ [x,y]
    for scan_id,local in [(row['scan_a'],[0,0,0]),(row['scan_b'],[*inverse_xy,-a])]:
        msg = LandmarkList();msg.header = scans[scan_id].header
        entry = LandmarkEntry();entry.id = f'camera_place_{index:03}'
        entry.tracking_from_landmark_transform.position.x=float(local[0])
        entry.tracking_from_landmark_transform.position.y=float(local[1])
        entry.tracking_from_landmark_transform.orientation.z=float(np.sin(local[2]/2))
        entry.tracking_from_landmark_transform.orientation.w=float(np.cos(local[2]/2))
        entry.translation_weight=100000.;entry.rotation_weight=100000.;msg.landmarks=[entry]
        events.append((record_times[scan_id]+1,serialize_message(msg)))
events.sort(key=lambda event:event[0])
destination = WORK/'camera_scans'
if destination.exists():
    import shutil
    shutil.rmtree(destination)
writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=str(destination),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
for name,kind in [('/scan','sensor_msgs/msg/LaserScan'),('/landmark','cartographer_ros_msgs/msg/LandmarkList')]:
    writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=kind,serialization_format='cdr'))
next_event = 0
for timestamp,blob in conn.execute('select timestamp,data from messages where topic_id=? order by timestamp',(topics['/scan/raw'],)):
    while next_event<len(events) and events[next_event][0]<timestamp:
        t,data=events[next_event];writer.write('/landmark',data,t);next_event+=1
    writer.write('/scan',blob,timestamp)
while next_event<len(events):
    t,data=events[next_event];writer.write('/landmark',data,t);next_event+=1
del writer
conn.close()
print('Camera loop constraints',sum(r['used_for_optimization'] for r in accepted),'held out',sum(not r['used_for_optimization'] for r in accepted),flush=True)
