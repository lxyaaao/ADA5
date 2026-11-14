"""Offline camera-assisted LiDAR mapping, shared by CLI and ROS 2 batch node."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from .geometry import pose_compose,pose_inverse,pose_between,transform_points,scan_to_points,register_scan
from .occupancy import build_occupancy,save_map,save_pcd
from .graph import nearest_timestamps,optimize_graph,wrap
from .vision import image_features,verify_images


def write_json(path,obj):
    def encode(x):
        if isinstance(x,np.ndarray):return encode(x.tolist())
        if isinstance(x,np.generic):return encode(x.item())
        if isinstance(x,float) and not np.isfinite(x):return None
        if isinstance(x,dict):return {k:encode(v) for k,v in x.items()}
        if isinstance(x,(list,tuple)):return [encode(v) for v in x]
        if isinstance(x,Path):return str(x)
        return x
    Path(path).write_text(json.dumps(encode(obj),indent=2,allow_nan=False),encoding='utf-8')


def _voxel(points,size=.06):
    _,indices=np.unique(np.floor(points/size).astype(np.int64),axis=0,return_index=True)
    return points[np.sort(indices)]


def lidar_odometry(data,output,period=.15,max_seconds=None):
    times=data['times_ns']; relative=(times-times[0])*1e-9
    selected=[];last=-1e10
    for i,t in enumerate(relative):
        if max_seconds is not None and t>max_seconds:break
        if t-last>=period:selected.append(i);last=t
    if len(selected)<2:raise ValueError('Need at least two LiDAR scans')
    def points(i):
        return scan_to_points(data['ranges'][i],float(data['angle_min']),float(data['angle_increment']),
                              max(.12,float(data['range_min'])),min(12.,float(data['range_max'])),voxel_size=.045)
    scans=[];poses=[];statuses=[];prev_delta=np.zeros(3);accepted_ids=[]
    key_ids=[];last_key_t=-1e10;submap_start=0
    for n,idx in enumerate(selected):
        scan=points(idx)
        if len(scan)<40:
            statuses.append({'scan_index':idx,'time_s':float(relative[idx]),'success':False,'reason':'too_few_returns'})
            if accepted_ids and (times[idx]-times[accepted_ids[-1]])>2_000_000_000:
                print(f'Tracking lost at {relative[idx]:.1f}s due to insufficient returns',flush=True)
                break
            continue
        if not poses:
            pose=np.zeros(3);result={'success':True,'rmse':0.,'inlier_ratio':1.,'condition':1.}
        else:
            # Target is a short local submap expressed in the last accepted scan frame.
            inverse=pose_inverse(poses[-1])
            start=max(len(scans)-5,submap_start)
            target=_voxel(np.vstack([transform_points(s,pose_compose(inverse,p)) for s,p in zip(scans[start:],poses[start:])]))
            result=register_scan(scan,target,initial=prev_delta,max_distance=.55,iterations=25)
            delta=np.asarray(result['pose'])
            elapsed=(times[idx]-times[accepted_ids[-1]])*1e-9
            plausible=np.linalg.norm(delta[:2])<max(.20,2.2*elapsed) and abs(delta[2])<max(.25,2.*elapsed)
            good=bool(result['success'] and result['inlier_ratio']>=.48 and result['rmse']<.16 and plausible)
            if not good:
                # Retry without a velocity prediction; no wheel or IMU measurements.
                retry=register_scan(scan,target,initial=np.zeros(3),max_distance=.8,iterations=40)
                d=np.asarray(retry['pose'])
                if retry['success'] and retry['inlier_ratio']>=.55 and retry['rmse']<.13 and np.linalg.norm(d[:2])<max(.20,2.2*elapsed) and abs(d[2])<max(.25,2.*elapsed):
                    result=retry;delta=d;good=True
            if not good:
                # During a turn, older scans may lose common structure even when
                # adjacent scans still overlap. Require a stricter bidirectional
                # adjacent fit, then restart the local submap at that boundary.
                adjacent=register_scan(scan,scans[-1],initial=prev_delta,max_distance=.45,iterations=35)
                d=np.asarray(adjacent['pose'])
                if adjacent['success'] and adjacent['inlier_ratio']>=.6 and adjacent['rmse']<.06 and adjacent['condition']<1000 and np.linalg.norm(d[:2])<max(.20,2.2*elapsed) and abs(d[2])<max(.25,2.*elapsed):
                    reverse=register_scan(scans[-1],scan,initial=pose_inverse(d),max_distance=.45,iterations=25)
                    cycle=pose_compose(d,reverse['pose'])
                    if reverse['success'] and np.linalg.norm(cycle[:2])<.06 and abs(cycle[2])<.03:
                        result=adjacent;delta=d;good=True;submap_start=len(scans)-1
                        result['recovery']='bidirectional_adjacent_scan'
            if not good:
                statuses.append({'scan_index':idx,'time_s':float(relative[idx]),'success':False,'reason':'registration_rejected',
                                 'rmse':float(result['rmse']),'inlier_ratio':float(result['inlier_ratio']),
                                 'icp_reason':result.get('reason'),'condition':float(result.get('condition',0)),
                                 'retry_reason':retry.get('reason'),'retry_condition':float(retry.get('condition',0))})
                prev_delta=np.zeros(3)
                # Fail closed: a prolonged tracking gap cannot be bridged honestly.
                if elapsed>2.0:
                    print(f'Tracking lost at {relative[idx]:.1f}s; map limited to tracked prefix',flush=True)
                    break
                continue
            pose=pose_compose(poses[-1],delta)
            prev_delta=delta
        scans.append(scan);poses.append(pose);accepted_ids.append(idx)
        statuses.append({'scan_index':idx,'time_s':float(relative[idx]),'success':True,'rmse':float(result['rmse']),
                         'inlier_ratio':float(result['inlier_ratio']),'condition':float(result.get('condition',0)),
                         'recovery':result.get('recovery')})
        dist=np.linalg.norm(pose[:2]-poses[key_ids[-1]][:2]) if key_ids else 1e9
        yaw=abs(wrap(pose[2]-poses[key_ids[-1]][2])) if key_ids else 1e9
        if not key_ids or (relative[idx]-last_key_t>=1. and (dist>=.35 or yaw>=.18 or relative[idx]-last_key_t>=3.)):
            key_ids.append(len(poses)-1);last_key_t=relative[idx]
        if n%200==0:print(f'LiDAR {n}/{len(selected)} t={relative[idx]:.1f}s pose={np.round(pose,2)}',flush=True)
    if key_ids[-1]!=len(poses)-1:key_ids.append(len(poses)-1)
    np.savez_compressed(output/'odometry.npz',poses=np.array(poses),scan_indices=np.array(accepted_ids),key_ids=np.array(key_ids))
    write_json(output/'tracking.json',statuses)
    return np.array(poses),np.array(accepted_ids),np.array(key_ids),scans,statuses


def camera_loops(cache,times,poses,ids,scans,output):
    rows=list(csv.DictReader((cache/'images.csv').open(encoding='utf-8')))
    rows.sort(key=lambda r:int(r['timestamp_ns']))
    image_times=np.array([int(r['timestamp_ns']) for r in rows],dtype=np.int64)
    associated,delta,valid=nearest_timestamps(image_times,times[ids],50_000_000)
    features=[]
    for k,(im,ok) in enumerate(zip(associated,valid)):
        if not ok:features.append(None);continue
        p=Path(rows[im]['path'])
        if not p.exists():
            p=cache/p
            if not p.exists():p=cache/'images'/Path(rows[im]['path']).name
        try:
            features.append(image_features(p))
        except ValueError:
            features.append(None);valid[k]=False
    write_json(output/'image_associations.json',[
        {'keyframe':k,'scan_timestamp_ns':int(times[ids[k]]),'image_timestamp_ns':int(image_times[j]),'delta_ms':float(delta[k]/1e6),
         'valid':bool(valid[k]),'image_path':None if features[k] is None else features[k]['path']} for k,j in enumerate(associated)])
    globals=np.array([f['global'] if f else np.zeros(768) for f in features])
    similarity=globals@globals.T
    edges=[];events=[];last_loop=-100
    for j in range(len(ids)):
        if features[j] is None or j-last_loop<3:continue
        dt=(times[ids[j]]-times[ids])*1e-9
        distance=np.linalg.norm(poses[:,:2]-poses[j,:2],axis=1)
        yaw=np.abs(wrap(poses[:,2]-poses[j,2]))
        candidates=np.flatnonzero((dt>35)&(distance<8.)&(yaw<1.2)&valid)
        # Repeated stationary frames are not a return after exploration.
        candidates=np.array([i for i in candidates if np.max(np.linalg.norm(poses[i:j+1,:2]-poses[i,:2],axis=1))>3.],dtype=int)
        if not len(candidates):continue
        ranked=candidates[np.argsort(similarity[j,candidates])[-5:][::-1]]
        for i in ranked:
            visual=verify_images(features[int(i)],features[j])
            if not visual['success']:continue
            guess=pose_between(poses[i],poses[j])
            trials=[register_scan(scans[j],scans[i],initial=guess,max_distance=.9,iterations=45)]
            if np.linalg.norm(guess[:2])<3:
                trials.append(register_scan(scans[j],scans[i],initial=np.array([0.,0.,guess[2]]),max_distance=.9,iterations=45))
            result=min(trials,key=lambda r:r['rmse']+(.5 if not r['success'] else 0))
            measured=np.asarray(result['pose'])
            correction=pose_between(guess,measured)
            accepted=bool(result['success'] and result['inlier_ratio']>=.68 and result['rmse']<.075
                          and result.get('condition',1e10)<2000 and np.linalg.norm(correction[:2])<3.
                          and abs(correction[2])<.45 and np.linalg.norm(measured[:2])<3.)
            # Reciprocal registration guards local ICP minima / directional overlap.
            inverse_error=None
            if accepted:
                reverse=register_scan(scans[i],scans[j],initial=pose_inverse(measured),max_distance=.4,iterations=25)
                cycle=pose_compose(measured,reverse['pose']);inverse_error=float(np.linalg.norm(cycle[:2]))
                accepted=bool(reverse['success'] and inverse_error<.08 and abs(cycle[2])<.04)
            event={'i':int(i),'j':j,'time_i_s':float((times[ids[i]]-times[0])*1e-9),'time_j_s':float((times[ids[j]]-times[0])*1e-9),
                   'accepted':accepted,'image_i':features[i]['path'],'image_j':features[j]['path'],'visual':visual,
                   'lidar_rmse':float(result['rmse']),'lidar_inlier_ratio':float(result['inlier_ratio']),
                   'lidar_condition':float(result.get('condition',0)),'pose':measured.tolist(),'odometry_relative_pose':guess.tolist(),
                   'inverse_error_m':inverse_error}
            events.append(event)
            if accepted:
                edges.append({'i':int(i),'j':j,'pose':measured.tolist(),'loop':True})
                last_loop=j
                print(f'Visual loop {i}->{j}, ORB {visual["inliers"]}, ICP {result["rmse"]:.3f}m',flush=True)
                break
    write_json(output/'loop_events.json',events)
    return edges,events,{'keyframes':len(ids),'associated_within_50ms':int(valid.sum()),'association_p95_ms':float(np.percentile(delta/1e6,95)),
                        'visually_verified_candidates':len(events),'accepted_camera_lidar_loops':len(edges)}


def _vehicle_offset(calibration):
    # Audit schema is validated after extraction. Missing transforms never become fabricated offsets.
    if not calibration.exists():return None
    data=json.loads(calibration.read_text(encoding='utf-8'))
    transforms=data.get('static_transforms',data.get('tf_static',[]))
    if isinstance(transforms,dict):transforms=list(transforms.values())
    for t in transforms:
        parent=t.get('parent',t.get('parent_frame',t.get('frame_id','')))
        child=t.get('child',t.get('child_frame',t.get('child_frame_id','')))
        if parent in ('base_link','base_footprint') and ('laser' in child or 'lidar' in child):
            if 'T_parent_child' in t:
                matrix=np.array(t['T_parent_child'])
                # Only a level scanner supports this planar vehicle-pose conversion.
                if abs(matrix[2,0])+abs(matrix[2,1])>0.02:
                    return None
                return np.array([matrix[0,3],matrix[1,3],np.arctan2(matrix[1,0],matrix[0,0])])
            xyz=t.get('translation',[]);q=t.get('rotation',[])
            if isinstance(xyz,dict):xyz=[xyz[k] for k in ('x','y','z')]
            if isinstance(q,dict):q=[q[k] for k in ('x','y','z','w')]
            if len(xyz)==3 and len(q)==4:
                x,y,z,w=q;yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
                return np.array([xyz[0],xyz[1],yaw])
    return None


def run_mapping(cache_dir='outputs/cache',output_dir='outputs/result',max_seconds=None,resume=False,visual_landmarks=True,lidar_revisits=False):
    cache=Path(cache_dir);output=Path(output_dir);output.mkdir(parents=True,exist_ok=True)
    started=time.time()
    # Materialize once: NpzFile decompresses a member on every __getitem__.
    with np.load(cache/'scans.npz') as archive:
        data={key:archive[key] for key in archive.files}
    times=data['times_ns']
    source_dir=Path(__file__).resolve().parent
    fingerprints={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source_dir.glob('*.py')}
    write_json(output/'run_config.json',{'estimator_inputs':['/scan/raw','/camera/camera/color/image_raw/compressed','/tf_static'],
        'calibration_audited':'/camera/camera/color/camera_info','scan_period_s':.15,'range_min_m':.12,'range_max_m':12.,
        'scan_voxel_m':.045,'submap_scans':5,'keyframe_min_interval_s':1.,'keyframe_distance_m':.35,
        'keyframe_angle_rad':.18,'keyframe_max_interval_s':3.,'association_window_ms':50,
        'loop_min_elapsed_s':35,'loop_min_excursion_m':3.,'loop_max_odometry_distance_m':8.,'pcd_voxel_m':.03,
        'occupancy_resolution_m':.05,'max_seconds':max_seconds,'resume':resume,'visual_landmarks':visual_landmarks,
        'lidar_revisits':lidar_revisits,'source_sha256':fingerprints})
    if resume and (output/'odometry.npz').exists():
        saved=np.load(output/'odometry.npz');poses=saved['poses'];ids=saved['scan_indices'];keys=saved['key_ids']
        scans=[scan_to_points(data['ranges'][i],float(data['angle_min']),float(data['angle_increment']),max(.12,float(data['range_min'])),min(12.,float(data['range_max'])),voxel_size=.045) for i in ids]
        statuses=json.loads((output/'tracking.json').read_text())
    else:poses,ids,keys,scans,statuses=lidar_odometry(data,output,max_seconds=max_seconds)
    kposes=poses[keys];kids=ids[keys];kscans=[scans[k] for k in keys]
    edges=[{'i':i,'j':i+1,'pose':pose_between(kposes[i],kposes[i+1]).tolist(),'loop':False} for i in range(len(keys)-1)]
    loops,events,metrics=camera_loops(cache,times,kposes,kids,kscans,output)
    geometric_loops=[];geometric_events=[];geometric_parameters=None;geometric_training=[]
    if lidar_revisits:
        from dataclasses import replace
        from .lidar_revisits import RevisitParameters,propose_revisits
        parameters=replace(RevisitParameters(),min_geometric_coverage=.50,min_mean_geometric_coverage=.65,
                           max_odometry_correction_m=2.0,min_converged_initializations=3)
        geometric_loops,geometric_events,geometric_training,geometric_parameters=propose_revisits(scans,poses,keys,times[ids],parameters)
    write_json(output/'lidar_revisit_events.json',{'events':geometric_events,'parameters':geometric_parameters,
               'training_raw_scan_indices':ids[geometric_training].tolist() if geometric_training else []})
    corrected,graph_stats=optimize_graph(kposes,edges+loops+geometric_loops)
    write_json(output/'pose_graph.json',{'edges':edges+loops+geometric_loops,'optimization':graph_stats})
    # Interpolate map-to-odometry correction across keyframes; preserve local motion between them.
    corrections=np.array([pose_compose(c,pose_inverse(p)) for c,p in zip(corrected,kposes)])
    corrections[:,2]=np.unwrap(corrections[:,2]);positions=(times[ids]-times[ids[0]])*1e-9
    dense_corr=np.column_stack([np.interp(positions,positions[keys],corrections[:,k]) for k in range(3)])
    final=np.array([pose_compose(c,p) for c,p in zip(dense_corr,poses)])
    offset=_vehicle_offset(cache.parent/'audit'/'calibration.json')
    vehicle=np.array([pose_compose(p,pose_inverse(offset)) for p in final]) if offset is not None else final
    with (output/'trajectory.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f);writer.writerow(['timestamp_ns','timestamp','x','y','yaw','lidar_x','lidar_y','lidar_yaw','pose_frame'])
        for t,p,l in zip(times[ids],vehicle,final):writer.writerow([int(t),f'{int(t)//10**9}.{int(t)%10**9:09d}',*p,*l,'base_link' if offset is not None else str(data['frame'])])
    with (output/'lidar_only_trajectory.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f);writer.writerow(['timestamp_ns','x','y','yaw']);writer.writerows([[int(t),*p] for t,p in zip(times[ids],poses)])
    np.savez_compressed(output/'trajectory_arrays.npz',times_ns=times[ids],lidar_only=poses,corrected=final,key_ids=keys)
    # Keep at most one map insertion per 0.4s unless vehicle moves significantly.
    map_ids=[0]
    for i in range(1,len(scans)):
        prev=map_ids[-1]
        if (times[ids[i]]-times[ids[prev]])>=400_000_000 and (np.linalg.norm(final[i,:2]-final[prev,:2])>.06 or abs(wrap(final[i,2]-final[prev,2]))>.04):map_ids.append(i)
    map_scans=[scans[i] for i in map_ids];map_poses=final[map_ids]
    lidar_pcd=save_pcd(map_scans,map_poses,output/'lidar_scan_map.pcd',voxel_size=.03,z=0.,max_range=12.)
    from .ros_node import load_ascii_pcd
    from .visual_landmarks import build_visual_landmarks,save_xyz_pcd
    lidar_points=load_ascii_pcd(output/'lidar_scan_map.pcd')
    visual_points=np.empty((0,3));visual_stats={'enabled':False,'accepted_landmarks':0}
    if visual_landmarks:
        visual_points,visual_stats=build_visual_landmarks(cache,times[ids],final,cache.parent/'audit'/'calibration.json',output)
    combined=np.vstack([lidar_points,visual_points])
    save_xyz_pcd(combined,output/'map.pcd')
    mixed=bool(len(visual_points))
    geometry='lidar_scan_plane_plus_triangulated_visual_landmarks' if mixed else 'lidar_scan_plane'
    z_definition=('LiDAR points use z=0 for the scan plane; visual points have triangulated height in that same local frame. '
                  'This is sparse visual structure, not a dense 3D reconstruction.') if mixed else 'All z=0: projection of observed LiDAR scan plane; no height reconstruction'
    occupancy=build_occupancy(map_scans,map_poses,resolution=.05,max_range=12.)
    save_map(occupancy,output)
    write_json(output/'map_metadata.json',{'file':'map.pcd','format':'PCD 0.7 ASCII','fields':['x','y','z'],'unit':'metre',
        'frame_id':'map','origin_timestamp_ns':int(times[ids[0]]),'origin':'First accepted LiDAR scan pose',
        'axes':'Initial LiDAR x forward, y left, z up','z_definition':z_definition,'geometry':geometry,
        'points':len(combined),'lidar_point_count':len(lidar_points),'visual_point_count':len(visual_points),
        'point_order':'LiDAR scan points first, followed by triangulated camera landmarks',
        'components':['lidar_scan_map.pcd','visual_landmarks.pcd'] if mixed else ['lidar_scan_map.pcd'],
        'voxel_size_m':.03,'visual_voxel_size_m':.05,'localization_file':'lidar_scan_map.pcd',
        'localization_note':'Use only the LiDAR scan plane for 2D scan localization; do not project visual XYZ into this plane.',
        'source_scan_frame':str(data['frame']),
        'base_to_lidar_se2':None if offset is None else offset.tolist(),'physical_reference_validation':'not available'})
    success=[s for s in statuses if s['success']]
    def loop_error(p):
        return [float(np.linalg.norm(pose_between(p[e['i']],p[e['j']])[:2]-np.asarray(e['pose'])[:2])) for e in loops]
    metrics.update({'input_scan_count':len(times),'tracked_scans':len(ids),'map_insertions':len(map_ids),'pcd_points':len(combined),
                    'geometry':geometry,'lidar_point_count':len(lidar_points),'visual_point_count':len(visual_points),
                    'visual_landmarks':visual_stats,'camera_mapping_contribution':mixed,
                    'bag_scan_duration_s':float((times[-1]-times[0])*1e-9),'tracked_duration_s':float((times[ids[-1]]-times[ids[0]])*1e-9),
                    'rejected_registrations':len(statuses)-len(success),'lidar_icp_rmse_median_m':float(np.median([s['rmse'] for s in success[1:]])),
                    'adjacent_scan_recoveries':sum(s.get('recovery')=='bidirectional_adjacent_scan' for s in success),
                    'camera_constraints_applied':bool(loops and graph_stats['success']),
                    'accepted_lidar_revisits':len(geometric_loops),
                    'lidar_revisit_constraints_applied':bool(geometric_loops and graph_stats['success']),
                    'loop_translation_residual_before_m':loop_error(kposes),'loop_translation_residual_after_m':loop_error(corrected),
                    'lidar_icp_rmse_p95_m':float(np.percentile([s['rmse'] for s in success[1:]],95)),
                    'estimated_path_length_m':float(np.linalg.norm(np.diff(final[:,:2],axis=0),axis=1).sum()),
                    'endpoint_separation_before_m':float(np.linalg.norm(poses[-1,:2]-poses[0,:2])),
                    'endpoint_separation_after_m':float(np.linalg.norm(final[-1,:2]-final[0,:2])),
                    'max_graph_correction_m':float(np.linalg.norm(final[:,:2]-poses[:,:2],axis=1).max()),
                    'max_camera_graph_correction_m':0.0 if not loops else (None if geometric_loops else float(np.linalg.norm(final[:,:2]-poses[:,:2],axis=1).max())),
                    'vehicle_transform_available':offset is not None,'base_to_lidar_se2':None if offset is None else offset.tolist(),
                    'map_origin_description':'First accepted LiDAR pose; x forward, y left in initial scan frame; metres; z=0 scan plane projection',
                    'absolute_accuracy':'UNVALIDATED: no independent physical reference measurements supplied',
                    'uses_imu':False,'uses_recorded_odometry':False,'uses_recorded_map':False,'graph':graph_stats,'elapsed_s':time.time()-started})
    write_json(output/'metrics.json',metrics)
    from .figures import make_figures
    make_figures(output,occupancy,poses,final,events,times[ids],visual_points=visual_points,lidar_point_count=len(lidar_points),lidar_loops=bool(geometric_loops))
    print(json.dumps(metrics,indent=2),flush=True)
    return metrics


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache',default='outputs/cache');parser.add_argument('--output',default='outputs/result')
    parser.add_argument('--max-seconds',type=float);parser.add_argument('--resume',action='store_true')
    parser.add_argument('--no-visual-landmarks',action='store_true',help='Disable metric multi-view camera landmarks for ablation')
    parser.add_argument('--lidar-revisits',action='store_true',help='Enable the partial-overlap LiDAR revisit configuration evaluated on fin301')
    args=parser.parse_args();run_mapping(args.cache,args.output,args.max_seconds,args.resume,not args.no_visual_landmarks,args.lidar_revisits)


if __name__=='__main__':main()
