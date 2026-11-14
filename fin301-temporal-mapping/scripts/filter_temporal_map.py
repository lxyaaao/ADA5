"""Create fixed temporal-consistency PCD subsets from accepted raw scans."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT))
from indoor_mapping.map_filter import read_ascii_pcd, write_pcd_subset
from indoor_mapping.temporal_filter import compute_temporal_evidence


def digest(path):
    with open(path, 'rb') as stream: return hashlib.file_digest(stream, 'sha256').hexdigest()


def plot_evidence(points, evidence, output):
    os.environ.setdefault('MPLCONFIGDIR', str(ROOT/'outputs/.matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    masks = [('Original', np.ones(len(points), bool)), ('Conservative: retain insufficient', ~evidence['conflicted']),
             ('Strict: static support only', evidence['static_supported'])]
    lo, hi = points[:, :2].min(0), points[:, :2].max(0);pad=np.maximum(hi-lo,1)*.03
    fig, axes = plt.subplots(1,3,figsize=(17,7),sharex=True,sharey=True)
    for ax,(label,mask) in zip(axes,masks):
        ax.scatter(points[mask,0],points[mask,1],s=.7,color='#234b60',edgecolors='none',linewidths=0)
        ax.set(xlim=(lo[0]-pad[0],hi[0]+pad[0]),ylim=(lo[1]-pad[1],hi[1]+pad[1]),xlabel='Map x (m)',ylabel='Map y (m)',title=f'{label}\n{mask.sum():,} points')
        ax.set_aspect('equal');ax.grid(alpha=.15)
    fig.suptitle('Temporal hit/free evidence | exact original coordinates; removed does not mean free')
    fig.tight_layout();fig.savefig(output/'temporal_before_after.png',dpi=170);plt.close(fig)
    example_ids=[]
    for label in ('conflicted','static_supported','insufficient'):
        candidates=np.flatnonzero(evidence[label]);score=evidence['free_epochs'][candidates] if label=='conflicted' else evidence['hit_epochs'][candidates]
        example_ids.extend(candidates[np.argsort(-score.astype(float),kind='stable')[:2]].tolist())
    fig,axes=plt.subplots(max(1,len(example_ids)),1,figsize=(13,max(3,1.5*len(example_ids))),sharex=True)
    colors={1:'#168572',2:'#bd493b',4:'#99a5ac'}
    for ax,pid in zip(np.atleast_1d(axes),example_ids):
        raw=evidence['epoch_states'][:,pid];state=np.where(raw&1,1,np.where(raw&2,2,np.where(raw&4,4,0)))
        for code in (1,2,4):
            ids=np.flatnonzero(state==code);ax.scatter(ids*2.+1,np.zeros(len(ids)),marker='|',s=120,color=colors[code],label={1:'Hit epoch',2:'Visible-free epoch',4:'Occluded epoch'}[code])
        label=next(k for k in ('conflicted','static_supported','insufficient') if evidence[k][pid])
        ax.set(ylim=(-.6,.6),yticks=[],title=f'Point {pid} ({points[pid,0]:.2f},{points[pid,1]:.2f}) {label}: hit={evidence["hit_epochs"][pid]}, free={evidence["free_epochs"][pid]}')
        ax.grid(axis='x',alpha=.2)
    np.atleast_1d(axes)[0].legend(ncol=3,fontsize=8,loc='upper right');np.atleast_1d(axes)[-1].set_xlabel('Seconds from first evidence scan (2s bins shown; actual timestamps determine spans)')
    fig.tight_layout();fig.savefig(output/'temporal_support_examples.png',dpi=170);plt.close(fig)
    return example_ids


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache',type=Path,default=Path('outputs/fin301/cache'))
    parser.add_argument('--result',type=Path,default=Path('outputs/fin301/result'))
    parser.add_argument('--protocol',type=Path,default=Path('outputs/fin301/verification/localization_protocol.json'))
    parser.add_argument('--output',type=Path,default=Path('outputs/fin301/temporal_filter'))
    args=parser.parse_args(argv);started=time.monotonic();args.output.mkdir(parents=True,exist_ok=True)
    if args.output.resolve()==args.result.resolve():raise ValueError('Output must not overwrite original map directory')
    sources={'source_pcd':args.result/'lidar_scan_map.pcd','scans':args.cache/'scans.npz',
             'trajectory':args.result/'trajectory_arrays.npz','scan_ids':args.result/'odometry.npz',
             'protocol':args.protocol,'module':ROOT/'indoor_mapping/temporal_filter.py',
             'exact_subset_io':ROOT/'indoor_mapping/map_filter.py','script':Path(__file__)}
    hashes={k:digest(p) for k,p in sources.items()}
    points,header,rows=read_ascii_pcd(sources['source_pcd'])
    with np.load(sources['scans']) as z:data={k:z[k] for k in z.files}
    with np.load(sources['trajectory']) as z:poses=z['corrected'];times=z['times_ns']
    with np.load(sources['scan_ids']) as z:scan_ids=z['scan_indices']
    if not np.array_equal(times,data['times_ns'][scan_ids]):raise ValueError('Corrected poses do not match raw timestamps')
    protocol=json.loads(args.protocol.read_text());query_ids={q['scan_index'] for g in ('queries','crosspass_queries') for q in protocol[g]}
    if query_ids.intersection(scan_ids):raise ValueError('Frozen localization queries leaked into temporal evidence')
    if not np.array_equal(scan_ids,protocol['all_odometry_scan_indices']):raise ValueError('Evidence IDs differ from frozen accepted odometry IDs')
    query_times=data['times_ns'][sorted(query_ids)]
    nearest_query_seconds=np.min(np.abs(query_times[:,None]-times[None,:]),axis=1)*1e-9
    evidence,report=compute_temporal_evidence(points,data['ranges'],data['times_ns'],scan_ids,poses,float(data['angle_min']),float(data['angle_increment']))
    evidence_path=args.output/'temporal_evidence.npz';np.savez_compressed(evidence_path,**evidence)
    candidates={}
    for name,keep in [('conservative',~evidence['conflicted']),('static',evidence['static_supported'])]:
        pcd=args.output/f'lidar_scan_map_clean_temporal_{name}.pcd';write_pcd_subset(pcd,header,rows,keep)
        reread,_,rereadrows=read_ascii_pcd(pcd);assert np.array_equal(reread,points[keep]);assert rereadrows==[r for r,k in zip(rows,keep) if k]
        mask=args.output/f'keep_mask_{name}.npy';np.save(mask,keep)
        candidates[name]={'file':pcd.name,'sha256':digest(pcd),'retained_points':int(keep.sum()),'removed_points':int((~keep).sum()),
                          'removed_fraction':float((~keep).mean()),'mask_file':mask.name,'mask_sha256':digest(mask),
                          'unknown_preserved':name=='conservative','exact_original_ascii_rows':True}
    fields=['point_id','x','y','z','state','hit_epochs','free_epochs','raw_free_epochs','occluded_epochs','within_epoch_hit_free_conflicts',
            'hit_span_s','free_span_s','free_fraction','first_hit_raw_scan','last_hit_raw_scan','first_free_raw_scan','last_free_raw_scan']
    with (args.output/'temporal_point_evidence.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for i,p in enumerate(points):
            state=next(k for k in ('conflicted','static_supported','insufficient') if evidence[k][i])
            writer.writerow({'point_id':i,'x':p[0],'y':p[1],'z':p[2],'state':state,**{k:evidence[k][i] for k in fields[5:]}})
    with (args.output/'scan_evidence_summary.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream);writer.writerow(['raw_scan_index','timestamp_ns','epoch','hit_points','free_points','occluded_points']);writer.writerows(evidence['per_scan_counts'])
    examples=plot_evidence(points,evidence,args.output)
    report.update(candidates=candidates,source_paths={k:str(p) for k,p in sources.items()},source_sha256=hashes,
                  evidence_npz=evidence_path.name,evidence_sha256=digest(evidence_path),
                  raw_evidence_scan_ids=scan_ids.tolist(),excluded_frozen_query_scan_ids=sorted(query_ids),query_evidence_intersection=[],
                  selected_trajectory='Current delivered one-loop corrected LiDAR trajectory',
                  corrected_pose_array_sha256=hashlib.sha256(poses.tobytes()).hexdigest(),
                  temporal_exclusion_applied=False,
                  query_nearest_evidence_time_s={'minimum':float(nearest_query_seconds.min()),'median':float(np.median(nearest_query_seconds)),'maximum':float(nearest_query_seconds.max())},
                  query_exclusion_scope='Exact raw query IDs excluded; nearby observations remain, so this is not temporal holdout',
                  camera_landmarks='Unchanged and not filtered',example_point_ids=examples,
                  point_csv='temporal_point_evidence.csv',scan_csv='scan_evidence_summary.csv',elapsed_s=time.monotonic()-started,
                  classification_is_semantic_human_detection=False,absolute_accuracy_validated=False,
                  original_maps_and_other_filters_unchanged=True)
    if any(digest(p)!=hashes[k] for k,p in sources.items()):raise RuntimeError('A source changed during temporal filtering')
    (args.output/'temporal_filter_report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('input_points','static_supported_points','conflicted_points','insufficient_points','conflicted_with_two_or_more_hit_epochs','candidates','elapsed_s')},indent=2),flush=True)


if __name__=='__main__':main()
