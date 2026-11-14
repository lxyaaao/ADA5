"""Inspectable result figures; no sensor data are synthesized."""
from pathlib import Path
import os
_mpl_cache=Path(__file__).resolve().parent.parent/'outputs'/'.matplotlib'
_mpl_cache.mkdir(parents=True,exist_ok=True)
os.environ.setdefault('MPLCONFIGDIR',str(_mpl_cache))
import json
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def make_figures(output,occupancy,initial,corrected,events,times,visual_points=None,lidar_point_count=None,lidar_loops=False):
    output=Path(output)
    visual_points=np.empty((0,3)) if visual_points is None else np.asarray(visual_points)
    has_loops=lidar_loops or any(e['accepted'] for e in events)
    grid=occupancy['grid'];origin=occupancy['origin'];res=occupancy['resolution']
    extent=[origin[0],origin[0]+grid.shape[1]*res,origin[1],origin[1]+grid.shape[0]*res]
    fig,ax=plt.subplots(figsize=(9,6),layout='constrained')
    ax.imshow(grid,cmap='gray',vmin=0,vmax=255,origin='lower',extent=extent)
    ax.plot(initial[:,0],initial[:,1],color='#e6922a',lw=.9,label='LiDAR-only trajectory',alpha=.8)
    ax.plot(corrected[:,0],corrected[:,1],color='#1978ab',lw=1.,label='Loop-corrected trajectory' if has_loops else 'LiDAR trajectory')
    ax.scatter(*corrected[0,:2],s=55,c='#2a9d59',marker='o',label='Start',zorder=5)
    ax.scatter(*corrected[-1,:2],s=55,c='#c13742',marker='x',label='End',zorder=5)
    ax.set(xlabel='Map x (m)',ylabel='Map y (m)',title='Indoor scan-plane map and estimated trajectory')
    ax.set_aspect('equal');ax.legend(fontsize=8,loc='best');fig.savefig(output/'map_trajectory.png',dpi=200,bbox_inches='tight');plt.close(fig)
    # Primary delivered format is PCD. This plot shows the exact exported points.
    points=[]
    with (output/'map.pcd').open() as f:
        for line in f:
            if line.startswith('DATA'):break
        for line in f:
            parts=line.split()
            if len(parts)>=3:points.append([float(parts[0]),float(parts[1])])
    points=np.array(points)
    fig,ax=plt.subplots(figsize=(9,6),layout='constrained')
    n=len(points) if lidar_point_count is None else lidar_point_count
    ax.scatter(points[:n,0],points[:n,1],s=.1,c='#233746',rasterized=True,label='LiDAR scan plane')
    if len(points)>n:
        ax.scatter(points[n:,0],points[n:,1],s=.5,c='#159b89',rasterized=True,label='Camera landmarks (XY projection)')
    ax.plot(corrected[:,0],corrected[:,1],color='#da6f27',lw=.8,label='Estimated trajectory')
    ax.set(xlabel='Map x (m)',ylabel='Map y (m)',title='Combined PCD: LiDAR geometry + camera landmarks' if len(points)>n else 'Exported PCD: 2D LiDAR returns, z = 0 m')
    ax.legend(fontsize=7,markerscale=5)
    ax.set_aspect('equal');ax.grid(alpha=.15);fig.savefig(output/'pcd_preview.png',dpi=220,bbox_inches='tight');plt.close(fig)
    accepted=[e for e in events if e['accepted']]
    if not accepted and (output/'camera_revisit.png').exists():
        (output/'camera_revisit.png').unlink()
    if accepted:
        e=max(accepted,key=lambda e:e['visual']['inliers'])
        a=cv2.imread(e['image_i']);b=cv2.imread(e['image_j'])
        def resize(im):return cv2.resize(im,(640,round(im.shape[0]*640/im.shape[1])))
        a=resize(a);b=resize(b);h=max(a.shape[0],b.shape[0]);canvas=np.full((h+45,1280,3),255,np.uint8)
        canvas[45:45+a.shape[0],:640]=a;canvas[45:45+b.shape[0],640:]=b
        cv2.putText(canvas,f'Camera revisit: {e["time_i_s"]:.1f}s -> {e["time_j_s"]:.1f}s | {e["visual"]["inliers"]} ORB inliers',(12,29),cv2.FONT_HERSHEY_SIMPLEX,.7,(25,25,25),2)
        rng=np.random.default_rng(42)
        pairs=list(zip(e['visual']['points_a'],e['visual']['points_b']))
        for index in np.linspace(0,len(pairs)-1,min(35,len(pairs)),dtype=int):
            pa,pb=pairs[index];color=tuple(map(int,rng.integers(30,230,3)))
            cv2.line(canvas,(round(pa[0]),round(pa[1])+45),(round(pb[0])+640,round(pb[1])+45),color,1)
        cv2.imwrite(str(output/'camera_revisit.png'),canvas)
    fig,ax=plt.subplots(figsize=(8,3),layout='constrained')
    ax.plot((times-times[0])*1e-9,np.linalg.norm(corrected[:,:2]-initial[:,:2],axis=1),color='#1978ab')
    ax.set(xlabel='Elapsed time (s)',ylabel='Graph correction (m)',title='Pose correction from accepted revisit constraints')
    ax.grid(alpha=.2);fig.savefig(output/'graph_correction.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,3),layout='constrained')
    if not accepted:
        ax.plot((times-times[0])*1e-9,np.zeros(len(times)))
        ax.set_title('No camera pose constraints accepted')
    elif not lidar_loops:
        ax.plot((times-times[0])*1e-9,np.linalg.norm(corrected[:,:2]-initial[:,:2],axis=1))
        ax.set_title('Camera-proposed, LiDAR-verified pose correction')
    else:
        ax.text(.5,.5,'Joint graph: camera contribution not separately attributed',ha='center',transform=ax.transAxes)
    ax.set(xlabel='Elapsed time (s)',ylabel='Camera pose correction (m)')
    fig.savefig(output/'camera_contribution.png',dpi=180);plt.close(fig)
    if len(visual_points):
        fig=plt.figure(figsize=(9,6),layout='constrained');ax=fig.add_subplot(projection='3d')
        artist=ax.scatter(visual_points[:,0],visual_points[:,1],visual_points[:,2],s=2,c=visual_points[:,2],cmap='viridis',rasterized=True)
        ax.set(xlabel='Map x (m)',ylabel='Map y (m)',zlabel='Height relative to initial scan plane (m)',title='Sparse camera landmarks: scale from LiDAR motion')
        span=np.maximum(np.ptp(visual_points,axis=0),.5);ax.set_box_aspect(span)
        fig.colorbar(artist,ax=ax,shrink=.6,label='z (m)')
        fig.savefig(output/'visual_landmarks_3d.png',dpi=180,bbox_inches='tight');plt.close(fig)
