"""Timestamp association and anchored SE(2) pose-graph least squares."""
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import coo_matrix


def nearest_timestamps(target_ns, query_ns, tolerance_ns=50_000_000):
    target_ns=np.asarray(target_ns,dtype=np.int64)
    query_ns=np.asarray(query_ns,dtype=np.int64)
    if not len(target_ns):
        raise ValueError('Cannot associate an empty timestamp sequence')
    if np.any(np.diff(target_ns)<0):
        raise ValueError('Target timestamps must be sorted')
    right=np.clip(np.searchsorted(target_ns,query_ns),0,len(target_ns)-1)
    left=np.maximum(right-1,0)
    use_left=np.abs(target_ns[left]-query_ns)<=np.abs(target_ns[right]-query_ns)
    ids=np.where(use_left,left,right)
    delta=np.abs(target_ns[ids]-query_ns)
    return ids,delta,delta<=tolerance_ns


def wrap(a):
    return (a+np.pi)%(2*np.pi)-np.pi


def optimize_graph(poses,edges):
    poses=np.asarray(poses,dtype=float)
    if len(poses)<2 or not any(e['loop'] for e in edges):
        return poses.copy(),{'success':True,'message':'No accepted loop edges; odometry unchanged'}
    ii=np.array([e['i'] for e in edges]); jj=np.array([e['j'] for e in edges])
    zz=np.asarray([e['pose'] for e in edges])
    # Engineering weights, not empirically calibrated measurement covariance.
    sigma=np.array([[.08,.08,.035] if e['loop'] else [.06,.06,.025] for e in edges])
    def residual(x):
        p=np.vstack((poses[0],x.reshape(-1,3)))
        d=p[jj,:2]-p[ii,:2]; c=np.cos(p[ii,2]);s=np.sin(p[ii,2])
        predicted=np.column_stack((c*d[:,0]+s*d[:,1],-s*d[:,0]+c*d[:,1],wrap(p[jj,2]-p[ii,2])))
        err=predicted-zz;err[:,2]=wrap(err[:,2])
        return (err/sigma).ravel()
    def jacobian(x):
        p=np.vstack((poses[0],x.reshape(-1,3)))
        d=p[jj,:2]-p[ii,:2];c=np.cos(p[ii,2]);s=np.sin(p[ii,2])
        u=c*d[:,0]+s*d[:,1];v=-s*d[:,0]+c*d[:,1]
        rows=[];cols=[];values=[]
        for k,(i,j) in enumerate(zip(ii,jj)):
            blocks=(((i,np.array([[-c[k],-s[k],v[k]],[s[k],-c[k],-u[k]],[0.,0.,-1.]]))),
                    ((j,np.array([[c[k],s[k],0.],[-s[k],c[k],0.],[0.,0.,1.]]))))
            for node,block in blocks:
                if node==0:continue
                block=block/sigma[k,:,None]
                for r in range(3):
                    for col in range(3):
                        if block[r,col]!=0:
                            rows.append(k*3+r);cols.append((node-1)*3+col);values.append(block[r,col])
        return coo_matrix((values,(rows,cols)),shape=(3*len(edges),3*(len(poses)-1))).tocsr()
    result=least_squares(residual,poses[1:].ravel(),jac=jacobian,loss='soft_l1',f_scale=1.,max_nfev=500,
                         ftol=1e-5,xtol=1e-6,tr_options={'atol':1e-9,'btol':1e-9,'maxiter':5000})
    corrected=np.vstack((poses[0],result.x.reshape(-1,3)))
    corrected[:,2]=wrap(corrected[:,2])
    if not result.success or not np.all(np.isfinite(corrected)):
        return poses.copy(),{'success':False,'message':str(result.message),'applied':False,'evaluations':result.nfev}
    return corrected,{'success':bool(result.success),'message':result.message,'cost':float(result.cost),'evaluations':result.nfev,
                      'weighted_rms_before':float(np.sqrt(np.mean(residual(poses[1:].ravel())**2))),
                      'weighted_rms_after':float(np.sqrt(np.mean(residual(result.x)**2)))}
