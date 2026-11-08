import numpy as np
from scipy.spatial import cKDTree
def wrap(a):return (a+np.pi)%(2*np.pi)-np.pi
def rot(a):return np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
def transform(p,x):return p@rot(x[2]).T+x[:2]
def relative(a,b):return np.r_[rot(a[2]).T@(b[:2]-a[:2]),wrap(b[2]-a[2])]
def compose(a,b):return np.r_[transform(b[None,:2],a)[0],wrap(a[2]+b[2])]
def voxel(p,s=.035):
 if len(p)==0:return p
 _,ids=np.unique(np.floor(p/s).astype(int),axis=0,return_index=True);return p[ids]
def points(r,angles):
 valid=np.isfinite(r)&(r>.22)&(r<9)
 # Isolated angular returns and range discontinuities are unreliable.
 with np.errstate(invalid='ignore'):
  delta=np.minimum(abs(r-np.roll(r,1)),abs(r-np.roll(r,-1)))
 valid &= delta<.07+.015*r
 return voxel(np.c_[r[valid]*np.cos(angles[valid]),r[valid]*np.sin(angles[valid])])
def normals(q):
 tree=cKDTree(q);d,ix=tree.query(q,k=min(9,len(q)))
 nb=q[ix];dev=nb-nb.mean(axis=1)[:,None,:];cov=np.einsum('nki,nkj->nij',dev,dev)
 eig,v=np.linalg.eigh(cov);n=v[:,:,0]; good=(eig[:,0]/np.maximum(eig[:,1],1e-12)<.12)&(d[:,-1]<.3)
 return tree,n,good

def icp(p,q,guess,limit=.25,iters=20,model=None):
 if len(p)<40 or len(q)<40:return guess,999,0
 tree,n,good=normals(q) if model is None else model
 x=guess.copy();support=0
 for _ in range(iters):
  z=transform(p,x);d,ix=tree.query(z);ok=(d<limit)&good[ix]
  if ok.sum()<40:return x,999,float(ok.mean())
  z=z[ok];nn=n[ix[ok]];err=np.sum(nn*(z-q[ix[ok]]),axis=1)
  med=np.median(abs(err));w=np.minimum(1,.025/np.maximum(abs(err),1e-9))
  a=np.c_[nn,nn[:,0]*(-(z[:,1]-x[1]))+nn[:,1]*(z[:,0]-x[0])]
  step=np.linalg.lstsq(a*w[:,None]**.5,-err*w**.5,rcond=None)[0]
  if np.linalg.norm(step[:2])>.3 or abs(step[2])>.2:break
  x+=step;x[2]=wrap(x[2]);support=ok.mean()
  if np.linalg.norm(step)<1e-5:break
 z=transform(p,x);d,ix=tree.query(z);ok=(d<.12)&good[ix]
 err=np.abs(np.sum(n[ix[ok]]*(z[ok]-q[ix[ok]]),axis=1))
 return x,float(np.sqrt(np.mean(np.minimum(err,.1)**2))) if len(err) else 999,float(np.mean(d<.12))

