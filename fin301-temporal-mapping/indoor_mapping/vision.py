"""Camera appearance retrieval with ORB and robust image-geometry verification.

Image geometry gates candidates only; all metric constraints come from LiDAR.
The image descriptor is intentionally local/reproducible, with no model download.
"""
from pathlib import Path
import cv2
import numpy as np


def image_features(path):
    image=cv2.imread(str(path))
    if image is None:raise ValueError(f'Cannot decode {path}')
    if image.shape[1]>640:
        image=cv2.resize(image,(640,round(image.shape[0]*640/image.shape[1])))
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    small=cv2.resize(gray,(32,24)).astype(np.float32)
    small=(small-small.mean())/(small.std()+10)
    desc=small.ravel();desc/=np.linalg.norm(desc)+1e-9
    kp,orb=cv2.ORB_create(nfeatures=1000,fastThreshold=15).detectAndCompute(gray,None)
    xy=np.array([p.pt for p in kp],dtype=np.float32).reshape(-1,2)
    return {'global':desc,'xy':xy,'orb':orb,'path':str(path),'image_area':int(gray.size),'blur':float(cv2.Laplacian(gray,cv2.CV_64F).var())}


def verify_images(a,b):
    if a['orb'] is None or b['orb'] is None or min(len(a['orb']),len(b['orb']))<30:
        return {'success':False,'matches':0,'inliers':0,'inlier_ratio':0.}
    matches=cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(a['orb'],b['orb'],k=2)
    good=[m[0] for m in matches if len(m)==2 and m[0].distance<.72*m[1].distance]
    # Remove duplicate train features so repeated texture cannot inflate support.
    unique={}
    for m in sorted(good,key=lambda m:m.distance):unique.setdefault(m.trainIdx,m)
    good=list(unique.values())
    if len(good)<24:return {'success':False,'matches':len(good),'inliers':0,'inlier_ratio':0.}
    pa=np.array([a['xy'][m.queryIdx] for m in good]);pb=np.array([b['xy'][m.trainIdx] for m in good])
    _,mask=cv2.findHomography(pa,pb,cv2.RANSAC,3.,maxIters=1500,confidence=.995)
    selected=np.zeros(len(good),bool) if mask is None else mask.ravel().astype(bool)
    n=int(selected.sum()); ratio=n/len(good)
    # Require feature spread rather than a small repeated sign/patch alone.
    spread=0.;spread_b=0.
    if n>=3:
        spread=float(cv2.contourArea(cv2.convexHull(pa[selected]))/a['image_area'])
        spread_b=float(cv2.contourArea(cv2.convexHull(pb[selected]))/b['image_area'])
    spread=min(spread,spread_b)
    return {'success':n>=30 and ratio>=.55 and spread>=.035,'matches':len(good),'inliers':n,
            'inlier_ratio':ratio,'spread':spread,'points_a':pa[selected].tolist(),'points_b':pb[selected].tolist()}
