"""Bounded point-to-line scan matching. No ROS and no wheel-motion prior."""
from dataclasses import dataclass
import math
import time
import numpy as np


def wrap(x):
    return (x+math.pi) % (2*math.pi)-math.pi


def transform(points, pose):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return points @ np.array([[c,s],[-s,c]]) + pose[:2]


def inverse(pose):
    c,s=math.cos(pose[2]),math.sin(pose[2]);x,y=pose[:2]
    return (-c*x-s*y, s*x-c*y, -pose[2])


def cloud(ranges, angle_min, angle_step, range_min, range_max, laser=(.29,0.,0.)):
    r=np.asarray(ranges,float)
    if (r.ndim != 1 or len(r)<3 or not all(math.isfinite(v) for v in
            (angle_min,angle_step,range_min,range_max,*laser)) or angle_step<=0 or range_max<=range_min):
        return np.empty((0,2))
    a=angle_min+np.arange(len(r))*angle_step
    valid=np.isfinite(r)&(r>=max(.1,range_min))&(r<=min(8.,range_max))
    return downsample(transform(np.column_stack((r[valid]*np.cos(a[valid]),r[valid]*np.sin(a[valid]))),laser))


def downsample(points, limit=240):
    points=np.asarray(points,float).reshape(-1,2)
    points=points[np.isfinite(points).all(axis=1)]
    if not len(points): return points
    _,idx=np.unique(np.floor(points/.04).astype(int),axis=0,return_index=True)
    points=points[np.sort(idx)]
    if len(points)>limit: points=points[np.linspace(0,len(points)-1,limit).astype(int)]
    return points


@dataclass
class Match:
    valid: bool = False
    reason: str = 'few_returns'
    pose: tuple = (0.,0.,0.)
    rmse: float = 0.
    overlap: float = 0.
    geometry: float = 0.
    iterations: int = 0


def match(current, reference, yaw_hint=0., budget_s=.08):
    """Return transform current base -> previous base, rejecting weak geometry."""
    cur,ref=downsample(current),downsample(reference,360)
    out=Match()
    if min(len(cur),len(ref))<40 or not math.isfinite(yaw_hint): return out
    deadline=time.monotonic()+budget_s
    distances=((ref[:,None,:]-ref[None,:,:])**2).sum(axis=2)
    neighbors=np.argpartition(distances,5,axis=1)[:,:6]
    local=ref[neighbors];centered=local-local.mean(axis=1,keepdims=True)
    eig,vectors=np.linalg.eigh(np.einsum('nki,nkj->nij',centered,centered)/6)
    normals=vectors[:,:,0]
    usable=(eig[:,1]>.0002)&(eig[:,0]<.15*eig[:,1])&(distances[np.arange(len(ref))[:,None],neighbors].max(axis=1)<.6**2)
    pose=np.array([0.,0.,yaw_hint])
    converged=False
    for iteration in range(20):
        if time.monotonic()>deadline:
            out.reason='budget';return out
        p=transform(cur,pose)
        d2=((p[:,None,:]-ref[None,:,:])**2).sum(axis=2)
        nearest=d2.argmin(axis=1)
        keep=(d2[np.arange(len(p)),nearest]<.30**2)&usable[nearest]
        out.overlap=float(keep.mean());out.iterations=iteration+1
        if keep.sum()<40 or out.overlap<.55:
            out.reason='low_overlap';return out
        pp,rr,nn=p[keep],ref[nearest[keep]],normals[nearest[keep]]
        residual=np.sum(nn*(pp-rr),axis=1)
        # Trim inconsistent returns (moving objects and new occlusions).
        good=np.abs(residual)<=max(.015,float(np.quantile(np.abs(residual),.85)))
        pp,nn,residual=pp[good],nn[good],residual[good]
        A=np.column_stack((nn[:,0],nn[:,1],-nn[:,0]*pp[:,1]+nn[:,1]*pp[:,0]))
        scale=np.maximum(np.linalg.norm(A,axis=0),1e-6)
        normalized=A/scale
        info=np.linalg.eigvalsh(normalized.T@normalized)
        directions=np.linalg.eigvalsh(nn.T@nn/len(nn))
        out.geometry=float(min(info[0]/max(info[-1],1e-9),directions[0]))
        if out.geometry<.015:
            out.reason='degenerate';return out
        step=np.linalg.lstsq(A,-residual,rcond=None)[0]
        if np.linalg.norm(step[:2])>.25 or abs(step[2])>.15:
            out.reason='large_step';return out
        # Linearization above uses left increments in reference coordinates.
        pose[:2]=transform(pose[None,:2],step)[0]
        pose[2]=wrap(pose[2]+step[2])
        out.rmse=float(np.sqrt(np.mean(residual**2)))
        if np.linalg.norm(step[:2])<.001 and abs(step[2])<.001:
            converged=True;break
    if not converged or out.rmse>.05:
        out.reason='residual' if converged else 'not_converged';return out
    out.valid=True;out.reason='ok';out.pose=tuple(float(v) for v in pose)
    return out


class ScanOdometry:
    def __init__(self,budget_s=.08):
        self.budget_s=budget_s
        self.reset()

    def reset(self):
        self.reference=None;self.stamp=None;self.history=[]

    def update(self, points, stamp, yaw_hint=0.):
        points=downsample(points)
        if not math.isfinite(stamp):
            self.reset();return Match(reason='invalid_stamp')
        dt=stamp-self.stamp if self.stamp is not None else 0.
        if self.reference is None or not .04<=dt<=.5:
            self.reference=points;self.history=[points];self.stamp=stamp
            return Match(reason='initializing' if dt==0 else 'scan_gap')
        result=match(points,self.reference,yaw_hint,self.budget_s)
        if result.valid and (math.hypot(*result.pose[:2])>1.2*dt+.02 or abs(result.pose[2])>2.*dt+.02):
            result.valid=False;result.reason='implausible_motion'
        self.stamp=stamp
        if result.valid:
            # Rolling map, at most three scans, expressed in the newest base frame.
            self.history=[transform(p,inverse(result.pose)) for p in self.history[-2:]]+[points]
            self.reference=downsample(np.concatenate(self.history),360)
        else:
            # No silent integration across rejected scans; acquire a fresh reference.
            self.history=[points];self.reference=points
        return result


class PoseHistory:
    def __init__(self): self.items=[]

    def add(self, stamp, pose):
        if not all(math.isfinite(v) for v in (stamp,*pose)): return
        if self.items and stamp<=self.items[-1][0]: self.items=[]
        self.items.append((stamp,tuple(pose)))
        self.items=[item for item in self.items if stamp-item[0]<3.]

    def at(self, stamp):
        for (a,p),(b,q) in zip(self.items,self.items[1:]):
            if a<=stamp<=b and b-a<=.3:
                t=(stamp-a)/(b-a)
                return (p[0]+t*(q[0]-p[0]),p[1]+t*(q[1]-p[1]),wrap(p[2]+t*wrap(q[2]-p[2])))
        return None


class SlipMonitor:
    def __init__(self): self.items=[]
    def reset(self): self.items=[]
    def update(self, stamp, wheel_delta, lidar_delta):
        self.items.append((stamp,math.hypot(*wheel_delta[:2]),math.hypot(*lidar_delta[:2]),abs(wheel_delta[2]),abs(lidar_delta[2])))
        self.items=[v for v in self.items if stamp-v[0]<=1.]
        total=np.sum([v[1:] for v in self.items],axis=0)
        wd,ld,wa,la=total
        return bool(len(self.items)>=3 and ((wd>.08 and wd-ld>.06 and ld<.5*wd) or
                                           (wa>.2 and wa-la>.15 and la<.5*wa)))
