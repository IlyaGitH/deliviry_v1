#!/usr/bin/env python3
"""ROS-free closed-loop raycast simulation of the production local planner.
Run: PYTHONPATH=src/atsd_drive python tools/simulate_avoidance.py --output /tmp/avoidance
2D ideal odometry, static geometry, 5 Hz lidar, 10 Hz control, limited acceleration.
This does not model wheel slip, low obstacles, drop-offs or person prediction.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import numpy as np
from atsd_drive.road_logic import RoadNav, Roads
from atsd_drive.local_planner import LocalCfg, LocalPlanner, ScanSpace, advance, wrap


def rectangle(x0, y0, x1, y1):
    p = [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
    return [(p[i],p[(i+1)%4]) for i in range(4)]


def raycast(pose, segments, n=455, rmax=12.):
    yaw = pose[2]
    origin = np.array(pose[:2]) + .29*np.array([math.cos(yaw),math.sin(yaw)])
    angles = -math.pi + np.arange(n)*2*math.pi/n + yaw
    rays = np.column_stack((np.cos(angles),np.sin(angles)))
    ranges = np.full(n,np.inf)
    for a,b in segments:
        delta = np.array(b)-a
        offset = np.array(a)-origin
        den = rays[:,0]*delta[1]-rays[:,1]*delta[0]
        valid = abs(den)>1e-9
        den = np.where(valid,den,1.)
        t = (offset[0]*delta[1]-offset[1]*delta[0])/den
        u = (offset[0]*rays[:,1]-offset[1]*rays[:,0])/den
        hit = valid & (t>=.05)&(t<=rmax)&(u>=0)&(u<=1)
        ranges[hit] = np.minimum(ranges[hit],t[hit])
    return ranges


def relative(point, anchor):
    dx,dy = point[0]-anchor[0],point[1]-anchor[1]
    c,s = math.cos(anchor[2]), math.sin(anchor[2])
    return c*dx+s*dy,-s*dx+c*dy


def collision(pose, segments, cfg):
    # Independent exact segment/axis-aligned rectangle intersection in body frame.
    for a,b in segments:
        a,b = np.array(relative(a,pose)),np.array(relative(b,pose))
        d=b-a
        lo,hi=0.,1.
        for axis,lower,upper in ((0,-cfg.rear,cfg.front),(1,-cfg.half_width,cfg.half_width)):
            if abs(d[axis])<1e-10:
                if a[axis]<lower or a[axis]>upper:
                    lo,hi=1.,0.
                    break
            else:
                t0,t1=sorted(((lower-a[axis])/d[axis],(upper-a[axis])/d[axis]))
                lo,hi=max(lo,t0),min(hi,t1)
        if lo<=hi:
            return True
    return False


def scenarios():
    return {
        'open': ([], (0.,0.,0.), (5.,0.)),
        'box': (rectangle(2.,-.55,2.8,.55), (0.,0.,0.), (5.,0.)),
        'left_blocked': (rectangle(2.,-.4,2.8,2.5), (0.,0.,0.), (5.,0.)),
        'right_blocked': (rectangle(2.,-2.5,2.8,.4), (0.,0.,0.), (5.,0.)),
        'slalom': (rectangle(2.,-.7,2.5,.25)+rectangle(4.,-.1,4.5,1.0), (0.,0.,0.), (7.,0.)),
        'wide_passage': (rectangle(-1.,.8,6.,1.0)+rectangle(-1.,-1.,6.,-.8), (0.,0.,0.), (5.,0.)),
        'narrow_passage': (rectangle(-1.,.4,6.,.6)+rectangle(-1.,-.6,6.,-.4), (0.,0.,0.), (5.,0.)),
        'dead_end': (rectangle(-1.,-.8,2.,-.6)+rectangle(-1.,.6,2.,.8)+rectangle(1.8,-.8,2.,.8), (0.,0.,0.), (4.,0.)),
        'enclosed': (rectangle(-.8,-.8,.8,.8), (0.,0.,0.), (4.,0.)),
        'reverse_arc': (rectangle(.95,-1.,1.2,1.), (0.,0.,0.), (4.,0.)),
        'scan_loss': ([], (0.,0.,0.), (5.,0.)),
        'odom_loss': ([], (0.,0.,0.), (5.,0.)),
        'invalid_ranges': ([], (0.,0.,0.), (5.,0.)),
        'moving_front': ([], (0.,0.,0.), (5.,0.)),
        'rear_blocked': (rectangle(.8,-1.,1.,1.)+rectangle(-1.,-1.,-.8,1.), (0.,0.,0.), (4.,0.)),
    }


def stress_scenarios(count=10, seed=482):
    rng=np.random.default_rng(seed)
    cases={}
    for i in range(count):
        x=float(rng.uniform(1.6,2.5)); y=float(rng.uniform(-.45,.45)); half=float(rng.uniform(.25,.85))
        geometry=rectangle(x,y-half,x+float(rng.uniform(.3,.9)),y+half)
        cases[f'random_box_{i}']=(geometry,(0.,0.,float(rng.uniform(-.15,.15))),(5.,0.))
    return cases


def run(name, data, duration=100., cfg=None):
    segments,pose,goal=data
    cfg=cfg or LocalCfg()
    planner=LocalPlanner(cfg)
    nav=RoadNav(Roads(allow_direct=True))
    nav.filter.set_manual(*pose)
    nav.plan(goal)
    v,w=0.,0.
    trace=[]; states=Counter(); times=[]
    space=None; anchor=pose; scan_t=0.
    collided=False; reached=False; reverse=0.
    blocked_since=None; outcome='time_limit'
    original_segments=segments
    snapshots=[]
    for tick in range(int(duration*10)):
        now=tick*.1
        segments = original_segments + (rectangle(1.7,-.5,2.,.5) if name=='moving_front' and 4.<=now<10. else [])
        if tick%2==0 and not (name=='scan_loss' and 4.<=now<6.):
            anchor=pose; scan_t=now
            ranges = np.full(455,np.nan) if name=='invalid_ranges' and 4.<=now<6. else raycast(pose,segments)
            space=ScanSpace(ranges,-math.pi,2*math.pi/455,.05,12.,cfg)
        local=(*relative(pose,anchor),wrap(pose[2]-anchor[2]))
        t=time.perf_counter()
        nav.filter.X[:] = pose
        preferred_v, preferred_w, done = nav.tick(None, correct_lane=planner.mode!='avoid', strict_arrival=True)
        local_goal = nav.follower._lookahead(pose[:2], 4.5)
        p0, bearing = nav.follower.segment()
        side = abs(-math.sin(bearing)*(pose[0]-p0[0])+math.cos(bearing)*(pose[1]-p0[1]))
        rejoined = side < .25 and abs(wrap(pose[2]-bearing)) < .15
        preferred = (preferred_v,preferred_w) if planner.mode!='avoid' or rejoined else None
        odom_age = now-4. if name=='odom_loss' and 4.<=now<6. else 0.
        d=planner.decide(space,local,relative(local_goal,anchor),now,now-scan_t,v,w,odom_age,pose,preferred)
        if abs(d.v)>.01: blocked_since=None
        elif blocked_since is None: blocked_since=now
        times.append((time.perf_counter()-t)*1000)
        states[d.state]+=1
        # The production Brain's independent forward corridor guard.
        # Front clearance below .50 m means no forward movement, even along an arc.
        front=raycast(pose,segments)
        angles=-math.pi+np.arange(455)*2*math.pi/455
        xx=.29+front*np.cos(angles); yy=front*np.sin(angles)
        guard=np.any(np.isfinite(front)&(xx>cfg.front)&(xx<cfg.front+.5)&(abs(yy)<.35))
        target_v,target_w=d.v,d.w
        if guard and target_v>0: target_v=target_w=0.
        old_v=v
        v += np.clip(target_v-v,-.35*.1,.35*.1)
        # Curvature-preserving motor controller; stop before changing gear.
        k=target_w/target_v if abs(target_v)>1e-8 else (w/old_v if abs(old_v)>.01 else 0.)
        w=v*k
        for _ in range(5):
            pose=advance(pose,v*.02,k)
            if collision(pose,segments,cfg): collided=True; break
        reverse+=max(0.,-v)*.1
        trace.append([round(now,2),*pose,v,w,d.state])
        snapshots.append(segments)
        if collided: outcome='collision'; break
        if math.dist(pose[:2],goal)<.3: reached=True; outcome='arrived'; break
        if d.state in ('stalled','escape_timeout'): outcome=d.state; break
        if blocked_since is not None and now-blocked_since>30.: outcome='blocked_timeout'; break
    return {'name':name,'outcome':outcome,'reached':reached,'collision':collided,'end_distance':round(math.dist(pose[:2],goal),3),
            'reverse_m':round(reverse,3),'sim_seconds':round(now,1),'states':dict(states),
            'plan_ms_p95':round(float(np.percentile(times,95)),2),'plan_ms_max':round(max(times),2),
            'trace':trace,'geometry':original_segments,'start':data[1],'goal':goal,
            'snapshots':snapshots if name=='moving_front' else None}


def svg(result,data,path):
    segments,start,goal=data
    points=[p for seg in segments for p in seg]+[t[1:3] for t in result['trace']]+[start[:2],goal]
    lo=np.min(points,axis=0)-1; hi=np.max(points,axis=0)+1
    scale=75
    def xy(p): return ((p[0]-lo[0])*scale,(hi[1]-p[1])*scale)
    width,height=(hi-lo)*scale
    parts=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}"><rect width="100%" height="100%" fill="white"/>']
    for a,b in segments:
        x,y=xy(a);xx,yy=xy(b)
        parts.append(f'<path d="M{x},{y} L{xx},{yy}" stroke="#333" stroke-width="4"/>')
    trace=result['trace']
    for a,b in zip(trace,trace[1:]):
        x,y=xy(a[1:3]);xx,yy=xy(b[1:3]);color='#e45c23' if b[4]<0 else '#087cce'
        parts.append(f'<path d="M{x},{y} L{xx},{yy}" stroke="{color}" stroke-width="3"/>')
    for p,color in ((start,'green'),(goal,'red')):
        x,y=xy(p);parts.append(f'<circle cx="{x}" cy="{y}" r="6" fill="{color}"/>')
    parts.append('</svg>');path.write_text(''.join(parts))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='/tmp/avoidance');parser.add_argument('--case',default='all');parser.add_argument('--duration',type=float,default=100.)
    parser.add_argument('--stress',type=int,default=0);parser.add_argument('--check',action='store_true')
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    results=[]; full_results=[]
    cases=stress_scenarios(args.stress) if args.stress else scenarios()
    for name,data in cases.items():
        if args.case!='all' and name!=args.case:continue
        result=run(name,data,args.duration)
        svg(result,data,out/f'{name}.svg')
        (out/f'{name}.json').write_text(json.dumps(result))
        full_results.append(result.copy())
        for key in ('trace','geometry','start','goal','snapshots'): result.pop(key,None)
        results.append(result);print(json.dumps(result),flush=True)
    (out/'summary.json').write_text(json.dumps(results,indent=2))
    template=Path(__file__).with_name('avoidance_viewer.html').read_text()
    (out/'simulation.html').write_text(template.replace('__DATA__',json.dumps(full_results)))
    if args.check:
        for r in results:
            assert not r['collision'], r
            expected='blocked_timeout' if r['name'] in ('narrow_passage','enclosed','rear_blocked') else 'arrived'
            assert r['outcome']==expected, r

if __name__=='__main__':main()
