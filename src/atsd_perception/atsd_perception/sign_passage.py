"""Image tracks for signs passed by the camera; no range estimate involved."""


def area(d):
    x,y,X,Y=d['box']
    return max(0,X-x)*max(0,Y-y)


def overlap(a,b):
    x,y,X,Y=a; u,v,U,V=b
    intersection=max(0,min(X,U)-max(x,u))*max(0,min(Y,V)-max(y,v))
    union=max(0,X-x)*max(0,Y-y)+max(0,U-u)*max(0,V-v)-intersection
    return intersection/union if union else 0


class SignPassage:
    def __init__(self, frames=6):
        self.frames=frames
        self.tracks=[]
        self.last=None

    def update(self, detections, now):
        # A camera/inference outage is not evidence of passing a sign.
        if self.last is not None and now-self.last>1.0:
            self.tracks=[t for t in self.tracks if t['armed']]
            for t in self.tracks: t['miss']=0
        self.last=now
        available=[d for d in detections if d.get('kind') in ('STOP','BUMP') and area(d)>0]
        events=[]; keep=[]
        for t in self.tracks:
            matches=[d for d in available if d['kind']==t['kind'] and overlap(d['box'],t['box'])>.1]
            if matches:
                d=max(matches,key=lambda d:overlap(d['box'],t['box']))
                available.remove(d)
                t['box']=list(d['box']);t['miss']=0;t['hits']+=1
                t['armed']=t['armed'] or t['hits']>=self.frames
                keep.append(t)
            elif t['armed']:
                t['miss']+=1
                if t['miss']>=self.frames: events.append(t['kind'])
                else: keep.append(t)
        # Acquire the largest remaining sign. Existing tracks continue independently.
        if available and len(keep)<16:
            d=max(available,key=area)
            keep.append(dict(kind=d['kind'],box=list(d['box']),hits=1,miss=0,armed=False))
        self.tracks=keep
        return events
