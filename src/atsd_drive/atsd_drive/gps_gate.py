"""Qualification of GPS fixes before they may correct the navigation pose."""
import math


class GpsGate:
    def __init__(self, stable_s=5., max_sigma=5., max_offset=30., timeout=2.):
        if not all(math.isfinite(v) and v > 0 for v in (stable_s,max_sigma,max_offset,timeout)):
            raise ValueError('GPS limits must be finite and positive')
        self.stable_s, self.max_sigma, self.max_offset, self.timeout = stable_s,max_sigma,max_offset,timeout
        self.reset()

    def reset(self, reason='acquiring'):
        self.reason = reason
        self.started = self.last_t = None
        self.first = self.last = self.last_odom = None
        self.travel = 0.
        self.count = 0

    def push(self, x, y, sigma, now, odom=None, estimate=None):
        if not all(math.isfinite(v) for v in (x,y,sigma,now)) or not 0 < sigma <= self.max_sigma:
            self.reset('poor_quality'); return False
        if estimate is not None and math.dist((x,y), estimate[:2]) > self.max_offset:
            self.reset('jump'); return False
        if self.last_t is not None and not 0 < now-self.last_t <= self.timeout:
            self.reset('acquiring')
        if self.last is not None:
            ds = math.dist(odom[:2], self.last_odom[:2]) if odom is not None and self.last_odom is not None else 0.
            self.travel += ds
            # Compare lengths: odom's heading need not yet be aligned with map north.
            if (math.dist((x,y), self.last) > ds+max(2.,2*sigma) or
                    math.dist((x,y), self.first) > self.travel+max(3.,3*sigma)):
                self.reset('jump'); return False
        if self.started is None:
            self.started, self.first = now, (x,y)
        self.last_t, self.last, self.last_odom = now, (x,y), odom
        self.count += 1
        good = self.count >= 5 and now-self.started >= self.stable_s
        self.reason = 'ok' if good else 'acquiring'
        return good
