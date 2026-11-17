"""Advisory intrusion detection only: NEVER emits a motion/stop command."""
import math


class RearGuard:
    def __init__(self, rear=.55, half_width=.20, duration=1., min_points=2, max_gap=.45):
        self.rear, self.half_width = rear, half_width
        self.duration, self.min_points, self.max_gap = duration, min_points, max_gap
        self.since = self.last = None
        self.latched = False

    def reset(self):
        self.since = self.last = None
        self.latched = False

    def update(self, ranges, angle_min, angle_increment, range_min, range_max, now, moving):
        if not moving:
            self.reset()
            return False
        if self.last is not None and not 0 < now-self.last <= self.max_gap:
            self.reset()
        self.last = now
        count = 0
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or not max(range_min, .02) <= r <= range_max:
                continue
            angle = angle_min + i*angle_increment
            x, y = r*math.cos(angle), r*math.sin(angle)
            if -self.rear <= x < 0 and abs(y) <= self.half_width:
                count += 1
        if count < self.min_points:
            self.since = None
            self.latched = False
            return False
        if self.since is None:
            self.since = now
        if not self.latched and now-self.since >= self.duration:
            self.latched = True
            return True
        return False
