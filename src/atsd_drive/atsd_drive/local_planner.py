"""Bounded local arc search in observed lidar free space; no GPS or ROS dependency.
Positive infinity means free to range_max; NaN and missing sectors remain unknown. This is local
avoidance, not global road replanning or a guarantee against unobserved hazards.
"""
from dataclasses import dataclass
import heapq
import math
import time

import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def relative_pose(pose, anchor):
    dx, dy = pose[0]-anchor[0], pose[1]-anchor[1]
    c, s = math.cos(anchor[2]), math.sin(anchor[2])
    return (c*dx+s*dy, -s*dx+c*dy, wrap(pose[2]-anchor[2]))


@dataclass
class LocalCfg:
    front: float = .35
    rear: float = .40
    half_width: float = .25
    margin: float = .12
    resolution: float = .10
    extent: float = 7.0
    radius: float = 1.0
    cruise_speed: float = .8
    forward_speed: float = .25
    reverse_speed: float = .15
    allow_reverse: bool = True  # user verified unobstructed 360-degree lidar coverage
    scan_timeout: float = .45
    odom_timeout: float = .35
    deceleration: float = .35  # conservative; measure under load
    latency: float = .65  # DDS command age + Brain watchdog + control-cycle allowance
    step: float = .40
    max_nodes: int = 700
    budget_s: float = .04
    short_reverse_m: float = 5.0  # prefer backing to a nearby goal behind the robot
    max_reverse_m: float = 5.0
    stall_s: float = 3.0
    max_avoid_s: float = 120.0

    def __post_init__(self):
        positive = (self.front, self.rear, self.half_width, self.margin, self.resolution,
                    self.extent, self.radius, self.cruise_speed, self.forward_speed, self.reverse_speed,
                    self.deceleration, self.step, self.scan_timeout, self.odom_timeout,
                    self.budget_s, self.short_reverse_m, self.max_reverse_m, self.stall_s, self.max_avoid_s)
        if not all(math.isfinite(x) and x > 0 for x in positive):
            raise ValueError('Local planner dimensions, speeds and limits must be positive and finite')
        if self.radius <= self.half_width or not math.isfinite(self.latency) or self.latency < 0 or self.max_nodes < 1:
            raise ValueError('Invalid turn radius, latency or search limit')


def advance(pose, distance, curvature):
    x, y, yaw = pose
    turn = distance * curvature
    if abs(curvature) < 1e-8:
        return (x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw)
    return (x + (math.sin(yaw + turn) - math.sin(yaw)) / curvature,
            y + (math.cos(yaw) - math.cos(yaw + turn)) / curvature, wrap(yaw + turn))


class ScanSpace:
    """Observed free cells in the robot frame at scan receipt, including occlusion.
    A finite first return proves free space only before that return. Positive
    infinity proves free space to range_max; NaN/missing rays remain unknown. The current body interior is the sole exception.
    """
    def __init__(self, ranges, angle_min, angle_increment, range_min, range_max,
                 cfg, laser=(.29, 0., 0.)):
        self.cfg = cfg
        self.n = int(math.ceil(2 * cfg.extent / cfg.resolution)) + 1
        axis = (np.arange(self.n) - self.n // 2) * cfg.resolution
        x, y = np.meshgrid(axis, axis)
        lx, ly, lyaw = laser
        dx, dy = x - lx, y - ly
        angles = np.arctan2(dy, dx) - lyaw
        values = np.asarray(ranges, dtype=float)
        self.free = np.zeros((self.n, self.n), dtype=bool)
        if (values.ndim != 1 or len(values) < 3 or
                not all(math.isfinite(z) for z in (angle_min, angle_increment, range_min, range_max, *laser)) or
                angle_increment <= 0 or range_min < 0 or range_max <= range_min):
            return
        # Use periodic indexing only for a genuinely full revolution.
        full = len(values) * angle_increment >= 2 * math.pi - 1.5 * angle_increment
        relative = (angles - angle_min) % (2 * math.pi)
        index = np.rint(relative / angle_increment).astype(int)
        valid_index = np.ones_like(index, bool) if full else index < len(values)
        index = index % len(values) if full else np.clip(index, 0, len(values)-1)
        measured = values[index]
        distance = np.hypot(dx, dy)
        finite = (np.isfinite(measured) & (measured >= range_min) & (measured <= range_max)) | np.isposinf(measured)
        measured = np.where(np.isposinf(measured), range_max, measured)
        self.free = valid_index & finite & (distance < measured - cfg.resolution) & (distance >= range_min)
        self.free |= (x <= cfg.front) & (x >= -cfg.rear) & (abs(y) <= cfg.half_width)
        # Dense footprint includes interior, not just centre or leading edge.
        fx = np.linspace(-cfg.rear-cfg.margin, cfg.front+cfg.margin,
                         int(math.ceil((cfg.front+cfg.rear+2*cfg.margin)/(cfg.resolution/2)))+1)
        fy = np.linspace(-cfg.half_width-cfg.margin, cfg.half_width+cfg.margin,
                         int(math.ceil((2*cfg.half_width+2*cfg.margin)/(cfg.resolution/2)))+1)
        self.foot = np.array(np.meshgrid(fx, fy)).reshape(2, -1).T
        # Extra raster/scan-to-scan clearance for planned movement; the stopping
        # check uses the configured physical margin. Forward guard mirrors Brain.
        extra = .15
        fy = np.arange(-cfg.half_width-cfg.margin-extra, cfg.half_width+cfg.margin+extra+.025, .05)
        fx = np.arange(-cfg.rear-cfg.margin-extra, cfg.front+cfg.margin+extra+.025, .05)
        self.plan_foot = np.array(np.meshgrid(fx, fy)).reshape(2, -1).T
        gx = np.arange(cfg.front, cfg.front+.60, .05)
        gy = np.arange(-.4, .425, .05)
        self.plan_foot_forward = np.concatenate((self.plan_foot, np.array(np.meshgrid(gx,gy)).reshape(2,-1).T))

    def clear(self, poses, planning=False, forward=False):
        if not hasattr(self, 'foot'):
            return False
        poses = np.asarray(poses).reshape(-1, 3)
        foot = self.foot
        if planning:
            foot = self.plan_foot_forward if forward else self.plan_foot
        cs, sn = np.cos(poses[:, 2:3]), np.sin(poses[:, 2:3])
        x = poses[:, 0:1] + cs * foot[:, 0] - sn * foot[:, 1]
        y = poses[:, 1:2] + sn * foot[:, 0] + cs * foot[:, 1]
        ix = np.rint(x / self.cfg.resolution).astype(int) + self.n // 2
        iy = np.rint(y / self.cfg.resolution).astype(int) + self.n // 2
        if np.any((ix < 0) | (iy < 0) | (ix >= self.n) | (iy >= self.n)):
            return False
        return bool(self.free[iy, ix].all())

    def front_clear(self, pose):
        x = np.arange(self.cfg.front, self.cfg.front+.55, .1)
        y = np.arange(-.35, .36, .1)
        xx, yy = np.meshgrid(x, y)
        cs, sn = math.cos(pose[2]), math.sin(pose[2])
        ix = np.rint((pose[0]+cs*xx-sn*yy)/self.cfg.resolution).astype(int)+self.n//2
        iy = np.rint((pose[1]+sn*xx+cs*yy)/self.cfg.resolution).astype(int)+self.n//2
        return bool(np.all((ix>=0)&(iy>=0)&(ix<self.n)&(iy<self.n)) and self.free[iy,ix].all())

    def arc(self, pose, distance, curvature, planning=False):
        # Sample rotational sweep as well as translation (corner travels farther).
        corner = math.hypot(max(self.cfg.front, self.cfg.rear)+self.cfg.margin,
                            self.cfg.half_width+self.cfg.margin)
        n = max(1, math.ceil(abs(distance)*(1+abs(curvature)*corner)/(self.cfg.resolution/2)))
        samples = [advance(pose, float(d), curvature) for d in np.linspace(0, distance, n+1)]
        if not self.clear(samples):
            return None
        if planning:
            if not self.clear([samples[-1]], True, distance > 0):
                return None
            if distance > 0:
                # Brain vetoes forward motion when its current front corridor is blocked.
                # Check at each intermediate pose too; reversing remains available.
                for p in samples:
                    if not self.front_clear(p):
                        return None
        return samples[-1]


@dataclass
class Decision:
    v: float = 0.
    w: float = 0.
    state: str = 'waiting_scan'
    nodes: int = 0
    path: tuple = ()


class LocalPlanner:
    def __init__(self, cfg=None):
        self.cfg = cfg or LocalCfg()
        self.reset()

    def reset(self):
        self.history = []
        self.last_pose = None
        self.reverse_distance = 0.
        self.gear = 0
        self.command = (0., 0.)
        self.motion_anchor = None
        self.motion_time = None
        self.stalled = False
        self.mode = 'waiting_scan'
        self.commit = None
        self.avoid_started = None
        self.follow_anchor = None

    def _key(self, p):
        return (round(p[0]/.2), round(p[1]/.2), round(wrap(p[2])/(math.pi/12)))

    def decide(self, space, pose, goal, now, scan_age, measured_v=0., measured_w=0.,
               odom_age=0., world_pose=None, preferred=None, short_reverse_allowed=True):
        c = self.cfg
        world = tuple(world_pose if world_pose is not None else pose)
        if (not all(math.isfinite(x) for x in (*pose, *goal, *world, now, measured_v, measured_w, scan_age, odom_age)) or
                abs(measured_v) > 2. or abs(measured_w) > 6.):
            self.command = (0., 0.)
            return Decision(state='invalid_input')
        if space is None or scan_age < 0 or scan_age > c.scan_timeout:
            self.command = (0., 0.)
            return Decision(state='stale_scan')
        if odom_age < 0 or odom_age > c.odom_timeout:
            self.command = (0., 0.)
            return Decision(state='stale_odometry')
        if self.stalled:
            return Decision(state='stalled')
        if self.motion_anchor is None:
            self.motion_anchor, self.motion_time = world, now
        moved = math.dist(world[:2], self.motion_anchor[:2])
        if moved > .08 or abs(wrap(world[2]-self.motion_anchor[2])) > .1:
            self.motion_anchor, self.motion_time = world, now
        elif abs(self.command[0]) > .01 and now-self.motion_time > c.stall_s:
            self.stalled = True
            self.command = (0., 0.)
            return Decision(state='stalled')
        elif abs(self.command[0]) < .01:
            self.motion_time = now
        if self.last_pose is not None and self.gear < 0:
            self.reverse_distance += math.dist(world[:2], self.last_pose[:2])
        self.last_pose = world
        # Check swept braking footprint before authorizing any new acceleration.
        stop_d = abs(measured_v) * (scan_age+c.latency) + measured_v**2/(2*c.deceleration)
        curvature = measured_w/measured_v if abs(measured_v) > .02 else 0.
        if space.arc(pose, math.copysign(stop_d, measured_v), curvature) is None:
            self.command = (0., 0.)
            return Decision(state='braking_blocked')
        if math.dist(pose[:2], goal) < .25:
            self.command = (0., 0.)
            return Decision(state='local_goal')
        attempt_expired = self.avoid_started is not None and now-self.avoid_started > c.max_avoid_s
        reverse_ok = c.allow_reverse and c.max_reverse_m-self.reverse_distance > stop_d+.05
        # A nearby goal behind us should not trigger a forward U-turn merely
        # because that forward arc is clear. Check a reverse arc first.
        dx, dy = goal[0]-pose[0], goal[1]-pose[1]
        gx = math.cos(pose[2])*dx + math.sin(pose[2])*dy
        gy = -math.sin(pose[2])*dx + math.cos(pose[2])*dy
        distance = math.hypot(gx, gy)
        if not attempt_expired and short_reverse_allowed and reverse_ok and gx < 0 and distance <= c.short_reverse_m:
            k = 2*gy/(distance*distance)
            speed = c.reverse_speed
            reverse_stop = speed*(scan_age+c.latency) + speed**2/(2*c.deceleration)
            sweep = max(c.step, reverse_stop)
            if (abs(k) <= 1/c.radius and
                    sweep <= c.max_reverse_m-self.reverse_distance and
                    space.arc(pose, -sweep, k, planning=True) is not None):
                self.commit = None
                self.follow_anchor = None
                if self.avoid_started is None:
                    self.avoid_started = now
                return self._command(-speed, -speed*k, measured_v, measured_w, 'avoid', 0, ())
        # Normal road/lane following can pass through if its complete near-term sweep is known free.
        if preferred is not None:
            v, w = preferred
            if v > 0:
                v = min(v, c.cruise_speed)
                k = preferred[1]/preferred[0]
                # A short clear corridor is not evidence we have escaped a cul-de-sac.
                # Rejoining requires a longer checked sweep, otherwise we'd alternate
                # backing out and immediately driving back into the same dead end.
                horizon = min(math.dist(pose[:2], goal), 4.5) if self.mode == 'avoid' else 1.2
                # Validate braking at the REQUESTED speed before acceleration.
                target_stop = v*(scan_age+c.latency) + v*v/(2*c.deceleration)
                # A clear arc that circles around a nearby goal is not route
                # recovery. Let local search choose a manoeuvre that approaches it.
                progress_pose = advance(pose, min(c.step, distance), k)
                approaches = math.dist(progress_pose[:2], goal) < distance-.01
                if approaches and abs(k) <= 1/c.radius + 1e-6 and space.arc(pose, max(horizon, stop_d+.5, target_stop+.1), k, planning=True) is not None:
                    self.commit = None
                    self.avoid_started = None
                    if self.follow_anchor is None:
                        self.follow_anchor = world[:2]
                    elif math.dist(world[:2], self.follow_anchor) > 1.:
                        self.reverse_distance = 0.
                        self.history.clear()
                    return self._command(v, v*k, measured_v, measured_w, 'following', 0, ())
        if attempt_expired:
            self.command = (0., 0.)
            return Decision(state='escape_timeout')
        self.follow_anchor = None
        if self.avoid_started is None:
            self.avoid_started = now
        # Execute only a short piece, but revalidate it against every new scan.
        # This prevents 10 Hz left/right dithering and bounds CPU on the Pi.
        if self.commit is not None:
            origin, direction, k = self.commit
            travelled = math.dist(world[:2], origin)
            if travelled < .25 and (direction > 0 or reverse_ok):
                distance = direction * max(c.step, .35-travelled)
                if space.arc(pose, distance, k, planning=True) is not None:
                    speed = c.forward_speed if direction > 0 else -c.reverse_speed
                    return self._command(speed, speed*k, measured_v, measured_w, 'avoid', 0, ())
            self.commit = None
        start = tuple(pose)
        initial_h = math.dist(start[:2], goal)
        heap = [(initial_h, 0, start, 0., None, ())]
        costs = {self._key(start): 0.}
        counter = 0
        best = None
        best_score = math.inf
        deadline = time.monotonic() + c.budget_s
        directions = (1, -1) if reverse_ok else (1,)
        while heap and counter < c.max_nodes and time.monotonic() < deadline:
            _, _, p, cost, first, path = heapq.heappop(heap)
            if cost > costs.get(self._key(p), math.inf) + 1e-8:
                continue
            distance_goal = math.dist(p[:2], goal)
            if first is not None:
                # Revisited endpoints are discouraged so a dead end can lead to backing out.
                # History is world based, converted to scan-frame distances using relative displacement.
                displacement = np.array(p[:2])-np.array(start[:2])
                rotation = world[2]-start[2]
                endpoint_world = np.array(world[:2]) + np.array([
                    math.cos(rotation)*displacement[0]-math.sin(rotation)*displacement[1],
                    math.sin(rotation)*displacement[0]+math.cos(rotation)*displacement[1]])
                visits = sum(max(0., 1.-math.dist(endpoint_world, q)/.7) for q in self.history)
                score = distance_goal + .12*cost + 1.2*visits
                if score < best_score:
                    best_score, best = score, (first, path)
                if distance_goal < .35:
                    best = (first, path)
                    break
            for direction in directions:
                for k in (0., -1/c.radius, 1/c.radius):
                    q = space.arc(p, direction*c.step, k, planning=True)
                    if q is None:
                        continue
                    new_cost = cost + c.step*(1. if direction > 0 else 1.35)
                    if path and direction != path[-1][3]:
                        new_cost += .5
                    key = self._key(q)
                    if new_cost >= costs.get(key, math.inf):
                        continue
                    costs[key] = new_cost
                    counter += 1
                    first_action = first if first is not None else (direction, k)
                    new_path = path + ((*q, direction),)
                    heapq.heappush(heap, (new_cost+math.dist(q[:2], goal), counter, q, new_cost, first_action, new_path))
        if best is None:
            self.command = (0., 0.)
            return Decision(state='blocked', nodes=counter)
        direction, k = best[0]
        speed = c.forward_speed if direction > 0 else -c.reverse_speed
        # Keep search speed conservative and ensure a stop can fit inside the checked first primitive.
        d = abs(speed)*(scan_age+c.latency) + speed**2/(2*c.deceleration)
        if space.arc(start, direction*max(c.step, d), k, planning=True) is None:
            self.command = (0., 0.)
            return Decision(state='blocked', nodes=counter)
        if not self.history or math.dist(world[:2], self.history[-1]) > .15:
            self.history.append(world[:2])
            self.history = self.history[-120:]
        self.commit = (world[:2], direction, k)
        return self._command(speed, speed*k, measured_v, measured_w, 'avoid', counter, best[1])

    def _command(self, v, w, measured_v, measured_w, mode, nodes, path):
        direction = 1 if v > 0 else -1
        opposite_motion = v*measured_v < -.001
        rotating_at_rest = abs(measured_v) < .03 and abs(measured_w) > .05
        if opposite_motion or rotating_at_rest or (self.gear and direction != self.gear and
                                                  (abs(measured_v) > .03 or abs(measured_w) > .05)):
            self.command = (0., 0.)
            self.mode = mode
            return Decision(state='gear_stop', nodes=nodes)
        self.gear = direction
        self.command = (v, w)
        self.mode = mode
        return Decision(v, w, mode, nodes, tuple(path))
