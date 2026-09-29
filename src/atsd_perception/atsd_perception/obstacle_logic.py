from dataclasses import dataclass

import numpy as np


@dataclass
class CorridorConfig:
    front_x: float = 0.35
    half_width: float = 0.25
    margin: float = 0.10
    look_ahead: float = 2.5
    detour_offset: float = 0.65
    side_extra: float = 1.2
    min_points: int = 3
    cluster_gap: float = 0.15
    default_side: str = 'L'


def scan_to_xy(ranges, angle_min, angle_inc, range_min, range_max,
               laser_x=0.0, laser_y=0.0, laser_yaw=0.0):
    r = np.asarray(ranges, dtype=np.float32)
    ang = angle_min + angle_inc * np.arange(r.size, dtype=np.float32) + laser_yaw
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, ang = r[ok], ang[ok]
    return np.stack([laser_x + r * np.cos(ang), laser_y + r * np.sin(ang)], axis=1)


def _nearest_cluster_dist(xs: np.ndarray, cfg: CorridorConfig):
    if xs.size < cfg.min_points:
        return None
    xs = np.sort(xs)
    for i in range(xs.size - cfg.min_points + 1):
        if xs[i + cfg.min_points - 1] - xs[i] <= cfg.cluster_gap:
            return float(xs[i])
    return None


def analyze(points: np.ndarray, cfg: CorridorConfig):
    if points is None or len(points) == 0:
        return None, '?', 0.0
    x, y = points[:, 0], points[:, 1]
    half = cfg.half_width + cfg.margin

    ahead = (x > cfg.front_x) & (x < cfg.front_x + cfg.look_ahead) & (np.abs(y) < half)
    near_x = _nearest_cluster_dist(x[ahead], cfg)
    if near_x is None:
        return None, '?', 0.0
    dist = max(0.0, near_x - cfg.front_x)

    body = ahead & (x < near_x + 1.2)
    depth = float(np.max(x[body]) - near_x) if np.any(body) else 0.0
    depth = depth if depth > 0.2 else 0.0

    x_lo = -cfg.front_x
    x_hi = near_x + max(depth, 0.5) + cfg.side_extra

    def side_free(sign):
        cy = sign * cfg.detour_offset
        band = (x > x_lo) & (x < x_hi) & (np.abs(y - cy) < half)
        return int(np.count_nonzero(band)) < cfg.min_points

    left, right = side_free(+1), side_free(-1)
    if left and right:
        side = cfg.default_side
    elif left:
        side = 'L'
    elif right:
        side = 'R'
    else:
        side = 'B'
    return dist, side, depth


class Debounce:

    def __init__(self, on_count=2, off_count=3):
        self.on_count, self.off_count = on_count, off_count
        self.hits = self.miss = 0
        self.active = False

    def update(self, present: bool) -> bool:
        if present:
            self.hits += 1
            self.miss = 0
            if self.hits >= self.on_count:
                self.active = True
        else:
            self.miss += 1
            self.hits = 0
            if self.miss >= self.off_count:
                self.active = False
        return self.active
