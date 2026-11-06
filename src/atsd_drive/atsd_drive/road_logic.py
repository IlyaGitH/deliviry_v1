# Езда по дороге: где робот (GPS + колёса + разметка), куда ехать (граф дорог), как рулить.
# Система координат карты: x на восток, y на север, метры от origin; курс 0 — восток, против часовой.
import heapq
import math
from dataclasses import dataclass

import numpy as np

EARTH_R = 6378137.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def to_local(lat, lon, origin):
    """Широта/долгота -> метры от origin. Плоская Земля: на 5 км ошибка меньше сантиметра."""
    lat0, lon0 = origin
    return (math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0)),
            math.radians(lat - lat0) * EARTH_R)


def to_geo(x, y, origin):
    lat0, lon0 = origin
    return (lat0 + math.degrees(y / EARTH_R),
            lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0)))))


class Roads:
    """Road geometry with bounded endpoint snapping and optional one-way segments."""

    def __init__(self, nodes=None, edges=None, *, directed_edges=None, allow_direct=False, snap_m=3.0):
        self.nodes = {str(k): (float(v[0]), float(v[1])) for k, v in (nodes or {}).items()}
        if not all(math.isfinite(x) for p in self.nodes.values() for x in p):
            raise ValueError('Non-finite road coordinate')
        self.allow_direct, self.snap_m = allow_direct, float(snap_m)
        if not math.isfinite(self.snap_m) or not .1 <= self.snap_m <= 5.:
            raise ValueError('Road snapping distance must be between 0.1 and 5 m')
        self.edges = []
        seen = set()
        for directed, rows in ((False, edges or []), (True, directed_edges or [])):
            for a, b in rows:
                a, b = str(a), str(b)
                if a not in self.nodes or b not in self.nodes:
                    raise ValueError('Road edge references a missing node')
                if a == b or math.dist(self.nodes[a], self.nodes[b]) < 1e-6:
                    continue
                key = (a, b, directed) if directed else (*sorted((a, b)), False)
                if key not in seen:
                    seen.add(key)
                    self.edges.append((a, b, directed))

    @classmethod
    def from_data(cls, data):
        if not isinstance(data, dict):
            raise ValueError('Expected a road graph object')
        return cls(data.get('nodes'), data.get('edges'), directed_edges=data.get('directed_edges'),
                   snap_m=data.get('snap_m', 3.0))

    def nearest(self, p):
        return min(self.nodes, key=lambda k: math.dist(self.nodes[k], p), default=None)

    def snap(self, p):
        best = None
        for i, (a, b, _) in enumerate(self.edges):
            ax, ay = self.nodes[a]; bx, by = self.nodes[b]
            dx, dy = bx-ax, by-ay
            t = max(0., min(1., ((p[0]-ax)*dx+(p[1]-ay)*dy)/(dx*dx+dy*dy)))
            q = (ax+t*dx, ay+t*dy)
            d = math.dist(p, q)
            if best is None or d < best[0]:
                best = (d, i, t, q)
        return best if best is not None and best[0] <= self.snap_m else None

    def route(self, start, goal):
        """Split the closest road segments at start/goal; never skip road bends."""
        if not all(math.isfinite(x) for x in (*start, *goal)):
            return None
        if not self.edges:
            return [tuple(start), tuple(goal)] if self.allow_direct else None
        ss, gg = self.snap(start), self.snap(goal)
        if ss is None or gg is None:
            return None
        # Object keys cannot collide with IDs supplied by a map or an operator.
        source, target = object(), object()
        coords = {**self.nodes, source: ss[3], target: gg[3]}
        adj = {k: [] for k in coords}
        for i, (a, b, directed) in enumerate(self.edges):
            chain = [(0., a), (1., b)]
            if i == ss[1]: chain.append((ss[2], source))
            if i == gg[1]: chain.append((gg[2], target))
            chain.sort(key=lambda item: item[0])
            for (_, u), (_, v) in zip(chain, chain[1:]):
                d = math.dist(coords[u], coords[v])
                adj[u].append((v, d))
                if not directed or d < 1e-8:
                    adj[v].append((u, d))
        dist, prev, queue = {source: 0.}, {}, [(0., 0, source)]
        counter = 0
        while queue:
            d, _, u = heapq.heappop(queue)
            if d > dist[u]: continue
            if u is target: break
            for v, length in adj[u]:
                nd = d+length
                if nd < dist.get(v, math.inf):
                    dist[v], prev[v] = nd, u
                    counter += 1
                    heapq.heappush(queue, (nd, counter, v))
        if target not in dist:
            return None
        ids = [target]
        while ids[-1] is not source:
            ids.append(prev[ids[-1]])
        path = [tuple(start)]
        for q in [coords[k] for k in reversed(ids)] + [tuple(goal)]:
            if math.dist(path[-1], q) > 1e-6:
                path.append(q)
        return path if len(path) > 1 else [tuple(start), tuple(goal)]


GATE_2D = 13.8  # хи-квадрат, 2 степени свободы, 99.9 %


class PoseFilter:
    """Фильтр Калмана по (x, y, курс). Колёса — прогноз; GPS, курс по доплеру и разметка — поправки.
    GPS проверяется на правдоподобие: при глушении/подмене фильтр ему не верит и едет по колёсам."""

    def __init__(self, scale_err=0.02, yaw_err=0.05):
        self.X = None
        self.P = None
        self.scale_err = scale_err   # ошибка пути по колёсам, доля
        self.yaw_err = yaw_err       # ошибка поворота по колёсам, доля
        self.manual = False
        self.gnss = 'none'           # none | ok | suspect | manual
        self.rejects = 0
        self.consistent = 0
        self.last_fix = None
        self.odo_since_fix = np.zeros(2)

    @property
    def ready(self):
        return self.X is not None

    @property
    def yaw_known(self):
        return self.ready and self.P[2, 2] < math.radians(15) ** 2

    def reset(self, x, y, yaw=None, sig_xy=1.0, sig_yaw=math.radians(5)):
        self.X = np.array([x, y, yaw or 0.0])
        self.P = np.diag([sig_xy ** 2, sig_xy ** 2, sig_yaw ** 2 if yaw is not None else math.pi ** 2])

    def set_manual(self, x, y, yaw):
        # Operator coordinates are authoritative; motion still comes from odometry/IMU.
        self.reset(x, y, yaw, 0.5, math.radians(5))
        self.manual = True
        self.gnss = 'manual'
        self.last_fix = None
        self.rejects = self.consistent = 0
        self.odo_since_fix = np.zeros(2)

    def predict(self, ds, dyaw):
        if not self.ready:
            return
        x, y, th = self.X
        mid = th + 0.5 * dyaw
        c, s = math.cos(mid), math.sin(mid)
        self.X = np.array([x + ds * c, y + ds * s, wrap(th + dyaw)])
        F = np.array([[1, 0, -ds * s], [0, 1, ds * c], [0, 0, 1]])
        q_along = (self.scale_err * abs(ds) + 0.002) ** 2
        q_yaw = (self.yaw_err * abs(dyaw) + 0.002 * abs(ds) + 1e-4) ** 2
        Q = np.diag([q_along * c * c + 1e-6, q_along * s * s + 1e-6, q_yaw])
        self.P = F @ self.P @ F.T + Q
        self.odo_since_fix += (ds * c, ds * s)

    def _update(self, z, h, H, R, gate=None):
        nu = z - h
        if H.shape[0] == 1 and H[0, 2]:
            nu = np.array([wrap(nu[0])])
        S = H @ self.P @ H.T + R
        d2 = float(nu @ np.linalg.solve(S, nu))
        if gate is not None and d2 > gate:
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.X = self.X + K @ nu
        self.X[2] = wrap(self.X[2])
        self.P = (np.eye(3) - K @ H) @ self.P
        return True

    def gnss_fix(self, x, y, sigma, wifi=None):
        """Новый фикс. Возвращает состояние GPS: ok — принят, suspect — не верим.
        wifi — (x, y, точность м) по Wi-Fi: грубо, но подменой GPS не сбить; фикс дальше
        трёх её точностей (не ближе 100 м) — подмена."""
        if self.manual:
            return self.gnss
        z = np.array([x, y])
        if wifi is not None and np.linalg.norm(z - wifi[:2]) > max(100.0, 3.0 * wifi[2]):
            self.gnss, self.consistent, self.last_fix = 'suspect', 0, None
            return self.gnss
        if not self.ready:
            self.reset(x, y, None, sig_xy=sigma)
            self.gnss, self.last_fix = 'ok', z
            return self.gnss
        # движение по GPS между фиксами должно совпадать с движением по колёсам
        # (пока курса нет — сравниваем только длину пути)
        tol = 1.5 + 0.5 * sigma
        if self.last_fix is None:
            moved_ok = True
        elif self.yaw_known:
            moved_ok = np.linalg.norm((z - self.last_fix) - self.odo_since_fix) < tol
        else:
            moved_ok = abs(np.linalg.norm(z - self.last_fix) - np.linalg.norm(self.odo_since_fix)) < tol
        self.last_fix, self.odo_since_fix = z, np.zeros(2)
        H = np.array([[1.0, 0, 0], [0, 1.0, 0]])
        R = np.eye(2) * sigma ** 2
        if self.gnss == 'suspect':
            # после скачка верим снова, только если фиксы долго согласны с колёсами и недалеко от нас
            self.consistent = self.consistent + 1 if moved_ok else 0
            if self.consistent >= 10 and np.linalg.norm(z - self.X[:2]) < 30.0:
                self.X[:2] = z
                self.P[:2, :2] = np.eye(2) * sigma ** 2
                self.gnss, self.rejects = 'ok', 0
            return self.gnss
        if moved_ok and self._update(z, self.X[:2], H, R, GATE_2D):
            self.gnss, self.rejects = 'ok', 0
        else:
            self.rejects += 1
            if self.rejects >= 3:
                self.gnss, self.consistent = 'suspect', 0
        return self.gnss

    def wifi_fix(self, x, y, sigma):
        """Место по Wi-Fi (±десятки метров). Без GPS и «Робот здесь» — старт отсюда; в пути поправляет,
        только если сами знаем место хуже, чем Wi-Fi: его ошибка — постоянный сдвиг для этих
        роутеров, повторами она не уменьшается, и верить ему точнее его точности нельзя."""
        if self.manual:
            return False
        if not self.ready:
            self.reset(x, y, None, sig_xy=sigma)
            return True
        if math.sqrt(max(self.P[0, 0], self.P[1, 1])) < sigma:
            return False
        H = np.array([[1.0, 0, 0], [0, 1.0, 0]])
        return self._update(np.array([x, y]), self.X[:2], H, np.eye(2) * sigma ** 2, GATE_2D)

    def gnss_lost(self):
        if self.manual:
            return
        if self.gnss != 'suspect':
            self.gnss = 'none'
        self.last_fix = None

    def heading(self, yaw, sigma):
        """Курс по доплеру GPS или по разметке и направлению дороги."""
        if not self.ready:
            return False
        if not self.yaw_known:  # курса ещё нет — принимаем как есть
            self.X[2], self.P[2, 2] = wrap(yaw), sigma ** 2
            self.P[:2, 2] = self.P[2, :2] = 0.0
            return True
        return self._update(np.array([yaw]), np.array([self.X[2]]), np.array([[0, 0, 1.0]]),
                            np.array([[sigma ** 2]]), 9.0)

    def cross_track(self, p0, bearing, offset, sigma):
        """Робот на offset м левее линии дороги (по разметке). Держит оценку на дороге без GPS."""
        if not self.ready:
            return False
        n = np.array([-math.sin(bearing), math.cos(bearing)])
        h = float(n @ (self.X[:2] - np.asarray(p0)))
        return self._update(np.array([offset]), np.array([h]), np.array([[n[0], n[1], 0.0]]),
                            np.array([[sigma ** 2]]), 9.0)


@dataclass
class FollowCfg:
    v_max: float = 0.8          # едем ровно с этой скоростью; меньше — только в крутом повороте и у точки
    w_max: float = 1.0
    w_smooth: float = 0.5       # сглаживание руля: доля нового значения за такт 0,1 с
    lane_look: float = 1.5      # точка прицеливания на полосе, м
    route_look: float = 2.5     # точка прицеливания на маршруте, м
    switch_m: float = 2.0       # узел пройден, если ближе
    junction_m: float = 4.0     # у перекрёстка разметке не верим, рулим по маршруту
    arrive_m: float = 0.3       # до точки вдоль дороги, м
    lane_align_deg: float = 30.0
    turn_deg: float = 60.0      # дальше — медленная дуга
    turn_radius: float = 1.0    # минимум радиуса, оба борта движутся вперёд
    turn_speed: float = 0.25
    creep_v: float = 0.3        # курса нет и разметки нет: медленно прямо, пока GPS не даст курс


class Follower:
    """Ведёт по ломаной: на дороге — по разметке, на перекрёстках и без разметки — по маршруту."""

    def __init__(self, cfg: FollowCfg = FollowCfg()):
        self.cfg = cfg
        self.path = []
        self.i = 0
        self.mode = 'idle'
        self.w_prev = 0.0

    def start(self, path):
        self.path, self.i, self.mode = [tuple(p) for p in path], 0, 'start'
        self.w_prev = 0.0

    def segment(self):
        a, b = self.path[self.i], self.path[self.i + 1]
        return a, math.atan2(b[1] - a[1], b[0] - a[0])

    def _project(self, p):
        a, b = np.asarray(self.path[self.i]), np.asarray(self.path[self.i + 1])
        d = b - a
        L2 = float(d @ d) or 1e-9
        return float(np.clip((np.asarray(p) - a) @ d / L2, 0.0, 1.0)), math.sqrt(L2)

    def remain(self, p):
        if not self.path:
            return 0.0
        t, L = self._project(p)
        r = (1 - t) * L
        for k in range(self.i + 1, len(self.path) - 1):
            r += math.dist(self.path[k], self.path[k + 1])
        return r

    def _lookahead(self, p, dist):
        t, L = self._project(p)
        i, left = self.i, dist + t * L
        while i < len(self.path) - 1:
            seg = math.dist(self.path[i], self.path[i + 1])
            if left <= seg:
                a, b = np.asarray(self.path[i]), np.asarray(self.path[i + 1])
                return a + (b - a) * (left / seg if seg else 0)
            left -= seg
            i += 1
        return np.asarray(self.path[-1])

    def step(self, pose, yaw_known, lane, strict_arrival=False):
        """pose (x, y, курс), lane — словарь от камеры или None. -> (v, w, приехал?)."""
        c = self.cfg
        p = pose[:2]
        while self.i < len(self.path) - 2:
            t, _ = self._project(p)
            if t >= 1.0 or math.dist(p, self.path[self.i + 1]) < c.switch_m:
                self.i += 1
            else:
                break
        p0, bearing = self.segment()
        nxt = self.path[self.i + 1]
        last_seg = self.i == len(self.path) - 2
        # A detour can pass the goal's projection while metres off the road.
        # nav_node uses strict arrival: proximity to the actual destination.
        side = abs(-math.sin(bearing)*(p[0]-p0[0]) + math.cos(bearing)*(p[1]-p0[1]))
        projected_arrival = not strict_arrival and last_seg and self.remain(p) < c.arrive_m and side < 3.0
        if math.dist(p, self.path[-1]) < c.arrive_m or projected_arrival:
            self.mode = 'arrived'
            return 0.0, 0.0, True
        at_junction = (not last_seg and math.dist(p, nxt) < c.junction_m) or \
            (self.i > 0 and math.dist(p, self.path[self.i]) < c.junction_m * 0.5)
        lane_ok = lane is not None and not at_junction and \
            (not yaw_known or abs(wrap(pose[2] + lane['angle'] - bearing)) < math.radians(c.lane_align_deg))

        if lane_ok:
            self.mode = 'lane'
            L = c.lane_look
            yt = lane['offset'] + math.tan(lane['angle']) * L
            k = 2 * yt / (L * L + yt * yt)
        elif yaw_known:
            self.mode = 'route'
            tx, ty = self._lookahead(p, c.route_look) - np.asarray(p)
            cs, sn = math.cos(pose[2]), math.sin(pose[2])
            xr, yr = cs * tx + sn * ty, -sn * tx + cs * ty
            ang = math.atan2(yr, xr)
            if abs(ang) > math.radians(c.turn_deg):
                self.mode = 'arc'
                v = min(c.turn_speed, c.v_max, c.w_max * c.turn_radius)
                w = math.copysign(v / c.turn_radius, ang)
                self.w_prev = w
                return v, w, False
            k = 2 * yr / (xr * xr + yr * yr)
        else:
            self.mode = 'creep'
            return c.creep_v, 0.0, False

        k = float(np.clip(k, -1.0 / c.turn_radius, 1.0 / c.turn_radius))

        # скорость постоянная: дрожание оценки места не должно дёргать газ. Сбавляем, только если
        # поворот круче, чем позволяет w_max, и на последних метрах — чтобы встать на точку
        v = min(c.v_max, c.w_max / abs(k)) if k else c.v_max
        v = min(v, max(0.15, 0.4 * math.dist(p, self.path[-1])))
        w = max(-c.w_max, min(c.w_max, v * k))
        w = self.w_prev + c.w_smooth * (w - self.w_prev)
        w = float(np.clip(w, -v / c.turn_radius, v / c.turn_radius))
        self.w_prev = w
        return v, w, False


class RoadNav:
    """Связка для узла ROS и симуляции: фильтр положения + граф дорог + ведение по маршруту."""

    def __init__(self, roads: Roads, cfg: FollowCfg = FollowCfg()):
        self.roads = roads
        self.filter = PoseFilter()
        self.follower = Follower(cfg)
        self.active = False
        self.last_lane, self.lane_age = None, 0

    def plan(self, goal):
        if not self.filter.ready:
            return None
        path = self.roads.route(tuple(self.filter.X[:2]), tuple(goal))
        if path:
            self.follower.start(path)
            self.active = True
        return path

    def stop(self):
        self.active = False

    def course(self, yaw, speed, turning):
        """Курс по доплеру GPS: только на ходу, по прямой и когда GPS в порядке."""
        if self.filter.gnss == 'ok' and speed > 0.3 and not turning:
            self.filter.heading(yaw, max(math.radians(3), 0.05 / speed))

    def tick(self, lane, correct_lane=True, strict_arrival=False):
        """10 раз в секунду: -> (v, w, приехал?)."""
        f = self.filter
        if not self.active or not f.ready:
            return 0.0, 0.0, False
        if not correct_lane:
            lane, self.last_lane, self.lane_age = None, None, 5
        fresh = lane is not None
        if fresh:
            self.last_lane, self.lane_age = lane, 0
        elif self.lane_age < 5:  # разметка пропала на миг (блик, пешеход) — держим последнюю, до 0,5 с
            lane, self.lane_age = self.last_lane, self.lane_age + 1
        v, w, done = self.follower.step(f.X, f.yaw_known, lane, strict_arrival=strict_arrival)
        if correct_lane and fresh and not f.manual and self.follower.mode == 'lane' and f.yaw_known:
            # разметка точнее GPS поперёк дороги и по курсу: робот на оси полосы, полоса вдоль дороги
            p0, bearing = self.follower.segment()
            f.heading(bearing - lane['angle'], math.radians(3))
            f.cross_track(p0, bearing, -lane['offset'], 0.5)
        if done:
            self.active = False
        return v, w, done
