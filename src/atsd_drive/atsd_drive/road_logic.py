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
    """Граф дорог: узлы — перекрёстки, повороты и точки заказа; рёбра — отрезки дороги."""

    def __init__(self, nodes=None, edges=None):
        self.nodes = {k: (float(v[0]), float(v[1])) for k, v in (nodes or {}).items()}
        self.adj = {k: [] for k in self.nodes}
        for a, b in edges or []:
            if a in self.nodes and b in self.nodes:
                self.adj[a].append(b)
                self.adj[b].append(a)

    def nearest(self, p):
        return min(self.nodes, key=lambda k: math.dist(self.nodes[k], p), default=None)

    def route(self, start, goal):
        """Ломаная от start до goal по дорогам. Без графа — прямая."""
        s, g = self.nearest(start), self.nearest(goal)
        if s is None:
            return [tuple(start), tuple(goal)]
        dist, prev, todo = {s: 0.0}, {}, [(0.0, s)]
        while todo:
            d, u = heapq.heappop(todo)
            if u == g:
                break
            if d > dist[u]:
                continue
            for v in self.adj[u]:
                nd = d + math.dist(self.nodes[u], self.nodes[v])
                if nd < dist.get(v, math.inf):
                    dist[v], prev[v] = nd, u
                    heapq.heappush(todo, (nd, v))
        if g not in dist:
            return None
        ids = [g]
        while ids[-1] != s:
            ids.append(prev[ids[-1]])
        pts = [self.nodes[k] for k in reversed(ids)]
        # робот уже за первым узлом по ходу (между ним и следующим) или стоит на нём — узел не нужен
        if len(pts) > 1 and (math.dist(start, pts[0]) < 1.0 or
                             math.dist(start, pts[1]) < math.dist(pts[0], pts[1])):
            pts = pts[1:]
        if math.dist(pts[-1], goal) > 0.5:
            pts.append(tuple(goal))
        return [tuple(start)] + pts


GATE_2D = 13.8  # хи-квадрат, 2 степени свободы, 99.9 %


class PoseFilter:
    """Фильтр Калмана по (x, y, курс). Колёса — прогноз; GPS, курс по доплеру и разметка — поправки.
    GPS проверяется на правдоподобие: при глушении/подмене фильтр ему не верит и едет по колёсам."""

    def __init__(self, scale_err=0.02, yaw_err=0.05):
        self.X = None
        self.P = None
        self.scale_err = scale_err   # ошибка пути по колёсам, доля
        self.yaw_err = yaw_err       # ошибка поворота по колёсам, доля
        self.gnss = 'none'           # none | ok | suspect
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

    def gnss_fix(self, x, y, sigma):
        """Новый фикс. Возвращает состояние GPS: ok — принят, suspect — не верим."""
        z = np.array([x, y])
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

    def gnss_lost(self):
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
    v_max: float = 0.5
    w_max: float = 1.0
    lane_look: float = 1.5      # точка прицеливания на полосе, м
    route_look: float = 2.5     # точка прицеливания на маршруте, м
    switch_m: float = 2.0       # узел пройден, если ближе
    junction_m: float = 4.0     # у перекрёстка разметке не верим, рулим по маршруту
    arrive_m: float = 0.3       # до точки вдоль дороги, м
    lane_align_deg: float = 30.0
    turn_deg: float = 60.0      # дальше — разворот на месте
    creep_v: float = 0.3        # курса нет и разметки нет: медленно прямо, пока GPS не даст курс


class Follower:
    """Ведёт по ломаной: на дороге — по разметке, на перекрёстках и без разметки — по маршруту."""

    def __init__(self, cfg: FollowCfg = FollowCfg()):
        self.cfg = cfg
        self.path = []
        self.i = 0
        self.mode = 'idle'

    def start(self, path):
        self.path, self.i, self.mode = [tuple(p) for p in path], 0, 'start'

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

    def step(self, pose, yaw_known, lane):
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
        # приехал: на последнем отрезке до точки вдоль дороги меньше arrive_m. Вдоль, а не по прямой:
        # постоянный сдвиг GPS вбок на пару метров не должен мешать остановиться
        side = abs(-math.sin(bearing) * (p[0] - p0[0]) + math.cos(bearing) * (p[1] - p0[1]))
        if math.dist(p, self.path[-1]) < c.arrive_m or (last_seg and self.remain(p) < c.arrive_m and side < 3.0):
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
                self.mode = 'turn'
                return 0.0, math.copysign(0.6, ang), False
            k = 2 * yr / (xr * xr + yr * yr)
        else:
            self.mode = 'creep'
            return c.creep_v, 0.0, False

        v = c.v_max / (1 + 1.5 * abs(k))
        v = min(v, max(0.15, 0.4 * math.dist(p, self.path[-1])))
        w = max(-c.w_max, min(c.w_max, v * k))
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

    def tick(self, lane):
        """10 раз в секунду: -> (v, w, приехал?)."""
        f = self.filter
        if not self.active or not f.ready:
            return 0.0, 0.0, False
        fresh = lane is not None
        if fresh:
            self.last_lane, self.lane_age = lane, 0
        elif self.lane_age < 5:  # разметка пропала на миг (блик, пешеход) — держим последнюю, до 0,5 с
            lane, self.lane_age = self.last_lane, self.lane_age + 1
        v, w, done = self.follower.step(f.X, f.yaw_known, lane)
        if fresh and self.follower.mode == 'lane' and f.yaw_known:
            # разметка точнее GPS поперёк дороги и по курсу: робот на оси полосы, полоса вдоль дороги
            p0, bearing = self.follower.segment()
            f.heading(bearing - lane['angle'], math.radians(3))
            f.cross_track(p0, bearing, -lane['offset'], 0.5)
        if done:
            self.active = False
        return v, w, done
