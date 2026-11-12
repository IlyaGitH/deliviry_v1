# Дорожная разметка по камере: кадр -> вид сверху -> светлые полосы -> прямые -> середина полосы.
# Координаты робота: x вперёд, y влево, метры от центра робота.
from dataclasses import dataclass

import numpy as np

from atsd_perception.vision_logic import CameraGeom, ground_to_pixel

try:
    import cv2
except Exception:
    cv2 = None


@dataclass
class LaneCfg:
    x_min: float = 0.9        # ближе камера землю не видит
    x_max: float = 6.0
    y_max: float = 3.0        # смотрим ±3 м вбок
    res: float = 0.02         # м на пиксель вида сверху
    tophat_m: float = 0.40    # линия уже этого — разметка, шире — блик/лужа/белая машина
    thresh: int = 35          # насколько линия ярче асфальта рядом (0–255)
    inlier_m: float = 0.10
    min_points: int = 40
    min_span_m: float = 1.0   # прямая должна тянуться хотя бы на метр вперёд
    max_angle_deg: float = 40.0
    lane_width: float = 3.5   # ожидаемая ширина для проверки пары, не измерение
    follow: str = 'center'    # center — середина полосы; right/left — держаться у линии
    keep_m: float = 0.6       # для right/left: на сколько от линии держать центр робота
    iters: int = 80
    clutter_max: float = 0.25  # «краской» оказалось больше этой доли вида сверху — не дорога (окна, жалюзи, плитка)


class BirdView:
    """Гомография кадр -> вид сверху. Строится один раз на геометрию камеры."""

    def __init__(self, cam: CameraGeom, cfg: LaneCfg):
        self.cfg = cfg
        self.w = int(round(2 * cfg.y_max / cfg.res))
        self.h = int(round((cfg.x_max - cfg.x_min) / cfg.res))
        ground = [(cfg.x_max, cfg.y_max), (cfg.x_max, -cfg.y_max),
                  (cfg.x_min, -cfg.y_max), (cfg.x_min, cfg.y_max)]
        src = np.float32([ground_to_pixel(gx, gy, cam) for gx, gy in ground])
        dst = np.float32([self.to_px(gx, gy) for gx, gy in ground])
        self.H = cv2.getPerspectiveTransform(src, dst)
        self.size = (int(round(2 * cam.cx)), int(round(2 * cam.cy)))

    def to_px(self, gx, gy):
        return (self.cfg.y_max - gy) / self.cfg.res, (self.cfg.x_max - gx) / self.cfg.res

    def to_ground(self, col, row):
        return self.cfg.x_max - row * self.cfg.res, self.cfg.y_max - col * self.cfg.res

    def warp(self, gray):
        top = cv2.warpPerspective(gray, self.H, (self.w, self.h), flags=cv2.INTER_LINEAR)
        valid = cv2.warpPerspective(np.full(gray.shape, 255, np.uint8), self.H, (self.w, self.h),
                                    flags=cv2.INTER_NEAREST)
        return top, valid


def marking_points(bgr, bird: BirdView):
    """Точки разметки в метрах: светлые узкие полосы на виде сверху (белая и жёлтая краска)."""
    cfg = bird.cfg
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, value = cv2.split(hsv)
    paint = ((s < 85) | ((h >= 15) & (h <= 40) & (s >= 60))) & (value >= 90)
    v = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    top, valid = bird.warp(v)
    k = max(3, int(round(cfg.tophat_m / cfg.res)) | 1)
    hat = cv2.morphologyEx(top, cv2.MORPH_TOPHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (k, 1)))
    # у края кадра top-hat видит ложную «полосу» — край обрезаем
    valid = cv2.erode(valid, np.ones((1, k), np.uint8))
    paint_top, _ = bird.warp(paint.astype(np.uint8)*255)
    rows, cols = np.nonzero((hat > cfg.thresh) & (valid > 0) & (paint_top > 127))
    gx, gy = bird.to_ground(cols.astype(np.float32), rows.astype(np.float32))
    return np.stack([gx, gy], axis=1), len(rows) / max(1, np.count_nonzero(valid))


def fit_lines(pts, cfg: LaneCfg, rng=None):
    """До трёх прямых y = a + b·x методом RANSAC. Возвращает [(a, b, число точек, точки)]."""
    rng = rng or np.random.default_rng(0)
    if len(pts) > 3000:
        pts = pts[rng.choice(len(pts), 3000, replace=False)]
    lines = []
    max_b = np.tan(np.radians(cfg.max_angle_deg))
    rest = pts
    for _ in range(3):
        if len(rest) < cfg.min_points:
            break
        best = None
        for _ in range(cfg.iters):
            i, j = rng.choice(len(rest), 2, replace=False)
            (x1, y1), (x2, y2) = rest[i], rest[j]
            if abs(x2 - x1) < 0.3:
                continue
            b = (y2 - y1) / (x2 - x1)
            if abs(b) > max_b:
                continue
            a = y1 - b * x1
            inl = np.abs(rest[:, 1] - (a + b * rest[:, 0])) < cfg.inlier_m
            n = int(inl.sum())
            if best is None or n > best[0]:
                best = (n, inl)
        if best is None or best[0] < cfg.min_points:
            break
        sel = rest[best[1]]
        if sel[:, 0].max() - sel[:, 0].min() < cfg.min_span_m:
            rest = rest[~best[1]]
            continue
        b, a = np.polyfit(sel[:, 0], sel[:, 1], 1)  # уточняем по всем точкам линии
        if abs(b) > max_b:
            rest = rest[~best[1]]
            continue
        lines.append((float(a), float(b), int(best[0]), sel))
        rest = rest[np.abs(rest[:, 1] - (a + b * rest[:, 0])) >= cfg.inlier_m * 1.5]
    return lines


def lane_from_lines(lines, cfg: LaneCfg):
    """Ближайшие линии слева и справа -> куда держать центр робота и угол полосы."""
    left = min((ln for ln in lines if ln[0] > 0), key=lambda ln: ln[0], default=None)
    right = max((ln for ln in lines if ln[0] <= 0), key=lambda ln: ln[0], default=None)
    if left and right and not (0.4 * cfg.lane_width < left[0] - right[0] < 1.6 * cfg.lane_width):
        # одна из «линий» — не край этой полосы: оставляем более длинную
        left, right = (left, None) if left[2] >= right[2] else (None, right)
    if not left and not right:
        return None
    if left and right:
        # Reject crossing/diverging edges BEFORE forcing a common slope.
        lo = min(left[3][:,0].min(), right[3][:,0].min())
        hi = max(left[3][:,0].max(), right[3][:,0].max())
        if (abs(np.arctan(left[1])-np.arctan(right[1])) > np.radians(8) or
                abs((left[1]-right[1])*(hi-lo)) > .5):
            return None
        widths = [(left[0]-right[0])+(left[1]-right[1])*x for x in (lo,hi)]
        if not all(.4*cfg.lane_width < w < 1.6*cfg.lane_width for w in widths):
            return None
        # края полосы параллельны: общий наклон по точкам обеих линий, у каждой свой сдвиг.
        # Так линия, видная только вдали, не уводит угол
        sx = sxy = 0.0
        for ln in (left, right):
            dx = ln[3][:, 0] - ln[3][:, 0].mean()
            sx += float(dx @ dx)
            sxy += float(dx @ (ln[3][:, 1] - ln[3][:, 1].mean()))
        b = sxy / sx
        left, right = ((float(ln[3][:, 1].mean() - b * ln[3][:, 0].mean()), b) + ln[2:]
                       for ln in (left, right))
    else:
        b = (left or right)[1]
    if cfg.follow == 'center' and not (left and right):
        return None  # a single edge cannot measure a lane centre or width
    if (cfg.follow == 'right' and right is None) or (cfg.follow == 'left' and left is None):
        return None
    width = left[0] - right[0] if left and right else 0.
    if cfg.follow == 'right' and right:
        a = right[0] + cfg.keep_m
    elif cfg.follow == 'left' and left:
        a = left[0] - cfg.keep_m
    elif left and right:
        a = 0.5 * (left[0] + right[0])
    else:
        a = left[0] - width / 2 if left else right[0] + width / 2
    return {'offset': a, 'angle': float(np.arctan(b)), 'width': float(width),
            'lines': int(bool(left)) + int(bool(right)),
            'left': left[:2] if left else None, 'right': right[:2] if right else None}


def detect_lane(bgr, bird: BirdView, rng=None, debug=None):
    """Кадр -> {'offset': м (+ — цель левее робота), 'angle': рад (+ — полоса уходит влево),
    'width': м, 'lines': 1|2, 'left'/'right': (a, b) краёв} или None, если разметки не видно.
    debug — словарь, куда положить найденные точки краски (для отладочной картинки)."""
    if cv2 is None or bgr is None or bgr.size == 0:
        return None
    pts, clutter = marking_points(bgr, bird)
    if debug is not None:
        debug['pts'], debug['clutter'] = pts, clutter
    if clutter > bird.cfg.clutter_max:
        return None  # на дороге краска — пара узких полос; когда «полос» всюду, прямые случайны
    lines = fit_lines(pts, bird.cfg, rng)
    lane = lane_from_lines(lines, bird.cfg)
    if debug is not None:
        debug['candidates'] = [ln[:2] for ln in lines]
        debug['reason'] = 'single edge, centre unknown' if len(lines)==1 else 'edges inconsistent' if lines and lane is None else 'no marking'
    return lane


class LaneTracker:
    """Require a stable sequence; never retain a lane after loss or a jump."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.last = self.filtered = None
        self.count = 0
        self.started = self.last_t = None

    def update(self, lane, now):
        if lane is None:
            self.reset()
            return None
        stable = (self.last is not None and 0 < now-self.last_t <= .4 and
                  abs(lane['offset']-self.last['offset']) < .25 and
                  abs(lane['angle']-self.last['angle']) < np.radians(6) and
                  abs(lane['width']-self.last['width']) < .4 and
                  lane['lines'] == self.last['lines'])
        if not stable:
            self.reset()
            self.started = now
        self.count += 1
        self.last, self.last_t = dict(lane), now
        if self.count < 3 or now-self.started < .15:
            return None
        out = dict(lane)
        if self.filtered is not None:
            for key in ('offset', 'angle', 'width'):
                out[key] = .35*lane[key]+.65*self.filtered[key]
        self.filtered = out
        return out
