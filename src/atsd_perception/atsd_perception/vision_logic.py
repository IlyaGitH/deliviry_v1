from collections import deque
from dataclasses import dataclass

import numpy as np

try:
    import cv2
except Exception:
    cv2 = None


@dataclass
class LightHsv:
    s_min: int = 90
    v_min: int = 140
    min_fraction: float = 0.02


def classify_light(bgr_roi, cfg: LightHsv = LightHsv()):
    if cv2 is None or bgr_roi is None or bgr_roi.size == 0:
        return None, {}
    h_img = bgr_roi.shape[0]
    if h_img < 6:
        return None, {}
    hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    bright = (s >= cfg.s_min) & (v >= cfg.v_min)

    masks = {
        'R': bright & ((h <= 10) | (h >= 160)),
        'Y': bright & (h >= 15) & (h <= 35),
        'G': bright & (h >= 40) & (h <= 95),
    }
    area = float(bgr_roi.shape[0] * bgr_roi.shape[1])
    rows = np.arange(h_img)[:, None]
    third = {'R': rows < h_img / 3, 'Y': (rows >= h_img / 3) & (rows < 2 * h_img / 3),
             'G': rows >= 2 * h_img / 3}

    scores = {}
    for color, m in masks.items():
        total = float(np.count_nonzero(m))
        in_place = float(np.count_nonzero(m & third[color]))
        scores[color] = (total + in_place) / area

    best = max(scores, key=scores.get)
    if scores[best] < cfg.min_fraction:
        return None, scores
    return best, scores


def distance_from_height(box_h_px: float, real_h_m: float, focal_px: float):
    if box_h_px <= 1:
        return 0.0
    return real_h_m * focal_px / box_h_px


class Confirm:

    def __init__(self, need=2, window=3):
        self.need = need
        self.hist = deque(maxlen=window)

    def update(self, label):
        self.hist.append(label)
        if label is None:
            return None
        return label if sum(1 for x in self.hist if x == label) >= self.need else None


@dataclass
class CameraGeom:
    focal_px: float = 460.0
    cx: float = 320.0
    cy: float = 240.0
    x: float = 0.28
    y: float = 0.0
    z: float = 0.385
    pitch_rad: float = 0.17


@dataclass
class SpotCfg:
    s_min: int = 110
    v_min: int = 60
    hue_low_max: int = 10
    hue_high_min: int = 170
    min_area_px: int = 250
    min_area_m2: float = 0.02
    max_area_m2: float = 2.5
    min_range_m: float = 0.4
    max_range_m: float = 3.5


def pixel_to_ground(u, v, cam: CameraGeom):
    xc = (u - cam.cx) / cam.focal_px
    yc = (v - cam.cy) / cam.focal_px
    cp, sp = np.cos(cam.pitch_rad), np.sin(cam.pitch_rad)
    down = sp + cp * yc
    if down <= 1e-6:
        return None
    t = cam.z / down
    return cam.x + t * (cp - sp * yc), cam.y - t * xc


def ground_to_pixel(gx, gy, cam: CameraGeom):
    cp, sp = np.cos(cam.pitch_rad), np.sin(cam.pitch_rad)
    px, py, pz = gx - cam.x, gy - cam.y, -cam.z
    depth = px * cp - pz * sp
    if depth <= 1e-6:
        return None
    xc = -py / depth
    yc = (-px * sp - pz * cp) / depth
    return cam.cx + xc * cam.focal_px, cam.cy + yc * cam.focal_px


def red_mask(bgr, cfg: SpotCfg):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    m = ((h <= cfg.hue_low_max) | (h >= cfg.hue_high_min)) & (s >= cfg.s_min) & (v >= cfg.v_min)
    return m.astype(np.uint8) * 255


def find_red_spot(bgr, cam: CameraGeom, cfg: SpotCfg = SpotCfg()):
    if cv2 is None or bgr is None or bgr.size == 0:
        return None
    mask = red_mask(bgr, cfg)
    horizon = int(cam.cy - cam.focal_px * np.tan(cam.pitch_rad)) + 10
    mask[:max(0, horizon)] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        area_px = cv2.contourArea(c)
        if area_px < cfg.min_area_px:
            continue
        x, y, w, h = cv2.boundingRect(c)
        mom = cv2.moments(c)
        ucol = mom['m10'] / mom['m00'] if mom['m00'] else x + w / 2
        near = pixel_to_ground(ucol, y + h, cam)
        far = pixel_to_ground(ucol, y, cam)
        left = pixel_to_ground(x, y + h, cam)
        right = pixel_to_ground(x + w, y + h, cam)
        if None in (near, far, left, right):
            continue
        gx = 0.5 * (near[0] + far[0])
        gy = 0.5 * (near[1] + far[1])
        width = abs(left[1] - right[1])
        depth = far[0] - near[0]
        area = width * depth * area_px / float(w * h)
        rng = float(np.hypot(gx, gy))
        if not (cfg.min_area_m2 <= area <= cfg.max_area_m2 and cfg.min_range_m <= rng <= cfg.max_range_m):
            continue
        if best is None or area > best['area']:
            best = {'x': float(gx), 'y': float(gy), 'area': float(area), 'box': [x, y, x + w, y + h]}
    return best
