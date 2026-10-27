import math

import cv2
import numpy as np

from atsd_perception.lane_logic import BirdView, LaneCfg, detect_lane
from atsd_perception.vision_logic import CameraGeom, ground_to_pixel

CAM = CameraGeom()
CFG = LaneCfg()
BIRD = BirdView(CAM, CFG)


def road(lines, seed=0, shadow=False, noise=12):
    """Кадр камеры: асфальт и линии y = c + tan(t)·x шириной w; dash — длина штриха/разрыва, м."""
    rng = np.random.default_rng(seed)
    img = np.clip(rng.normal(70, noise, (480, 640, 3)), 0, 255).astype(np.uint8)
    for c, t, w, color, dash in lines:
        x = 0.6
        while x < 14:
            x2 = x + (dash or 0.25)
            if not dash or int(round(x / dash)) % 2 == 0:
                quad = []
                for xx, side in ((x, 1), (x2, 1), (x2, -1), (x, -1)):
                    y = c + math.tan(t) * xx
                    nx, ny = -math.sin(t) * side * w / 2, math.cos(t) * side * w / 2
                    quad.append(ground_to_pixel(xx + nx, y + ny, CAM))
                if None not in quad:
                    cv2.fillPoly(img, [np.array(quad, np.int32)], color)
            x = x2
    if shadow:  # тень от столба поперёк дороги
        img[300:360] = (img[300:360] * 0.45).astype(np.uint8)
    return img


WHITE, YELLOW = (225, 225, 225), (40, 200, 230)


def check(lines, want_offset, want_angle, **kw):
    lane = detect_lane(road(lines, **kw), BIRD)
    assert lane is not None, lines
    assert abs(lane['offset'] - want_offset) < 0.08, (lane, want_offset)
    assert abs(lane['angle'] - want_angle) < math.radians(2.0), (lane, math.degrees(want_angle))
    return lane


def two_lines(c, t, w=3.5, **kw):
    return [(c + w / 2, t, 0.12, WHITE, kw.get('dash_left')), (c - w / 2, t, 0.12, WHITE, None)]


def test_centered_straight():
    lane = check(two_lines(0.0, 0.0), 0.0, 0.0)
    assert lane['lines'] == 2 and abs(lane['width'] - 3.5) < 0.1


def test_offset_and_angle_many_cases():
    for c in (-0.8, -0.3, 0.4, 0.9):
        for deg in (-12, -5, 0, 7, 15):
            check(two_lines(c, math.radians(deg)), c, math.radians(deg), seed=abs(int(c * 10) * 100 + deg))


def test_dashed_center_line_and_yellow_edge():
    lines = [(1.75, 0.05, 0.12, WHITE, 1.0), (-1.75, 0.05, 0.15, YELLOW, None)]
    check(lines, 0.0, 0.05)


def test_shadow_across_road():
    check(two_lines(0.3, math.radians(-4)), 0.3, math.radians(-4), shadow=True)


def test_single_line_uses_lane_width():
    # видна только правая линия в 1.2 м справа: середина полосы на 1.75 м левее неё
    lane = check([(-1.2, 0.0, 0.12, WHITE, None)], -1.2 + 1.75, 0.0)
    assert lane['lines'] == 1


def test_follow_right_line():
    cfg = LaneCfg(follow='right', keep_m=0.6)
    lane = detect_lane(road(two_lines(0.0, 0.0)), BirdView(CAM, cfg))
    assert abs(lane['offset'] - (-1.75 + 0.6)) < 0.08


def test_no_marking_is_none():
    assert detect_lane(road([]), BIRD) is None


def test_wide_bright_patch_is_not_a_line():
    img = road([])
    pts = [ground_to_pixel(x, y, CAM) for x, y in ((2.0, 1.0), (2.0, -1.0), (3.5, -1.0), (3.5, 1.0))]
    cv2.fillPoly(img, [np.array(pts, np.int32)], (230, 230, 230))  # белая машина/лужа 2 × 1.5 м
    assert detect_lane(img, BIRD) is None


def test_many_stripes_is_not_a_road():
    # жалюзи, плитка, окна: светлых полос всюду — прямая через них случайна
    img = road([(i * 0.3, 0.0, 0.12, WHITE, None) for i in range(-10, 11)])
    assert detect_lane(img, BIRD) is None


def test_wide_three_line_road_passes_clutter_gate():
    lane = detect_lane(road([(y, 0.0, 0.2, WHITE, None) for y in (1.75, 0.0, -1.75)]), BIRD)
    assert lane is not None
