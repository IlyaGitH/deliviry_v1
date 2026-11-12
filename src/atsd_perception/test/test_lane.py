import math

import cv2
import numpy as np

from atsd_perception.lane_logic import BirdView, LaneCfg, detect_lane
from atsd_perception.vision_logic import CameraGeom, ground_to_pixel

CAM = CameraGeom(z=.385, pitch_rad=.17)  # legacy geometry regression scenes
CFG = LaneCfg()
BIRD = BirdView(CAM, CFG)


def road(lines, seed=0, shadow=False, noise=12, cam=CAM):
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
                    quad.append(ground_to_pixel(xx + nx, y + ny, cam))
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
            # These views crop almost all of one boundary: old code invented it
            # from lane_width. They must now report no measured lane centre.
            if (c,deg) in {(-.8,-12),(-.3,-12),(.4,15),(.9,7),(.9,15)}:
                assert detect_lane(road(two_lines(c,math.radians(deg)), seed=abs(int(c*10)*100+deg)),BIRD) is None
            else:
                check(two_lines(c, math.radians(deg)), c, math.radians(deg), seed=abs(int(c * 10) * 100 + deg))


def test_dashed_center_line_and_yellow_edge():
    lines = [(1.75, 0.05, 0.12, WHITE, 1.0), (-1.75, 0.05, 0.15, YELLOW, None)]
    check(lines, 0.0, 0.05)


def test_shadow_across_road():
    check(two_lines(0.3, math.radians(-4)), 0.3, math.radians(-4), shadow=True)


def test_single_line_does_not_invent_center_or_width():
    debug = {}
    lane = detect_lane(road([(-1.2, 0.0, 0.12, WHITE, None)]), BIRD, debug=debug)
    assert lane is None
    assert debug['candidates'] and debug['reason'] == 'single edge, centre unknown'


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


def test_saturated_blue_edges_are_not_road_paint():
    assert detect_lane(road([(y,0.,.12,(255,40,30),None) for y in (-1.75,1.75)]), BIRD) is None


def test_door_edges_do_not_form_a_lane():
    image = np.full((480,640,3), 70, np.uint8)
    for x in (130,510):
        cv2.line(image,(x,140),(x,470),(235,235,235),8)
    assert detect_lane(image, BIRD) is None


def test_diverging_edges_are_not_forced_parallel():
    from atsd_perception.lane_logic import lane_from_lines
    x=np.linspace(1,5,100)
    lines=[]
    for a,b in ((1.75,.15),(-1.75,-.15)):
        points=np.column_stack([x,a+b*x])
        lines.append((a,b,len(x),points))
    assert lane_from_lines(lines,CFG) is None


def test_temporal_confirmation_rejects_jumps_loss_and_long_gaps():
    from atsd_perception.lane_logic import LaneTracker
    tracker=LaneTracker()
    lane=detect_lane(road(two_lines(0.,0.)), BIRD)
    assert tracker.update(lane,0.) is None
    assert tracker.update(lane,.1) is None
    assert tracker.update(lane,.2) is not None
    jumped={**lane,'offset':1.7}
    assert tracker.update(jumped,.3) is None
    assert tracker.update(None,.4) is None
    assert tracker.update(lane,.5) is None
    assert tracker.update(lane,.6) is None
    assert tracker.update(lane,.7) is not None
    assert tracker.update(lane,2.) is None


def test_horizontal_camera_at_thirty_cm_detects_two_boundaries():
    cam=CameraGeom(z=.30,pitch_rad=0.)
    bird=BirdView(cam,CFG)
    for offset in (-.3,0.,.3):
        lane=detect_lane(road(two_lines(offset,0.),cam=cam),bird)
        assert lane is not None and lane['lines']==2
        assert abs(lane['offset']-offset)<.08
        assert abs(lane['width']-3.5)<.15
