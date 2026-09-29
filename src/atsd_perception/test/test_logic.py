import math

import cv2
import numpy as np

from atsd_perception.obstacle_logic import CorridorConfig, Debounce, analyze, scan_to_xy
from atsd_perception.vision_logic import Confirm, classify_light, distance_from_height


def box(cx, cy, w, h, step=0.02):
    xs = np.arange(cx - w / 2, cx + w / 2 + 1e-6, step)
    ys = np.arange(cy - h / 2, cy + h / 2 + 1e-6, step)
    edge = [(x, cy - h / 2) for x in xs] + [(x, cy + h / 2) for x in xs] + \
           [(cx - w / 2, y) for y in ys] + [(cx + w / 2, y) for y in ys]
    return np.array(edge, dtype=np.float32)


CFG = CorridorConfig()


def test_clear_corridor():
    walls = np.vstack([box(2, 1.5, 6, 0.05), box(2, -1.5, 6, 0.05)])
    assert analyze(walls, CFG)[0] is None


def test_obstacle_ahead_prefers_default_left():
    pts = box(1.5, 0.0, 0.4, 0.4)
    dist, side, _ = analyze(pts, CFG)
    assert abs(dist - (1.3 - 0.35)) < 0.03
    assert side == 'L'


def test_wall_on_left_forces_right():
    pts = np.vstack([box(1.5, 0.0, 0.4, 0.4), box(1.5, 0.7, 4.0, 0.05)])
    assert analyze(pts, CFG)[1] == 'R'


def test_both_sides_blocked():
    pts = np.vstack([box(1.5, 0.0, 0.4, 0.4), box(1.5, 0.7, 4, 0.05), box(1.5, -0.7, 4, 0.05)])
    assert analyze(pts, CFG)[1] == 'B'


def test_single_noise_point_ignored():
    assert analyze(np.array([[1.0, 0.0]], dtype=np.float32), CFG)[0] is None


def test_scan_to_xy_front_point():
    r = [math.inf] * 360
    r[180] = 1.0       # angle_min=-pi, inc=1deg -> индекс 180 = 0 рад
    pts = scan_to_xy(r, -math.pi, math.radians(1), 0.05, 12, laser_x=0.29)
    assert pts.shape == (1, 2) and abs(pts[0, 0] - 1.29) < 1e-4


def test_debounce():
    d = Debounce(2, 3)
    assert [d.update(x) for x in (True, True, False, False, False)] == [False, True, True, True, False]


def light(color_row):
    img = np.zeros((90, 30, 3), np.uint8)
    bgr = {'R': (0, 0, 255), 'Y': (0, 220, 255), 'G': (0, 255, 0)}
    rows = {'R': (5, 25), 'Y': (35, 55), 'G': (65, 85)}
    y0, y1 = rows[color_row]
    cv2.circle(img, (15, (y0 + y1) // 2), 9, bgr[color_row], -1)
    return img


def test_classify_light_colors():
    for c in 'RYG':
        assert classify_light(light(c))[0] == c
    assert classify_light(np.zeros((90, 30, 3), np.uint8))[0] is None


def test_distance_and_confirm():
    assert abs(distance_from_height(46, 0.30, 460) - 3.0) < 1e-6
    c = Confirm(2, 3)
    assert [c.update(x) for x in ('R', None, 'R', 'G')] == [None, None, 'R', None]


def synth_floor(cam, spot_center, size=0.4, sign=True):
    from atsd_perception.vision_logic import ground_to_pixel
    img = np.full((480, 640, 3), (110, 110, 110), np.uint8)
    cx, cy = spot_center
    h = size / 2
    pts = [ground_to_pixel(cx + dx, cy + dy, cam) for dx, dy in ((-h, -h), (-h, h), (h, h), (h, -h))]
    cv2.fillPoly(img, [np.array(pts, np.int32)], (20, 20, 220))
    if sign:
        cv2.circle(img, (500, 60), 40, (0, 0, 230), -1)
    return img


def test_red_spot_ground_position():
    from atsd_perception.vision_logic import CameraGeom, find_red_spot, ground_to_pixel, pixel_to_ground
    cam = CameraGeom()
    u, v = ground_to_pixel(1.7, -0.3, cam)
    gx, gy = pixel_to_ground(u, v, cam)
    assert abs(gx - 1.7) < 1e-6 and abs(gy + 0.3) < 1e-6
    for center in ((1.2, 0.0), (2.0, 0.35), (2.8, -0.5)):
        spot = find_red_spot(synth_floor(cam, center), cam)
        assert spot is not None
        assert abs(spot['x'] - center[0]) < 0.08 and abs(spot['y'] - center[1]) < 0.05, (center, spot)
        assert 0.1 < spot['area'] < 0.25


def test_red_spot_ignores_sign_above_horizon():
    from atsd_perception.vision_logic import CameraGeom, find_red_spot
    img = np.full((480, 640, 3), (110, 110, 110), np.uint8)
    cv2.circle(img, (320, 80), 60, (0, 0, 230), -1)
    assert find_red_spot(img, CameraGeom()) is None


def test_ncnn_yolo_decode_maps_back_to_frame():
    from atsd_perception.ncnn_yolo import decode, letterbox
    frame = np.zeros((480, 640, 3), np.uint8)
    img, r, left, top = letterbox(frame, 640)
    assert img.shape == (640, 640, 3) and r == 1.0 and left == 0 and top == 80
    nc, n = 5, 8400
    pred = np.zeros((4 + nc, n), np.float32)

    def put(i, box, cls, score):
        x1, y1, x2, y2 = box
        pred[:4, i] = [(x1 + x2) / 2 * r + left, (y1 + y2) / 2 * r + top, (x2 - x1) * r, (y2 - y1) * r]
        pred[4 + cls, i] = score

    put(10, (100, 50, 160, 120), 0, 0.91)
    put(11, (102, 52, 162, 118), 0, 0.80)
    put(12, (400, 300, 440, 400), 4, 0.75)
    put(13, (10, 10, 20, 20), 2, 0.30)
    dets = decode(pred[None], r, left, top, frame.shape, conf=0.5, iou=0.45,
                  names={0: 'stop', 4: 'traffic_light'})
    dets.sort(key=lambda d: d['box'][0])
    assert [d['cls'] for d in dets] == ['stop', 'traffic_light']
    assert np.allclose(dets[0]['box'], [100, 50, 160, 120], atol=0.5) and abs(dets[0]['conf'] - 0.91) < 1e-6
    assert np.allclose(dets[1]['box'], [400, 300, 440, 400], atol=0.5)
    assert decode(pred.T, r, left, top, frame.shape, conf=0.95) == []


def test_ncnn_metadata(tmp_path):
    from atsd_perception.ncnn_yolo import read_metadata
    (tmp_path / 'metadata.yaml').write_text(
        "names:\n  0: stop\n  1: crosswalk\nimgsz:\n- 640\n- 640\n", encoding='utf-8')
    assert read_metadata(tmp_path) == ({0: 'stop', 1: 'crosswalk'}, 640)


def test_ncnn_yolo_end_to_end_with_tiny_net(tmp_path):
    import struct
    import pytest
    pytest.importorskip('ncnn')
    from atsd_perception.ncnn_yolo import NcnnYolo
    (tmp_path / 'model.ncnn.param').write_text(
        '7767517\n4 4\n'
        'Input in0 0 1 in0 0=640 1=640 2=3\n'
        'Convolution conv0 1 1 in0 c0 0=9 1=1 5=1 6=27\n'
        'Interp interp0 1 1 c0 i0 0=1 3=1 4=8400\n'
        'Reshape reshape0 1 1 i0 out0 0=8400 1=9\n')
    bias = [320.0, 320.0, 100.0, 100.0, 0.9, 0.1, 0.1, 0.1, 0.1]
    (tmp_path / 'model.ncnn.bin').write_bytes(struct.pack('<I', 0) + struct.pack('<27f', *[0.0] * 27)
                                              + struct.pack('<9f', *bias))
    (tmp_path / 'metadata.yaml').write_text('names:\n  0: stop\nimgsz: [640, 640]\n', encoding='utf-8')
    model = NcnnYolo(tmp_path, threads=2)
    dets = model.predict(np.zeros((480, 640, 3), np.uint8), conf=0.5)
    assert len(dets) == 1 and dets[0]['cls'] == 'stop'
    assert np.allclose(dets[0]['box'], [270, 190, 370, 290], atol=1.0)
