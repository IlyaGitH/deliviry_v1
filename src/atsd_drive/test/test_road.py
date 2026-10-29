import math

import numpy as np

from atsd_drive.road_logic import PoseFilter, RoadNav, Roads, to_geo, to_local, wrap

# Дорога-«Г» с поворотами: A —60 м→ B —40 м→ C —40 м→ D. Ось полосы = рёбра графа
NODES = {'A': (0.0, 0.0), 'B': (60.0, 0.0), 'C': (60.0, 40.0), 'D': (20.0, 40.0)}
EDGES = [('A', 'B'), ('B', 'C'), ('C', 'D')]
CORNERS = [NODES['B'], NODES['C']]


def lane_seen(truth, rng):
    """Что видит камера: середина ближайшей полосы в координатах робота, с шумом и пропусками."""
    x, y, th = truth
    best = None
    for a, b in EDGES:
        p0, p1 = np.array(NODES[a]), np.array(NODES[b])
        d = p1 - p0
        t = np.clip((np.array([x, y]) - p0) @ d / (d @ d), 0, 1)
        dist = np.linalg.norm(p0 + t * d - (x, y))
        if best is None or dist < best[0]:
            best = (dist, p0, math.atan2(d[1], d[0]))
    _, p0, bearing = best
    if abs(wrap(bearing - th)) > math.pi / 2:
        bearing = wrap(bearing + math.pi)
    ang = wrap(bearing - th)
    if any(math.dist((x, y), c) < 3.0 for c in CORNERS) or abs(ang) > math.radians(35):
        return None  # на перекрёстке разметки нет
    if rng.random() < 0.1:
        return None  # блик, грязь, пешеход закрыл
    c, s = math.cos(th), math.sin(th)
    px, py = c * (p0[0] - x) + s * (p0[1] - y), -s * (p0[0] - x) + c * (p0[1] - y)
    a = py - px * math.tan(ang)
    if abs(a) > 2.5:
        return None
    return {'offset': a + rng.normal(0, 0.03), 'angle': ang + rng.normal(0, math.radians(0.7)),
            'width': 3.5, 'lines': 2}


def deviation(truth):
    """Насколько робот ушёл с оси полосы (вне перекрёстков и первых 5 м, пока выравнивается со старта)."""
    p = np.array(truth[:2])
    if any(math.dist(p, c) < 6.0 for c in CORNERS) or math.dist(p, NODES['A']) < 5.0:
        return 0.0
    out = math.inf
    for a, b in EDGES:  # поперёк дороги: расстояние до оси, а не до конца отрезка
        p0, p1 = np.array(NODES[a]), np.array(NODES[b])
        d = p1 - p0
        t = (p - p0) @ d / (d @ d)
        if -0.1 <= t <= 1.1:
            out = min(out, float(np.linalg.norm(p0 + t * d - p)))
    return out


def simulate(seed, gps='good', odo_scale=1.03, yaw_scale=1.05, pose_given=False):
    rng = np.random.default_rng(seed)
    nav = RoadNav(Roads(NODES, EDGES))
    truth = [rng.normal(0, 0.3), rng.normal(0, 0.3), rng.normal(0, math.radians(5))]
    if pose_given:  # оператор поставил робота кнопкой «Робот здесь»
        nav.filter.reset(0.0, 0.0, 0.0, sig_xy=0.5, sig_yaw=math.radians(5))
    bias = np.zeros(2)
    v = w = 0.0
    states, max_dev, planned = set(), 0.0, False
    for k in range(6000):
        t = k * 0.1
        mid = truth[2] + 0.5 * w * 0.1
        truth = [truth[0] + v * 0.1 * math.cos(mid), truth[1] + v * 0.1 * math.sin(mid), wrap(truth[2] + w * 0.1)]
        nav.filter.predict(v * 0.1 * odo_scale + rng.normal(0, 0.002), w * 0.1 * yaw_scale + rng.normal(0, 0.002))

        if k % 2 == 0:  # GPS 5 раз в секунду
            jam = gps == 'jammed' or (gps == 'outage' and 40 <= t < 90)
            if jam:
                nav.filter.gnss_lost()
            else:
                bias = np.clip(bias + rng.normal(0, 0.05, 2), -2, 2)  # многолучёвость: медленный уход
                fix = np.array(truth[:2]) + bias + rng.normal(0, 0.7, 2)
                if gps == 'spoof' and 40 <= t < 70:
                    fix += (300.0, -150.0)  # подмена: «телепорт» и дальше правдоподобное движение
                nav.filter.gnss_fix(*fix, 2.5)
                if v > 0.3:
                    nav.course(truth[2] + rng.normal(0, math.radians(4)), v, abs(w) > 0.2)
            states.add(nav.filter.gnss)

        if not planned and nav.filter.ready:
            assert nav.plan(NODES['D'])
            planned = True
        v, w, done = nav.tick(lane_seen(truth, rng))
        max_dev = max(max_dev, deviation(truth))
        if done:
            return {'arrived': True, 'miss': math.dist(truth[:2], NODES['D']), 'dev': max_dev,
                    't': t, 'states': states}
    return {'arrived': False, 'miss': math.dist(truth[:2], NODES['D']), 'dev': max_dev, 't': 600, 'states': states}


def test_geo_roundtrip():
    origin = (53.5078, 49.4204)  # Тольятти
    x, y = to_local(53.5088, 49.4234, origin)
    assert abs(y - 111.3) < 0.5 and abs(x - 198.6) < 1.0
    lat, lon = to_geo(x, y, origin)
    assert abs(lat - 53.5088) < 1e-9 and abs(lon - 49.4234) < 1e-9


def test_route_follows_graph_and_skips_passed_node():
    r = Roads(NODES, EDGES)
    assert r.route((0, 0), NODES['D']) == [(0, 0), NODES['B'], NODES['C'], NODES['D']]
    # робот уже на отрезке A→B: в A не возвращается
    assert r.route((20, 0.5), NODES['D'])[1] == NODES['B']
    assert Roads().route((0, 0), (5, 5)) == [(0, 0), (5, 5)]


def test_spoofed_jump_rejected_then_recovered():
    f = PoseFilter()
    f.reset(0, 0, 0.0, 0.5, math.radians(3))
    for i in range(20):
        f.predict(0.1, 0.0)
        f.gnss_fix(0.1 * (i + 1), 0.0, 2.5)
    assert f.gnss == 'ok'
    for i in range(20):  # прыжок на 300 м
        f.predict(0.1, 0.0)
        f.gnss_fix(300 + 0.1 * (i + 21), 0.0, 2.5)
    assert f.gnss == 'suspect' and abs(f.X[0] - 4.0) < 1.0
    for i in range(15):  # подмена кончилась
        f.predict(0.1, 0.0)
        f.gnss_fix(0.1 * (i + 41), 0.0, 2.5)
    assert f.gnss == 'ok' and abs(f.X[0] - 5.5) < 1.0


def run(scenario, seeds, **kw):
    return [simulate(s, scenario, **kw) for s in seeds]


def test_good_gps_several_runs():
    for r in run('good', range(5)):
        assert r['arrived'] and r['miss'] < 2.5 and r['dev'] < 0.5, r


def test_gps_outage_several_runs():
    for r in run('outage', range(5)):
        assert r['arrived'] and r['miss'] < 2.5 and r['dev'] < 0.5, r


def test_gps_spoofing_several_runs():
    for r in run('spoof', range(5)):
        assert r['arrived'] and r['miss'] < 2.5 and r['dev'] < 0.5 and 'suspect' in r['states'], r


def test_gps_jammed_whole_trip_several_runs():
    # без GPS вовсе: путь только по колёсам (откалиброваны до 1 %), курс по разметке
    for r in run('jammed', range(5), odo_scale=1.01, pose_given=True):
        assert r['arrived'] and r['miss'] < 3.0 and r['dev'] < 0.5, r


def test_wifi_catches_spoof_far_away():
    f = PoseFilter()
    # подмена «уводит» GPS на 2 км, Wi-Fi видит нас у начала координат ±40 м
    assert f.gnss_fix(2000.0, 0.0, 2.5, wifi=(0.0, 0.0, 40.0)) == 'suspect'
    assert not f.ready  # по подменённому фиксу позу не задаём
    assert f.gnss_fix(5.0, 3.0, 2.5, wifi=(0.0, 0.0, 40.0)) == 'ok'
    assert f.ready
