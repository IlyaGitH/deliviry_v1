from atsd_drive.nav_logic import find_point, want_leg


def test_leg_follows_mission_state():
    m = {'state': 'TO_PICKUP', 'from': 'kpp', 'to': 'admin', 'pause': []}
    assert want_leg(m) == ('TO_PICKUP', 'kpp')
    assert want_leg({**m, 'state': 'TO_DROP'}) == ('TO_DROP', 'admin')
    assert want_leg({**m, 'state': 'RETURN'}) == ('RETURN', 'home')
    assert want_leg({**m, 'state': 'WAIT_LOAD'}) is None
    assert want_leg({**m, 'state': 'FAULT'}) is None
    assert want_leg({'state': 'UNKNOWN', 'link': False}) is None


def test_hold_pauses_drop_goal_but_obstacle_does_not():
    m = {'state': 'TO_DROP', 'from': 'kpp', 'to': 'admin'}
    assert want_leg({**m, 'pause': ['red_light']}) is None
    assert want_leg({**m, 'pause': ['stop_sign', 'obstacle']}) is None
    assert want_leg({**m, 'pause': ['obstacle']}) == ('TO_DROP', 'admin')


def test_find_point():
    pts = [{'id': 'kpp', 'x': 2.0, 'y': 0.0}]
    assert find_point(pts, 'kpp')['x'] == 2.0
    assert find_point(pts, 'home') == {'id': 'home', 'x': 0.0, 'y': 0.0}
    assert find_point(pts, 'nope') is None
