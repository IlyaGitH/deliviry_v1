LEG_POINT = {'TO_PICKUP': 'from', 'TO_DROP': 'to', 'RETURN': None}
# На этих паузах цель снимается, иначе Nav2 сочтёт стоящего робота застрявшим.
# Препятствия здесь нет: его объезжает сам Nav2.
HOLD_PAUSES = {'estop', 'no_pi_link', 'stop_sign', 'red_light', 'speed_limit_zero', 'recovery'}
HOME = {'id': 'home', 'x': 0.0, 'y': 0.0}  # начало карты = где стоял робот при картографировании


def want_leg(mission: dict):
    """(состояние, точка), куда сейчас надо ехать, или None."""
    state = mission.get('state')
    if state not in LEG_POINT or HOLD_PAUSES & set(mission.get('pause') or []):
        return None
    point = mission.get(LEG_POINT[state]) if LEG_POINT[state] else HOME['id']
    return (state, point) if point else None


def find_point(points: list, pid: str):
    pt = next((p for p in points if p.get('id') == pid), None)
    return pt or (HOME if pid == HOME['id'] else None)
