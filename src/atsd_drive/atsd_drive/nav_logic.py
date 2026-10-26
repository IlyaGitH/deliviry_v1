LEG_POINT = {'TO_PICKUP': 'from', 'TO_DROP': 'to', 'RETURN': None}
# На этих паузах цель снимается, иначе Nav2 сочтёт стоящего робота застрявшим.
# Препятствия здесь нет: его объезжает сам Nav2.
HOLD_PAUSES = {'estop', 'no_pi_link', 'stop_sign', 'red_light', 'speed_limit_zero', 'recovery'}
# Возврат на базу едет к точке home из points.json. Начало координат карты не годится:
# это место, где включали Brain, а не где начинали съёмку
HOME = 'home'


def want_leg(mission: dict):
    """(состояние, точка), куда сейчас надо ехать, или None."""
    state = mission.get('state')
    if state not in LEG_POINT or HOLD_PAUSES & set(mission.get('pause') or []):
        return None
    point = mission.get(LEG_POINT[state]) if LEG_POINT[state] else HOME
    return (state, point) if point else None


def find_point(points: list, pid: str):
    return next((p for p in points if p.get('id') == pid), None)
