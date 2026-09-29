from dataclasses import dataclass, field
import math

FLAGS = {
    0: 'watchdog',
    1: 'estop',
    2: 'gyro_calibrating',
    3: 'gyro_fault',
    4: 'motor_hot',
    5: 'motor_missing',
    6: 'stall',
    7: 'bumper',
    8: 'battery_low',
    9: 'mission_active',
    10: 'paused',
    11: 'detour',
    12: 'recovery',
    13: 'fault',
    14: 'manual',
    15: 'no_pi_link',
    16: 'docking',
    17: 'zone_near',
}
FLAG_ESTOP = 1 << 1
FLAG_DOCKING = 1 << 16

PAUSE = {
    1: 'estop',
    2: 'no_pi_link',
    4: 'stop_sign',
    8: 'red_light',
    16: 'obstacle',
    32: 'speed_limit_zero',
    64: 'recovery',
}

MISSION_STATES = ('IDLE', 'TO_PICKUP', 'WAIT_LOAD', 'TO_DROP',
                  'WAIT_UNLOAD', 'DONE', 'RETURN', 'FAULT')
TRAVEL_STATES = ('TO_PICKUP', 'TO_DROP', 'RETURN')

SIGN_KINDS = ('STOP', 'CROSS', 'BUMP')
LIGHT_COLORS = ('R', 'Y', 'G', 'N')
SIDES = ('L', 'R', 'B', '?')


@dataclass
class Encoders:
    deg_l: int
    deg_r: int
    heading_mdeg: int
    brain_ms: int
    millivolts: int
    flags: int


@dataclass
class Pose:
    x_mm: int
    y_mm: int
    yaw_mdeg: int
    v_mm_s: int
    w_mrad_s: int


@dataclass
class Status:
    state: str
    from_idx: int
    to_idx: int
    wp: int
    wp_count: int
    remain_mm: int
    pause: int
    mission_ds: int


@dataclass
class Event:
    name: str
    args: list = field(default_factory=list)
    raw: str = ''


@dataclass
class Info:
    text: str


def decode_flags(flags: int) -> list:
    return [name for bit, name in FLAGS.items() if flags & (1 << bit)]


def decode_pause(mask: int) -> list:
    return [name for bit, name in PAUSE.items() if mask & bit]


def parse_line(text: str):
    text = text.strip()
    if len(text) < 1:
        return None
    parts = text.split()
    kind = parts[0]
    try:
        if kind == 'E' and len(parts) >= 7:
            return Encoders(int(parts[1]), int(parts[2]), int(parts[3]),
                            int(parts[4]), int(parts[5]), int(parts[6]))
        if kind == 'P' and len(parts) >= 6:
            return Pose(*(int(v) for v in parts[1:6]))
        if kind == 'S' and len(parts) >= 8:
            wp, _, n = parts[4].partition('/')
            state = parts[1] if parts[1] in MISSION_STATES else 'UNKNOWN'
            return Status(state, int(parts[2]), int(parts[3]), int(wp), int(n or 0),
                          int(parts[5]), int(parts[6], 16), int(parts[7]))
        if kind == 'K' and len(parts) >= 2:
            return Event(parts[1], parts[2:], text[2:])
        if kind == 'I':
            return Info(text[2:].strip())
    except ValueError:
        return None
    return None


def _mm(meters: float) -> int:
    if meters is None or not math.isfinite(meters):
        return 0
    return max(0, int(round(meters * 1000.0)))


def cmd_velocity(lin_m_s: float, ang_rad_s: float) -> str:
    return f'V {int(round(lin_m_s * 1000.0))} {int(round(ang_rad_s * 1000.0))}\n'


def cmd_heartbeat() -> str:
    return 'H\n'


def cmd_mission(from_idx: int, to_idx: int) -> str:
    return f'M {int(from_idx)} {int(to_idx)}\n'


def cmd_simple(letter: str) -> str:
    assert letter in ('C', 'A', 'U', 'X', 'R', '?')
    return f'{letter}\n'


def cmd_speed_limit(fraction: float) -> str:
    pct = int(round(max(0.0, min(1.0, fraction)) * 100))
    return f'L {pct}\n'


def cmd_pose(x_m: float, y_m: float, yaw_rad: float = None) -> str:
    base = f'P {int(round(x_m * 1000))} {int(round(y_m * 1000))}'
    if yaw_rad is not None and math.isfinite(yaw_rad):
        base += f' {int(round(math.degrees(yaw_rad) * 1000))}'
    return base + '\n'


def cmd_sign(kind: str, dist_m: float = 0.0) -> str:
    kind = kind.upper()
    if kind not in SIGN_KINDS:
        raise ValueError(f'неизвестный знак {kind}')
    return f'F {kind} {_mm(dist_m)}\n'


def cmd_light(color: str, dist_m: float = 0.0) -> str:
    color = color.upper()[:1]
    if color not in LIGHT_COLORS:
        raise ValueError(f'неизвестный сигнал {color}')
    return f'T {color} {_mm(dist_m)}\n'


def cmd_obstacle(dist_m: float, side: str = '?', depth_m: float = 0.0,
                 offset_m: float = 0.0) -> str:
    if dist_m is None or dist_m <= 0:
        return 'O 0\n'
    side = (side or '?').upper()[:1]
    if side not in SIDES:
        side = '?'
    return f'O {max(1, _mm(dist_m))} {side} {_mm(depth_m)} {_mm(offset_m)}\n'


def cmd_nudge(lin_m_s: float, ang_rad_s: float) -> str:
    return f'N {int(round(lin_m_s * 1000.0))} {int(round(ang_rad_s * 1000.0))}\n'


def cmd_dock(fwd_m: float, lat_m: float) -> str:
    return f'D {int(round(fwd_m * 1000))} {int(round(lat_m * 1000))}\n'


def cmd_zone(idx: int, rssi: int = -127, near: bool = False) -> str:
    return f'B {int(idx)} {int(rssi)} {1 if near else 0}\n'


def parse_sign_msg(text: str):
    parts = text.split()
    if not parts:
        return None
    kind = parts[0].upper()
    alias = {'CROSSWALK': 'CROSS', 'BUMP_SIGN': 'BUMP', 'BUMP_WARN': 'BUMP'}
    kind = alias.get(kind, kind)
    if kind not in SIGN_KINDS:
        return None
    dist = float(parts[1]) if len(parts) > 1 else 0.0
    return kind, dist


def parse_light_msg(text: str):
    parts = text.split()
    if not parts:
        return None
    color = parts[0].upper()[:1]
    if color not in LIGHT_COLORS:
        return None
    dist = float(parts[1]) if len(parts) > 1 else 0.0
    return color, dist


def parse_spot_msg(text: str):
    parts = text.split()
    if len(parts) < 2 or parts[0].lower() == 'none':
        return None
    return float(parts[0]), float(parts[1])


def parse_zone_msg(text: str):
    parts = text.split()
    if len(parts) < 2 or parts[0].lower() == 'none':
        return None
    near = len(parts) > 2 and parts[2] not in ('0', 'false')
    return parts[0], int(float(parts[1])), near


def parse_obstacle_msg(text: str):
    parts = text.split()
    if not parts or parts[0].lower() in ('clear', 'none', '0', '0.0'):
        return 0.0, '?', 0.0, 0.0
    dist = float(parts[0])
    side = parts[1].upper()[:1] if len(parts) > 1 else '?'
    depth = float(parts[2]) if len(parts) > 2 else 0.0
    offset = float(parts[3]) if len(parts) > 3 else 0.0
    return dist, side, depth, offset
