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
    12: 'recovery',
    13: 'fault',
    14: 'manual',
    15: 'no_pi_link',
    17: 'zone_near',
    18: 'nudge',
}
FLAG_ESTOP = 1 << 1

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


def odom_offset(last, raw):
    """Сдвиг (x, y, yaw), с которым одометрия Brain после перезагрузки продолжится с позы last."""
    oyaw = last[2] - raw[2]
    c, s = math.cos(oyaw), math.sin(oyaw)
    return last[0] - (c * raw[0] - s * raw[1]), last[1] - (s * raw[0] + c * raw[1]), oyaw


def apply_offset(off, raw):
    c, s = math.cos(off[2]), math.sin(off[2])
    return off[0] + c * raw[0] - s * raw[1], off[1] + s * raw[0] + c * raw[1], raw[2] + off[2]


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
        if kind == 'S' and len(parts) >= 6:
            state = parts[1] if parts[1] in MISSION_STATES else 'UNKNOWN'
            return Status(state, int(parts[2]), int(parts[3]), int(parts[4], 16), int(parts[5]))
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


def cmd_nav(lin_m_s: float, ang_rad_s: float) -> str:
    return f'G {int(round(lin_m_s * 1000.0))} {int(round(ang_rad_s * 1000.0))}\n'


def cmd_nav_result(state: str, ok: bool) -> str:
    return f'{"Y" if ok else "Q"} {state}\n'


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
