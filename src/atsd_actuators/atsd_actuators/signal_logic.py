from dataclasses import dataclass, field

TRAVEL = ('TO_PICKUP', 'TO_DROP', 'RETURN')
WAITING = ('WAIT_LOAD', 'WAIT_UNLOAD')

ONESHOT = {
    'MISSION_START': 'start',
    'CARGO_LOADED': 'start',
    'STOP_SIGN_GO': 'beep',
    'GREEN_GO': 'beep',
    'ARRIVED_PICKUP': 'arrive',
    'ARRIVED_DROP': 'arrive',
    'DELIVERED': 'double',
    'REJECT': 'double',
    'DETOUR_START': 'double',
    'DOCKED': 'beep',
    'DOCK_NOT_FOUND': 'double',
    'ZONE_MISMATCH': 'double',
}
ONESHOT_LEN = {'beep': 0.2, 'double': 0.4, 'start': 1.0, 'arrive': 1.2}


@dataclass
class SignalConfig:
    head_moving: float = 1.0
    head_drl: float = 0.3
    strip_level: float = 1.0
    strip_idle: float = 0.3
    approach_m: float = 1.8
    crosswalk_beep_s: float = 4.0
    alarm_s: float = 6.0


@dataclass
class Inputs:
    link: bool = False
    state: str = 'UNKNOWN'
    pause: list = field(default_factory=list)
    flags: list = field(default_factory=list)
    obstacle_m: float = 0.0


@dataclass
class Output:
    head: float
    strip: float
    mode: str
    buzzer: str


class SignalLogic:

    def __init__(self, cfg: SignalConfig = SignalConfig()):
        self.cfg = cfg
        self.alarm_since = None
        self.crosswalk_at = -1e9
        self.oneshot = None
        self.oneshot_until = 0.0

    def on_event(self, name: str, now: float):
        if name == 'CROSSWALK_SLOW':
            self.crosswalk_at = now
        pattern = ONESHOT.get(name)
        if pattern:
            self.oneshot = pattern
            self.oneshot_until = now + ONESHOT_LEN.get(pattern, 0.5)
            return pattern
        return None

    def decide(self, i: Inputs, now: float) -> Output:
        c = self.cfg
        emergency = (not i.link) or i.state == 'FAULT' or 'estop' in i.flags
        if emergency:
            if self.alarm_since is None:
                self.alarm_since = now
            buzz = 'alarm' if (now - self.alarm_since) < c.alarm_s and i.link else 'off'
            return Output(c.head_drl, c.strip_level, 'beacon', buzz)
        self.alarm_since = None

        if i.state in TRAVEL:
            if 'recovery' in i.flags:
                return Output(c.head_moving, c.strip_level, 'beacon', 'reverse')
            near = 0.0 < i.obstacle_m <= c.approach_m
            crosswalk = (now - self.crosswalk_at) < c.crosswalk_beep_s
            if i.pause:
                buzz = 'approach' if 'obstacle' in i.pause else 'off'
                return Output(c.head_moving, c.strip_level, 'blink', buzz)
            buzz = 'approach' if (near or crosswalk) else 'off'
            return Output(c.head_moving, c.strip_level, 'beacon', buzz)

        if i.state in WAITING:
            return Output(c.head_drl, c.strip_level, 'solid', 'off')
        if i.state == 'DONE':
            return Output(c.head_drl, c.strip_level, 'solid', 'off')
        return Output(c.head_drl, c.strip_idle, 'solid', 'off')

    def buzzer_free(self, now: float) -> bool:
        return now >= self.oneshot_until
