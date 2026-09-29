from dataclasses import dataclass, field
import uuid as uuidlib

APPLE_ID = 0x004C


def parse_ibeacon(manufacturer_data: dict):
    data = manufacturer_data.get(APPLE_ID) if manufacturer_data else None
    if not data or len(data) < 23 or data[0] != 0x02 or data[1] != 0x15:
        return None
    return {
        'uuid': str(uuidlib.UUID(bytes=bytes(data[2:18]))).upper(),
        'major': int.from_bytes(data[18:20], 'big'),
        'minor': int.from_bytes(data[20:22], 'big'),
    }


@dataclass
class BeaconRule:
    point: str
    name: str = ''
    minor: int = -1


@dataclass
class ZoneTracker:
    rules: list
    ibeacon_uuid: str = ''
    near_rssi: int = -65
    hysteresis: int = 5
    lost_s: float = 4.0
    alpha: float = 0.35
    rssi: dict = field(default_factory=dict)
    seen: dict = field(default_factory=dict)
    near: dict = field(default_factory=dict)

    def match(self, name, manufacturer_data):
        name = (name or '').strip().upper()
        beacon = parse_ibeacon(manufacturer_data)
        for r in self.rules:
            if r.name and name == r.name.upper():
                return r.point
            if beacon and r.minor >= 0 and beacon['minor'] == r.minor and \
                    (not self.ibeacon_uuid or beacon['uuid'] == self.ibeacon_uuid.upper()):
                return r.point
        return None

    def update(self, name, manufacturer_data, rssi, now):
        point = self.match(name, manufacturer_data)
        if point is None or rssi is None:
            return None
        prev = self.rssi.get(point)
        if prev is None or now - self.seen.get(point, 0) > self.lost_s:
            prev = rssi
        value = prev + self.alpha * (rssi - prev)
        self.rssi[point] = value
        self.seen[point] = now
        thr = self.near_rssi - (self.hysteresis if self.near.get(point) else 0)
        self.near[point] = value >= thr
        return point

    def zones(self, now):
        return {p: {'rssi': round(v), 'near': self.near.get(p, False), 'age': round(now - self.seen[p], 1)}
                for p, v in self.rssi.items() if now - self.seen[p] <= self.lost_s}

    def best(self, now):
        z = self.zones(now)
        if not z:
            return None
        point = max(z, key=lambda k: z[k]['rssi'])
        return point, z[point]['rssi'], z[point]['near']
