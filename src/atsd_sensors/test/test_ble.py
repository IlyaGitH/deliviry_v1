import uuid

from atsd_sensors.ble_logic import BeaconRule, ZoneTracker, parse_ibeacon

UUID = 'E2C56DB5-DFFB-48D2-B060-D0F5A71096E0'


def ibeacon(minor, major=1, u=UUID):
    return {0x004C: bytes([0x02, 0x15]) + uuid.UUID(u).bytes + major.to_bytes(2, 'big')
            + minor.to_bytes(2, 'big') + bytes([0xC5])}


def tracker():
    rules = [BeaconRule('kpp', 'ATSD-KPP', 1), BeaconRule('admin', 'ATSD-ADMIN', 2)]
    return ZoneTracker(rules, UUID, near_rssi=-65)


def test_parse_ibeacon():
    b = parse_ibeacon(ibeacon(3, 7))
    assert b == {'uuid': UUID, 'major': 7, 'minor': 3}
    assert parse_ibeacon({0x004C: b'\x10\x05abc'}) is None


def test_match_by_name_and_ibeacon():
    t = tracker()
    assert t.match('atsd-kpp', {}) == 'kpp'
    assert t.match(None, ibeacon(2)) == 'admin'
    assert t.match(None, ibeacon(2, u='11111111-2222-3333-4444-555555555555')) is None
    assert t.match('Galaxy A52', {}) is None


def test_zone_near_hysteresis_and_expiry():
    t = tracker()
    for i in range(10):
        t.update('ATSD-KPP', {}, -58, i * 0.5)
    assert t.best(5.0) == ('kpp', -58, True)
    for i in range(3):
        t.update('ATSD-KPP', {}, -68, 5.5 + i * 0.5)
    assert t.best(6.5)[2] is True
    for i in range(10):
        t.update('ATSD-KPP', {}, -80, 7 + i * 0.5)
    assert t.best(12.0)[2] is False
    t.update(None, ibeacon(2), -50, 12.0)
    assert t.best(12.1)[0] == 'admin'
    assert t.best(30.0) is None
