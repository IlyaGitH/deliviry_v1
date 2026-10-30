import struct

import numpy as np

from atsd_sensors.lidar_logic import POINTS, bins_from_packets


def reference(starts, ends, payloads, n, invert, offset, rmin, rmax, min_q, crop):
    """Прежний разбор по одной точке — с ним должен совпасть новый."""
    ranges, intens = [float('inf')] * n, [0.0] * n
    for s, e, p in zip(starts, ends, payloads):
        step = ((e - s) % 360.0) / (POINTS - 1)
        for i in range(POINTS):
            dmm, q = struct.unpack_from('<HB', p, i * 3)
            if dmm == 0 or q < min_q:
                continue
            a = -((s + step * i) % 360.0) if invert else (s + step * i) % 360.0
            a = (a + offset + 180.0) % 360.0 - 180.0
            if any((lo <= a <= hi) if lo <= hi else (a >= lo or a <= hi) for lo, hi in crop):
                continue
            d = dmm / 1000.0
            if d < rmin or d > rmax:
                continue
            k = min(int((a + 180.0) / 360.0 * n), n - 1)
            if d < ranges[k]:
                ranges[k], intens[k] = d, float(q)
    return np.array(ranges), np.array(intens)


def test_matches_point_by_point_parsing():
    rng = np.random.default_rng(1)
    starts = np.arange(0, 360, 0.8 * POINTS)[:38] % 360.0           # один оборот, как у LD19
    ends = (starts + 0.8 * (POINTS - 1)) % 360.0
    payloads = []
    for _ in starts:
        pts = [(int(rng.choice([0, rng.integers(30, 13000)])), int(rng.integers(0, 255))) for _ in range(POINTS)]
        payloads.append(b''.join(struct.pack('<HB', d, q) for d, q in pts))
    for invert, crop in ((True, ()), (False, ((150.0, -150.0), (10.0, 30.0)))):
        args = (list(starts), list(ends), payloads, 455, invert, 7.0, 0.05, 12.0, 20, crop)
        r_new, q_new, _ = bins_from_packets(*args)
        r_ref, q_ref = reference(*args)
        assert np.array_equal(np.isinf(r_new), np.isinf(r_ref))
        fin = ~np.isinf(r_ref)
        assert np.allclose(r_new[fin], r_ref[fin]) and np.allclose(q_new[fin], q_ref[fin])


def test_empty():
    r, q, n = bins_from_packets([0.0], [8.8], [bytes(36)], 100)
    assert np.isinf(r).all() and n == 0
