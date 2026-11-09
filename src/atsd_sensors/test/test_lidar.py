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
        fin = np.isfinite(r_ref) & np.isfinite(r_new)
        if not crop:
            assert np.isfinite(r_new[np.isfinite(r_ref)]).all()
        assert np.allclose(r_new[fin], r_ref[fin]) and np.allclose(q_new[fin], q_ref[fin])


def test_empty():
    r, q, n = bins_from_packets([0.0], [8.8], [bytes(36)], 100)
    assert np.isposinf(r).any() and np.isnan(r).any() and n == 0


def test_missing_and_cropped_sectors_are_unknown_not_free():
    payload = b''.join(struct.pack('<HB', 0, 80) for _ in range(POINTS))
    ranges, _, _ = bins_from_packets([0.], [20.], [payload], 360, False, crop=((5., 10.),))
    assert np.isposinf(ranges[181:184]).all()  # received rays without a return
    assert np.isnan(ranges[185:190]).all()  # deliberately masked
    assert np.isnan(ranges[:170]).all()  # packet never received


def test_low_quality_returns_do_not_authorize_free_space():
    payload = b''.join(struct.pack('<HB', 500, 1) for _ in range(POINTS))
    ranges, _, _ = bins_from_packets([0.], [11.], [payload], 360, False, min_q=20)
    assert np.isnan(ranges[180:192]).all()


def test_full_crop_never_becomes_free_inf():
    ranges, _, _ = bins_from_packets([0.], [20.], [bytes(36)], 360, crop=((-180.,180.),))
    assert np.isnan(ranges).all()


def test_native_sampling_gaps_preserve_finite_wall_instead_of_infinity():
    payload = b''.join(struct.pack('<HB', 1000, 80) for _ in range(POINTS))
    starts = np.arange(0., 360., 12.)
    r, _, _ = bins_from_packets(starts, starts+11., [payload]*len(starts), 455)
    assert np.all(r == 1.)


def test_missing_packet_is_still_unknown_with_native_spacing():
    starts = np.arange(0., 360., 12.)[1:]
    r, _, _ = bins_from_packets(starts, starts+11., [bytes(36)]*len(starts), 455, False)
    angles = -180.+(np.arange(455)+.5)*360./455
    assert np.isnan(r[(angles > 2.) & (angles < 10.)]).all()
    assert np.isposinf(r[(angles > 20.) & (angles < 30.)]).all()


def test_no_packets_is_entirely_unknown():
    r, q, count = bins_from_packets([], [], [], 455)
    assert np.isnan(r).all() and not q.any() and count == 0
