# Оборот лидара LD19 из пакетов — сразу массивом (numpy), без цикла по 4500 точкам в секунду на Python.
import numpy as np

POINTS = 12  # точек в пакете
_POINT = np.dtype([('d', '<u2'), ('q', 'u1')])  # расстояние мм, яркость


def bins_from_packets(starts, ends, payloads, n, invert=True, offset=0.0, range_min=0.05, range_max=12.0,
                      min_q=0, crop=()):
    """starts/ends — углы пакетов (°), payloads — 36 байт точек каждого. -> (ranges[n], intensities[n], точек).
    В каждом секторе — ближайшая точка; пустые сектора — inf."""
    data = np.frombuffer(b''.join(payloads), _POINT).reshape(len(payloads), POINTS)
    s, e = np.asarray(starts, float), np.asarray(ends, float)
    step = ((e - s) % 360.0) / (POINTS - 1)
    ang = (s[:, None] + step[:, None] * np.arange(POINTS)) % 360.0
    d = data['d'] / 1000.0
    keep = (data['d'] > 0) & (data['q'] >= min_q) & (d >= range_min) & (d <= range_max)
    a = ((-ang if invert else ang) + offset + 180.0) % 360.0 - 180.0
    for lo, hi in crop:  # сектора, закрытые корпусом робота
        keep &= ~(((a >= lo) & (a <= hi)) if lo <= hi else ((a >= lo) | (a <= hi)))
    a, d, q = a[keep], d[keep], data['q'][keep]
    idx = np.minimum(((a + 180.0) / 360.0 * n).astype(int), n - 1)
    o = np.lexsort((d, idx))                       # по сектору, внутри — по расстоянию
    idx, d, q = idx[o], d[o], q[o]
    first = np.r_[True, idx[1:] != idx[:-1]] if len(idx) else np.zeros(0, bool)
    ranges = np.full(n, np.inf)
    intens = np.zeros(n)
    ranges[idx[first]], intens[idx[first]] = d[first], q[first]
    return ranges, intens, int(keep.sum())
