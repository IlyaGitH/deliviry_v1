from atsd_sensors.gnss_logic import GNSS_ID, cfg_gnss_payload, snr_summary


def blocks(payload):
    assert payload[:4] == bytes([0, 0, 0xFF, len(GNSS_ID)])
    out = {}
    for i in range(4, len(payload), 8):
        b = payload[i:i + 8]
        out[b[0]] = (b[1], b[2], int.from_bytes(b[4:8], 'little'))
    return out


def test_glonass_beidou_without_gps():
    b = blocks(cfg_gnss_payload(['GLONASS', 'BEIDOU']))
    assert len(b) == 6
    on = {gid for gid, (_, _, flags) in b.items() if flags & 1}
    assert on == {6, 3}  # без GPS выключаются и SBAS, QZSS — они на той же частоте
    assert all(flags >> 16 == 1 for _, _, flags in b.values())
    assert sum(res for res, _, _ in b.values()) <= 32


def test_gps_brings_sbas_qzss():
    on = {gid for gid, (_, _, f) in blocks(cfg_gnss_payload(['GPS', 'GLONASS'])).items() if f & 1}
    assert on == {0, 1, 5, 6}


def test_snr_summary():
    assert snr_summary({}) == '-'
    assert snr_summary({'GP': [17, [37, 40, 42, 41], 0.0], 'GL': [5, [26], 0.0]}) == '40дБx5'
