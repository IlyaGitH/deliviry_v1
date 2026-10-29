# Разбор состояния GNSS и настройка систем u-blox M8 — без ROS, чтобы проверять тестами.

FIX_NAME = {1: 'нет', 2: '2D', 3: '3D'}  # поле GSA: 1 — решения нет


def snr_summary(gsv):
    """Медиана уровня сигнала и сколько спутников его дают: «41дБx17».
    Под открытым небом 30–48 и разброс; много спутников с ровным ~40 и без фикса —
    похоже на ретранслятор или подмену, все ниже 20 — экранировано или глушат."""
    snr = sorted(s for v in gsv.values() for s in v[1])
    return f'{snr[len(snr) // 2]}дБx{len(snr)}' if snr else '-'


# UBX-CFG-GNSS: id системы, резерв и максимум каналов. SBAS и QZSS живут на частоте GPS L1
GNSS_ID = {'GPS': (0, 8, 16), 'SBAS': (1, 1, 3), 'GALILEO': (2, 4, 8),
           'BEIDOU': (3, 8, 16), 'QZSS': (5, 0, 3), 'GLONASS': (6, 8, 14)}
MAJOR = ('GPS', 'GLONASS', 'GALILEO', 'BEIDOU')


def cfg_gnss_payload(systems):
    """Какие системы слушать: ['GLONASS', 'BEIDOU'] — без GPS, если его глушат или подменяют."""
    on = set(systems)
    if 'GPS' in on:
        on |= {'SBAS', 'QZSS'}
    blocks = b''
    for name, (gid, res, mx) in GNSS_ID.items():
        flags = (0x01 << 16) | (0x01 if name in on else 0)  # сигнал L1, бит 0 — включена
        blocks += bytes([gid, res if name in on else 0, mx, 0]) + flags.to_bytes(4, 'little')
    return bytes([0, 0, 0xFF, len(GNSS_ID)]) + blocks
