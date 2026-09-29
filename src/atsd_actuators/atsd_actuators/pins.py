HEADLIGHT = 18
STRIP = 13
LOCK = 19
BUZZER = 26
DOOR_SENSOR = 27

RESERVED = {
    2: 'I2C SDA — X1202',
    3: 'I2C SCL — X1202',
    14: 'UART TXD — GNSS BN-880',
    15: 'UART RXD — GNSS BN-880',
    6: 'X1202, детектор пропадания питания',
    16: 'X1202, управление зарядом',
}


def check_pin(pin: int, purpose: str) -> int:
    pin = int(pin)
    if pin in RESERVED:
        raise ValueError(f'GPIO{pin} для «{purpose}» занят: {RESERVED[pin]}')
    if not 2 <= pin <= 27:
        raise ValueError(f'GPIO{pin} для «{purpose}» вне колодки')
    return pin


def check_unique(pins: dict):
    seen = {}
    for purpose, pin in pins.items():
        if pin in seen:
            raise ValueError(f'GPIO{pin} назначен дважды: «{seen[pin]}» и «{purpose}»')
        seen[pin] = purpose
