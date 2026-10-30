# Курс по гироскопу MPU-6050/6500 (плата GY-521): складываем скорость поворота во времени.
# Ноль гироскопа плывёт от температуры и тряски — пока робот стоит (колёса не крутятся),
# запоминаем, что показывает гироскоп, и вычитаем: на остановках курс не ползёт.
import math

WHO_AM_I = {0x68: 'MPU-6050', 0x70: 'MPU-6500', 0x71: 'MPU-9250', 0x73: 'MPU-9255', 0x72: 'MPU-6052C'}
# регистр GYRO_CONFIG: диапазон °/с -> (биты FS_SEL, единиц на °/с)
GYRO_RANGE = {250: (0, 131.0), 500: (1, 65.5), 1000: (2, 32.8), 2000: (3, 16.4)}
# регистр CONFIG (DLPF): полоса гироскопа, Гц -> значение
DLPF = {250: 0, 184: 1, 92: 2, 41: 3, 20: 4, 10: 5, 5: 6}


def be16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


class GyroYaw:
    """Курс по одной оси гироскопа. update() раз в такт; still — колёса стоят (из одометрии)."""

    def __init__(self, sign=1.0, scale=1.0, bias_tau_s=2.0):
        self.sign, self.scale, self.tau = sign, scale, bias_tau_s
        self.yaw = 0.0          # рад, против часовой — плюс (как в ROS)
        self.rate = 0.0         # рад/с после вычета нуля
        self.bias = 0.0         # °/с
        self.bias_known = False
        self.still_t = 0.0      # сколько секунд подряд стоим
        self.clipped = 0        # сколько раз гироскоп упирался в предел (удар) — курс мог сбиться

    def update(self, gz_dps, dt, still, clipped=False):
        if clipped:
            self.clipped += 1
        self.still_t = self.still_t + dt if still else 0.0
        if self.still_t > 0.5:
            # стоим полсекунды — всё, что показывает гироскоп, его собственный ноль
            k = 1.0 if not self.bias_known else min(1.0, dt / self.tau)
            self.bias += k * (gz_dps - self.bias)
            self.bias_known = True
            self.rate = 0.0
            return self.yaw
        if not self.bias_known:
            self.rate = 0.0  # ноль ещё не измерен: без него курс уплывал бы на градусы в секунду
            return self.yaw
        self.rate =math.radians(gz_dps - self.bias) * self.sign * self.scale
        self.yaw = math.atan2(math.sin(self.yaw + self.rate * dt), math.cos(self.yaw + self.rate * dt))
        return self.yaw
