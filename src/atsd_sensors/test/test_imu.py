import math

from atsd_sensors.imu_logic import GyroYaw, be16


def test_be16():
    assert be16(0x7F, 0xFF) == 32767 and be16(0x80, 0x00) == -32768 and be16(0xFF, 0xFF) == -1


def test_bias_learned_at_rest_and_removed_while_driving():
    g = GyroYaw()
    dt = 0.005
    for _ in range(200):                      # стоим 1 с, гироскоп врёт на +1,5 °/с
        g.update(1.5, dt, still=True)
    assert g.bias_known and abs(g.bias - 1.5) < 1e-6
    assert g.yaw == 0.0                       # на стоянке курс не ползёт
    for _ in range(2000):                     # 10 с поворачиваем на 9 °/с (+ тот же ноль)
        g.update(9.0 + 1.5, dt, still=False)
    assert abs(math.degrees(g.yaw) - 90.0) < 0.5


def test_sign_and_clip_counter():
    g = GyroYaw(sign=-1.0)
    for _ in range(200):
        g.update(0.0, 0.005, still=True)
    for _ in range(200):
        g.update(10.0, 0.005, still=False, clipped=True)
    assert math.degrees(g.yaw) < -9.0 and g.clipped == 200
