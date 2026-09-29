#!/usr/bin/env python3
import argparse
import asyncio
import os
import subprocess
import sys
import time
from pathlib import Path

for base in (Path('/opt/atsd/ws/src'), Path(__file__).resolve().parents[2]):
    for pkg in ('atsd_perception', 'atsd_sensors'):
        if (base / pkg).is_dir():
            sys.path.insert(0, str(base / pkg))

OK, BAD, WARN = '\033[1;32mOK\033[0m', '\033[1;31mНЕТ\033[0m', '\033[1;33m!!\033[0m'
results = []


def report(name, ok, detail=''):
    mark = OK if ok is True else (WARN if ok is None else BAD)
    print(f'  [{mark}] {name:<28} {detail}')
    results.append((name, ok))


def link(path):
    if not os.path.exists(path):
        return None
    return os.path.realpath(path)


def check_services():
    try:
        out = subprocess.run(['systemctl', 'is-active', 'atsd-drive', 'atsd-sensors',
                              'atsd-actuators', 'atsd-perception'],
                             capture_output=True, text=True).stdout.split()
        if 'active' in out:
            print(f'{WARN} Сервисы ATSD запущены и держат порты. Сначала: sudo systemctl stop atsd.target\n')
    except FileNotFoundError:
        pass


def check_v5():
    import serial
    dev = link('/dev/v5')
    if dev is None:
        report('VEX V5 /dev/v5', False, 'нет симлинка: Brain выключен или не тот кабель/порт')
        return
    try:
        s = serial.Serial('/dev/v5', 115200, timeout=0.1)
        t_drain = time.time()
        while time.time() - t_drain < 0.5:
            s.read(s.in_waiting or 1)
        t0, info, e_lines, events = time.time(), None, 0, []
        buf, last_q = b'', 0.0
        while time.time() - t0 < 3.0 and not (info and e_lines > 100):
            if info is None and time.time() - last_q > 0.4:
                s.write(b'?\n')
                last_q = time.time()
            buf += s.read(s.in_waiting or 1)
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', 'ignore').strip()
                if text.startswith('I '):
                    info = text[2:]
                elif text.startswith('E '):
                    e_lines += 1
                    last_e = text
                elif text.startswith('K '):
                    events.append(text[2:])
        s.close()
        if info:
            report('VEX V5 прошивка', info.startswith('ATSD-1M 2.'), f'{info} ({dev})')
        else:
            report('VEX V5 прошивка', False, 'ответа нет: программа на Brain не запущена?')
        rate = e_lines / max(time.time() - t0, 0.1)
        report('VEX V5 телеметрия', rate > 30, f'{rate:.0f} строк E в секунду (ждём ~50)')
        if e_lines:
            parts = last_e.split()
            mv, flags = int(parts[5]), int(parts[6])
            report('VEX V5 батарея', mv > 11000, f'{mv / 1000:.2f} В, флаги 0x{flags:04X}')
    except Exception as e:
        report('VEX V5 /dev/v5', False, str(e))


def check_lidar():
    import serial
    dev = link('/dev/lidar')
    if dev is None:
        report('Лидар /dev/lidar', False, 'нет симлинка: переходник CP2102 не определился')
        return
    try:
        s = serial.Serial('/dev/lidar', 230400, timeout=0.1)
        s.reset_input_buffer()
        t0, data = time.time(), b''
        while time.time() - t0 < 1.0:
            data += s.read(4096)
        s.close()
        frames = data.count(b'\x54\x2c')
        report('Лидар LD19', frames > 200, f'{frames} кадров/с (ждём ~450), {dev}')
    except Exception as e:
        report('Лидар /dev/lidar', False, str(e))


def check_gnss():
    import serial
    dev = link('/dev/gnss')
    if dev is None:
        report('GNSS /dev/gnss', False, 'нет симлинка: проверьте dtparam=uart0=on и udev')
        return
    for baud in (115200, 9600, 38400):
        try:
            s = serial.Serial('/dev/gnss', baud, timeout=0.2)
            s.reset_input_buffer()
            t0, data = time.time(), b''
            while time.time() - t0 < 1.5:
                data += s.read(1024)
            s.close()
            nmea = data.count(b'$G')
            if nmea:
                fix = b'GGA,' in data
                report('GNSS BN-880', True, f'{nmea} NMEA на {baud} бод, {dev}'
                       + ('' if fix else ' (GGA не пришла)'))
                return
        except Exception as e:
            report('GNSS /dev/gnss', False, str(e))
            return
    report('GNSS BN-880', False, 'NMEA нет: TX/RX перепутаны или консоль на serial0')


def check_camera():
    dev = link('/dev/camera')
    if dev is None:
        report('Камера /dev/camera', False, 'нет симлинка')
        return
    try:
        import cv2
        cap = cv2.VideoCapture('/dev/camera', cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ok, frame = False, None
        for _ in range(10):
            ok, frame = cap.read()
            if ok:
                break
        cap.release()
        report('Камера', ok, f'{frame.shape[1]}x{frame.shape[0]}, {dev}' if ok else 'кадр не прочитан')
        if ok:
            try:
                from atsd_perception.vision_logic import CameraGeom, find_red_spot
                spot = find_red_spot(frame, CameraGeom())
                report('Красная разметка', True if spot else None,
                       f'x {spot["x"]:.2f} м, y {spot["y"]:+.2f} м, {spot["area"]:.2f} м²' if spot
                       else 'не видна (положите пятно в 1–2 м перед роботом)')
            except ImportError:
                report('Красная разметка', None, 'нет atsd_perception, пропускаю')
            model_dir = Path('/opt/atsd/models/signs_ncnn_model')
            if not model_dir.exists():
                report('Модель знаков', None, f'нет {model_dir}')
            else:
                try:
                    from atsd_perception.ncnn_yolo import NcnnYolo
                    model = NcnnYolo(model_dir, threads=3)
                    model.predict(frame, conf=0.4)
                    t0 = time.time()
                    dets = model.predict(frame, conf=0.4)
                    ms = (time.time() - t0) * 1000
                    found = ', '.join(f'{d["cls"]} {d["conf"]:.2f}' for d in dets) or 'в кадре ничего'
                    report('Модель знаков (ncnn)', True, f'{ms:.0f} мс на кадр, классы {list(model.names.values())}; {found}')
                except Exception as e:
                    report('Модель знаков (ncnn)', False, str(e))
    except ImportError:
        report('Камера', None, 'нет python3-opencv, пропускаю')


def check_ble(seconds=8.0):
    try:
        from bleak import BleakScanner
        from atsd_sensors.ble_logic import BeaconRule, ZoneTracker
    except ImportError as e:
        report('BLE-метки', None, f'пропускаю: {e}')
        return
    ids = ['kpp', 'admin', 'testing', 'depot']
    names = ['ATSD-KPP', 'ATSD-ADMIN', 'ATSD-TEST', 'ATSD-SKLAD']
    tracker = ZoneTracker([BeaconRule(i, n, k + 1) for k, (i, n) in enumerate(zip(ids, names))],
                          'E2C56DB5-DFFB-48D2-B060-D0F5A71096E0')
    total = [0]

    def cb(device, adv):
        total[0] += 1
        tracker.update(adv.local_name or device.name, adv.manufacturer_data, adv.rssi, time.monotonic())

    async def run():
        async with BleakScanner(detection_callback=cb):
            await asyncio.sleep(seconds)
    try:
        asyncio.run(run())
    except Exception as e:
        report('Bluetooth', False, f'{e} — dtoverlay=disable-bt в config.txt? rfkill?')
        return
    report('Bluetooth', total[0] > 0, f'{total[0]} пакетов за {seconds:.0f} с')
    zones = tracker.zones(time.monotonic())
    for pid in ids:
        z = zones.get(pid)
        report(f'Метка {pid}', True if z else None,
               f'{z["rssi"]} дБм{" · рядом" if z["near"] else ""}' if z else 'не найдена')


def check_gauge():
    try:
        from smbus2 import SMBus
    except ImportError:
        report('Гейдж X1202', None, 'нет smbus2, пропускаю')
        return
    try:
        with SMBus(1) as bus:
            sw = lambda r: (lambda raw: ((raw & 0xFF) << 8) | (raw >> 8))(bus.read_word_data(0x36, r))
            volts = (sw(0x02) >> 4) * 1.25 / 1000.0
            pct = sw(0x04) / 256.0
        report('Гейдж X1202 (I2C 0x36)', True, f'{pct:.0f} %, {volts:.2f} В на банку')
    except Exception as e:
        report('Гейдж X1202 (I2C 0x36)', False, f'{e} — плата сидит на pogo-контактах?')


def check_gpio(with_lock):
    os.environ.setdefault('GPIOZERO_PIN_FACTORY', 'lgpio')
    try:
        from gpiozero import Button, DigitalOutputDevice, PWMLED
    except ImportError:
        report('GPIO', False, 'нет gpiozero/lgpio')
        return

    def ask(q):
        try:
            return input(f'      {q} [Enter — да, n — нет] ').strip().lower() != 'n'
        except EOFError:
            return None

    for name, pin in (('Фары, канал 1', 18), ('Лента, канал 2', 13)):
        try:
            led = PWMLED(pin, frequency=200)
        except Exception as e:
            report(name, False, f'GPIO{pin} занят ({e}) — остановите сервисы: sudo systemctl stop atsd.target')
            return
        print(f'  {name}: GPIO{pin} плавно 0 → 50 % → 0')
        for v in [i / 20 for i in range(11)] + [i / 20 for i in range(10, -1, -1)]:
            led.value = v
            time.sleep(0.05)
        led.close()
        report(name, ask('Загорелось?'), f'GPIO{pin}')

    bz = DigitalOutputDevice(26, initial_value=False)
    print('  Зуммер, канал 4: GPIO26, два коротких сигнала')
    for _ in range(2):
        bz.on(); time.sleep(0.12); bz.off(); time.sleep(0.12)
    bz.close()
    report('Зуммер, канал 4', ask('Пискнуло?'), 'GPIO26')

    door = Button(27, pull_up=True, bounce_time=0.05)
    time.sleep(0.1)
    report('Геркон крышки', None, f'GPIO27: {"замкнут (крышка закрыта)" if door.is_pressed else "разомкнут"}'
           ' — если геркона нет, это нормально')
    door.close()

    if with_lock:
        lock = DigitalOutputDevice(19, initial_value=False)
        print('  Замок, канал 3: GPIO19, импульс 0,4 с')
        lock.on(); time.sleep(0.4); lock.off()
        lock.close()
        report('Замок, канал 3', ask('Щёлкнул, крышка отскочила?'), 'GPIO19')
    else:
        report('Замок, канал 3', None, 'пропущен, запустите с --lock')


def main():
    ap = argparse.ArgumentParser(description='АТСД-1М · проверка железа без ROS. '
                                             'Перед запуском: sudo systemctl stop atsd.target')
    ap.add_argument('--lock', action='store_true', help='подать импульс на замок')
    ap.add_argument('--no-gpio', action='store_true', help='не трогать GPIO')
    a = ap.parse_args()

    print('\nАТСД-1М · проверка железа\n')
    check_services()
    try:
        import serial
    except ImportError:
        print('Нет pyserial: sudo apt install python3-serial')
        sys.exit(1)

    print('USB и последовательные порты')
    check_v5()
    check_lidar()
    check_camera()
    check_gnss()
    print('\nBluetooth')
    check_ble()
    print('\nI2C')
    check_gauge()
    if not a.no_gpio:
        print('\nGPIO')
        check_gpio(a.lock)

    bad = [n for n, ok in results if ok is False]
    print('\nИтог: ' + ('всё в порядке' if not bad else 'проблемы: ' + ', '.join(bad)) + '\n')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
