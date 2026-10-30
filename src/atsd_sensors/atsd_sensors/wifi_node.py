#!/usr/bin/env python3
# Раз в period_s сканирует Wi-Fi и спрашивает у сервиса, где мы: /wifi/fix (грубо, ±десятки метров).
# Нужен интернет (например, раздача с телефона). Ключ Яндекса — в key_file, без него — beaconDB.
import json
import subprocess
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from atsd_sensors.wifi_logic import URL, Budget, parse_iw, parse_nmcli, parse_response, request_body, \
    same_networks, usable


class WifiNode(Node):

    def __init__(self):
        super().__init__('wifi_node')
        d = self.declare_parameter
        d('iface', 'wlan0')
        d('period_s', 20.0)
        # полное сканирование уводит Wi-Fi с канала на секунды: страницы и видео в это время стоят.
        # Делаем его редко, в промежутках — только готовые списки драйвера и NetworkManager
        d('active_scan_s', 120.0)
        d('key_file', '/etc/atsd/locator.key')
        d('min_aps', 2)
        d('yandex_api', 'yandex')  # yandex — api.lbs.yandex.net, yandex_v1 — новая версия Локатора
        d('daily_limit', 90)  # у Яндекса бесплатно 100 в день; кончились — спрашиваем beaconDB
        d('budget_file', '/opt/atsd/data/locator_budget.json')
        p = lambda n: self.get_parameter(n).value
        self.iface, self.period, self.min_aps = p('iface'), float(p('period_s')), int(p('min_aps'))
        self.active_every, self.last_active = float(p('active_scan_s')), -1e9
        try:
            with open(p('key_file'), encoding='utf-8') as f:
                self.key = f.read().strip()
        except OSError:
            self.key = ''
        self.yandex = str(p('yandex_api'))
        self.provider = self.yandex if self.key else 'beacondb'
        self.budget = Budget(p('budget_file'), int(p('daily_limit')))
        self.last = None  # (точки, место или None) прошлого запроса
        self.yandex_off = 0.0  # до какого времени Яндекс не спрашиваем (ключ отклонён, лимит)
        self.pub_fix = self.create_publisher(NavSatFix, 'wifi/fix', 5)
        self.pub_status = self.create_publisher(String, 'wifi/status', 5)
        self.stop = threading.Event()
        self.last_text = None
        threading.Thread(target=self.loop, daemon=True).start()
        self.get_logger().info(f'Место по Wi-Fi: {self.provider}, раз в {self.period:.0f} с')

    def status(self, text, provider=None):
        self.pub_status.publish(String(data=f'wifi {provider or self.provider}: {text}'))
        if text != self.last_text:  # в журнал — только когда меняется, иначе строка каждые 20 с
            self.get_logger().info(text)
            self.last_text = text

    def scan(self):
        """Точки вокруг и почему список мог быть неполным ('' — всё хорошо)."""
        ap_mode = subprocess.run(['nmcli', '-g', 'GENERAL.CONNECTION', 'dev', 'show', self.iface],
                                 capture_output=True, text=True, timeout=5).stdout.strip() == 'atsd-hotspot'
        now = time.monotonic()
        # своя точка доступа: сканировать нельзя — клиенты (ноутбук оператора) потеряли бы связь
        if not ap_mode and now - self.last_active >= self.active_every:
            self.last_active = now
            try:  # полное сканирование драйвером, от root по правилу /etc/sudoers.d/atsd-wifi-scan
                d = subprocess.run(['sudo', '-n', '/usr/sbin/iw', 'dev', self.iface, 'scan'],
                                   capture_output=True, text=True, timeout=40)
                if d.returncode == 0:
                    return parse_iw(d.stdout), ''
                err = (d.stderr.strip().splitlines() or ['?'])[-1][:80]
            except subprocess.TimeoutExpired:
                err = 'iw scan дольше 40 с'
            except FileNotFoundError:
                err = 'нет iw или sudo'
        else:
            err = 'режим точки доступа — только старый список' if ap_mode else ''
        # между полными сканированиями — то, что драйвер и NetworkManager уже знают (эфир не трогаем)
        r = subprocess.run(['nmcli', '-t', '-f', 'BSSID,SIGNAL,SSID', 'dev', 'wifi', 'list', 'ifname', self.iface,
                            '--rescan', 'no'], capture_output=True, text=True, timeout=10)
        found = parse_nmcli(r.stdout) if r.returncode == 0 else []
        try:
            d = subprocess.run(['iw', 'dev', self.iface, 'scan', 'dump'], capture_output=True, text=True, timeout=10)
            found += parse_iw(d.stdout) if d.returncode == 0 else []
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass
        return found, err

    def locate(self, provider, aps):
        body, ctype = request_body(provider, aps, self.key)
        req = urllib.request.Request(URL[provider].format(key=self.key), data=body.encode(), method='POST',
                                     headers={'Content-Type': ctype,
                                              'User-Agent': 'atsd-1m/1.0 (delivery robot)'})
        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                return parse_response(provider, json.loads(resp.read() or b'{}'))
        except urllib.error.HTTPError as e:
            if e.code == 404:
                return None  # beaconDB: место не найдено
            raise

    def loop(self):
        while not self.stop.wait(self.period):
            provider = None
            try:
                found, err = self.scan()
                aps = usable(found)
                if len(aps) < self.min_aps:
                    why = f'; полного сканирования нет: {err}' if err else ''
                    self.status(f'видно {len(aps)} точек (всего {len(found)}) — мало{why}')
                    continue
                nets = frozenset(b for b, _ in aps[:20])
                if self.last and same_networks(nets, self.last[0]):
                    pos, provider, note = self.last[1], self.last[2], ' (те же сети — без запроса)'
                else:
                    yandex = self.key and time.monotonic() > self.yandex_off
                    provider = self.yandex if yandex and self.budget.take() else 'beacondb'
                    pos, note = self.locate(provider, aps[:20]), ''
                    self.last = (nets, pos, provider)
            except (OSError, subprocess.SubprocessError, ValueError) as e:
                msg = f'{e.__class__.__name__}: {e}'
                if provider == self.yandex and getattr(e, 'code', None) in (403, 429):
                    self.yandex_off = time.monotonic() + 600  # не жжём дневной лимит повторами
                    msg += ' — Яндекс не спрашиваю 10 мин'
                self.status(f'нет ответа ({msg})'[:200], provider)
                continue
            if self.key:
                note += f', Яндекс сегодня {self.budget.used}/{self.budget.limit}'
            if pos is None:
                self.status(f'{len(aps)} точек, место неизвестно{note}', provider)
                continue
            lat, lon, acc = pos
            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = 'wifi'
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.latitude, fix.longitude = lat, lon
            fix.position_covariance[0] = fix.position_covariance[4] = acc ** 2
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            self.pub_fix.publish(fix)
            self.status(f'{len(aps)} точек -> {lat:.6f} {lon:.6f} ±{acc:.0f} м{note}', provider)

    def destroy_node(self):
        self.stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WifiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
