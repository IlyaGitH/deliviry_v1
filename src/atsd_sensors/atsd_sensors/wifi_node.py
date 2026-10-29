#!/usr/bin/env python3
# Раз в period_s сканирует Wi-Fi и спрашивает у сервиса, где мы: /wifi/fix (грубо, ±десятки метров).
# Нужен интернет (например, раздача с телефона). Ключ Яндекса — в key_file, без него — beaconDB.
import json
import subprocess
import threading
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from atsd_sensors.wifi_logic import URL, parse_iw, parse_nmcli, parse_response, request_body, usable


class WifiNode(Node):

    def __init__(self):
        super().__init__('wifi_node')
        d = self.declare_parameter
        d('iface', 'wlan0')
        d('period_s', 20.0)
        d('key_file', '/etc/atsd/locator.key')
        d('min_aps', 2)
        p = lambda n: self.get_parameter(n).value
        self.iface, self.period, self.min_aps = p('iface'), float(p('period_s')), int(p('min_aps'))
        try:
            with open(p('key_file'), encoding='utf-8') as f:
                self.key = f.read().strip()
        except OSError:
            self.key = ''
        self.provider = 'yandex' if self.key else 'beacondb'
        self.pub_fix = self.create_publisher(NavSatFix, 'wifi/fix', 5)
        self.pub_status = self.create_publisher(String, 'wifi/status', 5)
        self.stop = threading.Event()
        self.last_text = None
        threading.Thread(target=self.loop, daemon=True).start()
        self.get_logger().info(f'Место по Wi-Fi: {self.provider}, раз в {self.period:.0f} с')

    def status(self, text):
        self.pub_status.publish(String(data=f'wifi {self.provider}: {text}'))
        if text != self.last_text:  # в журнал — только когда меняется, иначе строка каждые 20 с
            self.get_logger().info(text)
            self.last_text = text

    def scan(self):
        cmd = ['nmcli', '-t', '-f', 'BSSID,SIGNAL,SSID', 'dev', 'wifi', 'list', 'ifname', self.iface]
        r = subprocess.run(cmd + ['--rescan', 'yes'], capture_output=True, text=True, timeout=20)
        err = ''
        if r.returncode != 0:
            # не дали сканировать (нет правила polkit) или режим точки доступа — берём кэш
            err = (r.stderr.strip().splitlines() or ['?'])[-1][:80]
            r = subprocess.run(cmd + ['--rescan', 'no'], capture_output=True, text=True, timeout=20)
        found = parse_nmcli(r.stdout) if r.returncode == 0 else []
        try:  # драйвер слышит больше, чем показывает NetworkManager
            d = subprocess.run(['iw', 'dev', self.iface, 'scan', 'dump'], capture_output=True, text=True, timeout=10)
            found += parse_iw(d.stdout) if d.returncode == 0 else []
        except FileNotFoundError:
            pass  # iw не установлен
        return found, err

    def locate(self, aps):
        req = urllib.request.Request(URL[self.provider].format(key=self.key),
                                     data=request_body(self.provider, aps).encode(), method='POST',
                                     headers={'Content-Type': 'application/json',
                                              'User-Agent': 'atsd-1m/1.0 (delivery robot)'})
        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                return parse_response(self.provider, json.loads(resp.read() or b'{}'))
        except urllib.error.HTTPError as e:
            if e.code == 404:
                return None  # beaconDB: место не найдено
            raise

    def loop(self):
        while not self.stop.wait(self.period):
            try:
                found, err = self.scan()
                aps = usable(found)
                if len(aps) < self.min_aps:
                    why = f'; сканировать не дали: {err} — старый список' if err else ''
                    self.status(f'видно {len(aps)} точек (всего {len(found)}) — мало{why}')
                    continue
                pos = self.locate(aps[:20])
            except (OSError, subprocess.SubprocessError, ValueError) as e:
                self.status(f'нет ответа ({e.__class__.__name__}: {e})'[:160])
                continue
            if pos is None:
                self.status(f'{len(aps)} точек, место неизвестно')
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
            self.status(f'{len(aps)} точек -> {lat:.6f} {lon:.6f} ±{acc:.0f} м')

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
