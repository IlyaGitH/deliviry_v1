#!/usr/bin/env python3
import asyncio
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from atsd_sensors.ble_logic import BeaconRule, ZoneTracker

TRAVEL = ('TO_PICKUP', 'TO_DROP', 'RETURN')

try:
    from bleak import BleakScanner
    BLEAK_OK = True
except Exception:
    BLEAK_OK = False


class BleNode(Node):

    def __init__(self):
        super().__init__('ble_node')
        d = self.declare_parameter
        d('point_ids', ['kpp', 'admin', 'testing', 'depot'])
        d('names', ['ATSD-KPP', 'ATSD-ADMIN', 'ATSD-TEST', 'ATSD-SKLAD'])
        d('ibeacon_uuid', 'E2C56DB5-DFFB-48D2-B060-D0F5A71096E0')
        d('ibeacon_minors', [1, 2, 3, 4])
        d('near_rssi', -65)
        d('hysteresis', 5)
        d('lost_s', 4.0)
        d('rate_hz', 2.0)
        d('adapter', 'hci0')
        # Wi-Fi и Bluetooth в Pi — один чип: непрерывный поиск меток отнимает эфир у Wi-Fi (веб тормозит).
        # Постоянно ищем только у точки: на погрузке/выдаче и в последних near_m рейса
        d('near_m', 25.0)
        d('idle_scan_s', 3.0)
        d('idle_pause_s', 27.0)
        p = lambda n: self.get_parameter(n).value

        ids, names, minors = list(p('point_ids')), list(p('names')), list(p('ibeacon_minors'))
        rules = [BeaconRule(pid, names[i] if i < len(names) else '', int(minors[i]) if i < len(minors) else -1)
                 for i, pid in enumerate(ids)]
        self.tracker = ZoneTracker(rules, str(p('ibeacon_uuid')), int(p('near_rssi')),
                                   int(p('hysteresis')), float(p('lost_s')))
        self.adapter = str(p('adapter'))
        self.lock = threading.Lock()
        self.last_seen_any = 0.0

        self.near_m, self.idle_scan, self.idle_pause = float(p('near_m')), float(p('idle_scan_s')), \
            float(p('idle_pause_s'))
        self.state, self.remain, self.remain_t = None, None, 0.0
        self.create_subscription(String, 'mission/state', self.on_state, 10)
        self.create_subscription(Float32, 'nav/remain', self.on_remain, 10)
        self.pub_zone = self.create_publisher(String, 'ble/zone', 10)
        self.pub_zones = self.create_publisher(String, 'ble/zones', 10)
        self.create_timer(1.0 / float(p('rate_hz')), self.publish)

        if not BLEAK_OK:
            self.get_logger().error('Нет bleak: pip3 install --break-system-packages bleak')
            return
        self.running = True
        threading.Thread(target=lambda: asyncio.run(self.scan()), daemon=True).start()
        self.get_logger().info('BLE: ' + ', '.join(f'{r.point}={r.name}/minor {r.minor}' for r in rules))

    def on_adv(self, device, adv):
        name = adv.local_name or device.name
        with self.lock:
            if self.tracker.update(name, adv.manufacturer_data, adv.rssi, time.monotonic()):
                self.last_seen_any = time.monotonic()

    def on_state(self, msg: String):
        try:
            self.state = json.loads(msg.data).get('state')
        except (ValueError, AttributeError):
            pass

    def on_remain(self, msg: Float32):
        self.remain, self.remain_t = msg.data, time.monotonic()

    def need_now(self):
        """Метки нужны сейчас: стоим на точке или подъезжаем к ней."""
        if self.state in ('WAIT_LOAD', 'WAIT_UNLOAD'):
            return True
        return self.state in TRAVEL and self.remain is not None and \
            time.monotonic() - self.remain_t < 2.0 and self.remain < self.near_m

    async def scan(self):
        while self.running:
            try:
                async with BleakScanner(detection_callback=self.on_adv, adapter=self.adapter):
                    t0 = time.monotonic()
                    # у точки — слушаем, пока нужно; иначе — коротко, чтобы веб видел, какие метки рядом
                    while self.running and (self.need_now() or time.monotonic() - t0 < self.idle_scan):
                        await asyncio.sleep(0.5)
                t0 = time.monotonic()
                while self.running and not self.need_now() and time.monotonic() - t0 < self.idle_pause:
                    await asyncio.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f'Сканер BLE: {e}. Повтор через 5 с', throttle_duration_sec=30.0)
                await asyncio.sleep(5.0)

    def publish(self):
        now = time.monotonic()
        with self.lock:
            zones = self.tracker.zones(now)
            best = self.tracker.best(now)
        self.pub_zones.publish(String(data=json.dumps(zones)))
        self.pub_zone.publish(String(data=f'{best[0]} {best[1]} {int(best[2])}' if best else 'none'))

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BleNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
