#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

try:
    from smbus2 import SMBus
    I2C_OK = True
except Exception:
    I2C_OK = False


class BatteryNode(Node):

    def __init__(self):
        super().__init__('battery_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', 0x36)
        self.declare_parameter('cells', 4)
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('low_pct', 20.0)

        p = self.get_parameter
        self.bus_id = int(p('i2c_bus').value)
        self.addr = int(p('address').value)
        self.cells = int(p('cells').value)
        self.low_pct = float(p('low_pct').value)

        self.bus = None
        if I2C_OK:
            try:
                self.bus = SMBus(self.bus_id)
            except Exception as e:
                self.get_logger().error(f'Шина I2C {self.bus_id} недоступна: {e}')
        else:
            self.get_logger().warn('Нет smbus2 — узел работает вхолостую. '
                                   'pip3 install --break-system-packages smbus2')

        self.pub = self.create_publisher(BatteryState, 'power/computer', 5)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 5)
        self.create_timer(1.0 / float(p('rate_hz').value), self.tick)

        self.get_logger().info(
            f'Гейдж X1202: шина {self.bus_id}, адрес 0x{self.addr:02X}, '
            f'банок {self.cells}')

    def read_word_swapped(self, reg):
        raw = self.bus.read_word_data(self.addr, reg)
        return ((raw & 0xFF) << 8) | (raw >> 8)

    def tick(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        pct = float('nan')
        volts = float('nan')

        if self.bus is not None:
            try:
                vcell = self.read_word_swapped(0x02)
                soc = self.read_word_swapped(0x04)
                volts = (vcell >> 4) * 1.25 / 1000.0
                pct = min(max(soc / 256.0, 0.0), 100.0)
                msg.present = True
            except Exception as e:
                self.get_logger().warn(f'Чтение гейджа не удалось: {e}',
                                       throttle_duration_sec=10.0)
                msg.present = False
        else:
            msg.present = False

        msg.voltage = volts
        msg.percentage = pct / 100.0 if pct == pct else float('nan')
        self.pub.publish(msg)

        st = DiagnosticStatus()
        st.name = 'atsd/battery_computer'
        st.hardware_id = 'X1202'
        if pct != pct:
            st.level = DiagnosticStatus.WARN
            st.message = 'Нет данных от гейджа'
        elif pct < self.low_pct:
            st.level = DiagnosticStatus.WARN
            st.message = f'Низкий заряд вычислителя: {pct:.0f} %'
        else:
            st.level = DiagnosticStatus.OK
            st.message = f'{pct:.0f} %'
        st.values = [KeyValue(key='percent', value=f'{pct:.1f}'),
                     KeyValue(key='cell_volt', value=f'{volts:.3f}')]

        arr = DiagnosticArray()
        arr.header.stamp = msg.header.stamp
        arr.status = [st]
        self.pub_diag.publish(arr)

    def destroy_node(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BatteryNode()
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
