import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pymodbus.client import ModbusSerialClient


class ModbusJoystickController(Node):
    def __init__(self):
        super().__init__('modbus_joystick_controller')

        # Modbus setup
        self.client = ModbusSerialClient(
            port='/dev/ttyUSB0',  # <-- Change this if your USB device is different (e.g., /dev/ttyUSB1)
            baudrate=9600,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=1
        )

        if not self.client.connect():
            self.get_logger().error("Failed to connect to Modbus device.")
            raise RuntimeError("Modbus connection failed")

        # Slave IDs for both motor drivers
        self.slave_ids = [1, 2]
        self.register_address = 124  # 40125 - 40001
        self.last_cmd = None

        # Subscribe to joystick messages
        self.subscription = self.create_subscription(Joy, '/joy', self.joystick_callback, 10)

    def joystick_callback(self, msg: Joy):
        axis_y = msg.axes[1]
        btn_enable = msg.buttons[0]
        btn_disable = msg.buttons[1]

        # Motor enable/disable
        if btn_enable:
            self.send_command(159, "Motors Enabled")
        elif btn_disable:
            self.send_command(158, "Motors Disabled")

        # Jog control
        if axis_y > 0.5:
            if self.last_cmd != 'forward':
                self.send_command(150, "Jog Forward")
                self.last_cmd = 'forward'
        elif axis_y < -0.5:
            if self.last_cmd != 'reverse':
                self.send_command(151, "Jog Reverse")
                self.last_cmd = 'reverse'
        else:
            if self.last_cmd not in [None, 'stop']:
                self.send_command(216, "Stop Jog")
                self.last_cmd = 'stop'

    def send_command(self, value, log_msg):
        for sid in self.slave_ids:
            result = self.client.write_register(self.register_address, value, slave=sid)
            if result.isError():
                self.get_logger().error(f"Failed to write {value} to motor {sid}")
            else:
                self.get_logger().info(f"{log_msg} (Motor {sid})")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ModbusJoystickController()
    except RuntimeError:
        rclpy.shutdown()
        return
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
