import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient


class ModbusMotorController(Node):
    def __init__(self):
        super().__init__('modbus_motor_controller')

        self.client = ModbusSerialClient(
            port='/dev/ttyUSB0',
            baudrate=9600,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=1
        )

        if not self.client.connect():
            self.get_logger().error(' Failed to connect to Modbus device.')
            rclpy.shutdown()
            return

        self.slave_ids = [1, 2]  # Control both motors
        self.register_address = 124  # 40125 - 40001
        self.control_loop()

    def enable_motor(self):
        self.write_to_all(159, 'Motor enabled.')  # 0x9F

    def disable_motor(self):
        self.write_to_all(158, 'Motor disabled.')  # 0x9E

    def start_jog(self):
        self.write_to_all(150, 'Start Jogging (CJ sent).')  # 0x96

    def stop_jog(self):
        self.write_to_all(216, 'Stop Jogging (SJ sent).')  # 0xD8

    def write_to_all(self, value, message):
        for sid in self.slave_ids:
            result = self.client.write_register(self.register_address, value, slave=sid)
            if result.isError():
                self.get_logger().error(f"Failed to write {value} to register {self.register_address} for motor {sid}.")
            else:
                self.get_logger().info(f"{message} (Motor {sid})")

    def control_loop(self):
        while rclpy.ok():
            cmd = input("Enter command (enable / disable / start_jog / stop_jog / exit): ").strip().lower()

            if cmd == "enable":
                self.enable_motor()
            elif cmd == "disable":
                self.disable_motor()
            elif cmd == "start_jog":
                self.start_jog()
            elif cmd == "stop_jog":
                self.stop_jog()
            elif cmd == "exit":
                self.get_logger().info(" Exiting...")
                break
            else:
                self.get_logger().warn(" Invalid command. Use 'enable', 'disable', 'start_jog', 'stop_jog', or 'exit'.")


def main(args=None):
    rclpy.init(args=args)
    node = ModbusMotorController()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
