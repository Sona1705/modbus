import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient

class ModbusMotorController(Node):
    def __init__(self):
        super().__init__('modbus_motor_controller')

        # Set up Modbus RTU client
        self.client = ModbusSerialClient(
            method='rtu',
            port='/dev/ttyUSB0',  # adjust based on your setup
            baudrate=115200,
            timeout=1
        )

        if self.client.connect():
            self.get_logger().info('Modbus client connected successfully.')
        else:
            self.get_logger().error('Failed to connect to Modbus device.')

        # Example: Read holding register at address 0x0001
        response = self.client.read_holding_registers(0x0001, 1, unit=1)
        if response.isError():
            self.get_logger().error('Read error')
        else:
            self.get_logger().info(f'Register Value: {response.registers[0]}')

        # Stop the node after one read
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ModbusMotorController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
