import rclpy
from rclpy.node import Node
import smbus2
import bme280
from std_msgs.msg import Float32

class BME280I2CNode(Node):
    def __init__(self):
        super().__init__('bme280_i2c_reader')

        # Create separate publishers for each sensor reading under /sensors namespace
        self.temp_publisher_ = self.create_publisher(Float32, '/sensors/temperature', 10)
        self.humidity_publisher_ = self.create_publisher(Float32, '/sensors/humidity', 10)
        self.pressure_publisher_ = self.create_publisher(Float32, '/sensors/pressure', 10)

        # I2C setup
        self.port = 1  # I2C port (usually 1 for Raspberry Pi)
        self.address = 0x76  # BME280 I2C address (0x76 or 0x77)

        try:
            self.bus = smbus2.SMBus(self.port)
            # Initialize BME280 sensor using Pimoroni library
            self.sensor = bme280.BME280(i2c_addr=self.address, i2c_dev=self.bus)
            self.get_logger().info(f"Connected to BME280 via I2C on port {self.port}, address 0x{self.address:02x}")
        except Exception as e:
            self.get_logger().error(f"I2C error: {e}")
            exit(1)

        self.timer = self.create_timer(2.0, self.read_sensor)

    def read_sensor(self):
        try:
            # Read sensor data using Pimoroni library methods
            temperature = self.sensor.get_temperature()
            humidity = self.sensor.get_humidity()
            pressure = self.sensor.get_pressure()

            # Publish temperature
            temp_msg = Float32()
            temp_msg.data = temperature
            self.temp_publisher_.publish(temp_msg)

            # Publish humidity
            hum_msg = Float32()
            hum_msg.data = humidity
            self.humidity_publisher_.publish(hum_msg)

            # Publish pressure
            pres_msg = Float32()
            pres_msg.data = pressure
            self.pressure_publisher_.publish(pres_msg)

            self.get_logger().info(f"Published - Temp: {temperature:.2f}°C, Humidity: {humidity:.2f}%, Pressure: {pressure:.2f}hPa")

        except Exception as e:
            self.get_logger().warn(f"Failed to read sensor: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BME280I2CNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
