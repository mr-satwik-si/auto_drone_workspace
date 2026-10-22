import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TempSensorNode(Node):
    def __init__(self):
        super().__init__('temp_sensor_node')

        self.declare_parameter('simulate', True)
        self.simulate = self.get_parameter('simulate').value

        self.pub = self.create_publisher(Float32, '/temperature', 10)
        self.timer = self.create_timer(1.0, self.read_temp)

        self.get_logger().info(f'TempSensorNode started (simulate={self.simulate})')

    def read_temp(self):
        if self.simulate:
            # Stable room temperature ~25°C with small variations
            temp = random.gauss(25.0, 0.5)
            if random.random() < 0.0001:  # 1% chance of heat spike
                temp = random.uniform(36.0, 40.0)
            temp = max(0.0, temp)
        else:
            try:
                import board
                import adafruit_dht
                dht = adafruit_dht.DHT22(board.D4)
                temp = dht.temperature
            except Exception as e:
                self.get_logger().warn(f'Sensor read failed: {e}, using simulation')
                temp = random.gauss(28.0, 2.0)

        msg = Float32()
        msg.data = float(temp)
        self.pub.publish(msg)
        self.get_logger().info(f'Temperature: {temp:.1f}C')

def main(args=None):
    rclpy.init(args=args)
    node = TempSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
