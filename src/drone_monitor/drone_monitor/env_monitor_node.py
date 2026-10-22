#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class EnvMonitorNode(Node):
    def __init__(self):
        super().__init__('env_monitor_node')

        # Still subscribe to sensors for dashboard display
        # but NO alerts for temperature or sound
        self.create_subscription(Float32, '/temperature', self.temp_callback, 10)
        self.create_subscription(Float32, '/sound_level', self.sound_callback, 10)

        self.get_logger().info(
            'EnvMonitorNode started — sensor data flowing to dashboard only')

    def temp_callback(self, msg):
        # Just log — no alerts
        self.get_logger().debug(f'Temperature: {msg.data:.1f}C')

    def sound_callback(self, msg):
        # Just log — no alerts
        self.get_logger().debug(f'Sound: {msg.data:.1f}dB')

def main(args=None):
    rclpy.init(args=args)
    node = EnvMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()