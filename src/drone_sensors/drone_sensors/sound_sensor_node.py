import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import random
import math

class SoundSensorNode(Node):
    def __init__(self):
        super().__init__('sound_sensor_node')

        self.declare_parameter('sample_rate', 44100)
        self.declare_parameter('simulate', True)

        self.simulate = self.get_parameter('simulate').value

        self.pub = self.create_publisher(Float32, '/sound_level', 10)
        self.timer = self.create_timer(0.1, self.read_sound)

        self.get_logger().info(f'SoundSensorNode started (simulate={self.simulate})')

    def read_sound(self):
        if self.simulate:
            # Stable room noise ~40dB with small variations
            db = random.gauss(40.0, 2.0)
            if random.random() < 0.0001:  # 1% chance of loud spike
                db = random.uniform(82.0, 90.0)
            db = max(0.0, db)
        else:
            try:
                import pyaudio
                # Real microphone reading would go here
                db = 0.0
            except ImportError:
                self.get_logger().warn('PyAudio not installed, using simulation')
                db = random.gauss(45.0, 8.0)

        msg = Float32()
        msg.data = float(db)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SoundSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
