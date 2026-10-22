import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

class BatteryManagerNode(Node):
    def __init__(self):
        super().__init__('battery_manager_node')

        self.declare_parameter('recharge_threshold', 20.0)
        self.declare_parameter('dock_x', 0.0)
        self.declare_parameter('dock_y', 0.0)
        self.declare_parameter('dock_z', 0.0)

        self.recharge_threshold = self.get_parameter('recharge_threshold').value
        self.dock_x = self.get_parameter('dock_x').value
        self.dock_y = self.get_parameter('dock_y').value
        self.dock_z = self.get_parameter('dock_z').value

        self.is_charging = False

        self.create_subscription(BatteryState, '/battery', self.battery_callback, 10)

        self.land_pub   = self.create_publisher(Bool, '/drone/land', 10)
        self.goal_pub   = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.status_pub = self.create_publisher(String, '/battery_status', 10)

        self.get_logger().info(f'BatteryManagerNode started. Recharge threshold: {self.recharge_threshold}%')

    def battery_callback(self, msg):
        pct = msg.percentage * 100.0

        status = String()
        status.data = f'Battery: {pct:.1f}% | Charging: {self.is_charging}'
        self.status_pub.publish(status)

        if pct < self.recharge_threshold and not self.is_charging:
            self.get_logger().warn(f'Battery at {pct:.1f}% — initiating auto-recharge')
            self._go_to_dock()

        if pct > 90.0 and self.is_charging:
            self.is_charging = False
            self.get_logger().info(f'Battery charged ({pct:.1f}%) — resuming patrol')

    def _go_to_dock(self):
        self.is_charging = True

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.dock_x
        goal.pose.position.y = self.dock_y
        goal.pose.position.z = self.dock_z
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        land = Bool()
        land.data = True
        self.land_pub.publish(land)
        self.get_logger().info('Landing command sent — drone docking for recharge')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
