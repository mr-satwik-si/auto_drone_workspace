import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class PatrolPlannerNode(Node):
    def __init__(self):
        super().__init__('patrol_planner_node')

        self.declare_parameter('waypoints', [
            1.0, 0.0, 1.2,
            1.0, 2.0, 1.2,
            3.0, 2.0, 1.2,
            3.0, 0.0, 1.2,
        ])

        raw = self.get_parameter('waypoints').value
        self.waypoints = [
            [raw[i], raw[i+1], raw[i+2]]
            for i in range(0, len(raw), 3)
        ]

        self.current_wp = 0
        self.paused = False

        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.create_subscription(Bool, '/drone/land', self.pause_callback, 10)

        self.timer = self.create_timer(5.0, self.patrol_loop)
        self.get_logger().info(f'PatrolPlannerNode started with {len(self.waypoints)} waypoints')

    def pause_callback(self, msg):
        self.paused = msg.data
        if self.paused:
            self.get_logger().info('Patrol paused — drone recharging')
        else:
            self.get_logger().info('Patrol resumed')

    def patrol_loop(self):
        if self.paused:
            return

        wp = self.waypoints[self.current_wp]

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = wp[0]
        goal.pose.position.y = wp[1]
        goal.pose.position.z = wp[2]
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info(f'Navigating to waypoint {self.current_wp}: {wp}')

        self.current_wp = (self.current_wp + 1) % len(self.waypoints)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
