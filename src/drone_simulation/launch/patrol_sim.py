#!/usr/bin/env python3
import time
import threading
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
import subprocess

DOCK_X = 0.0
DOCK_Y = 0.0
DOCK_Z = 0.05
FLY_Z  = 1.2
STEP_DELAY = 0.1

RECHARGE_THRESHOLD = 0.30
FULL_CHARGE        = 1.00

wall_waypoints = [
    (0.2, 0.2),
    (0.5, 0.2),
    (1.0, 0.2),
    (1.5, 0.2),
    (2.0, 0.2),
    (2.5, 0.2),
    (3.0, 0.2),
    (3.5, 0.2),
    (3.8, 0.2),
    (3.8, 0.5),
    (3.8, 1.0),
    (3.8, 1.5),
    (3.8, 2.0),
    (3.8, 2.5),
    (3.8, 3.0),
    (3.8, 3.5),
    (3.8, 3.8),
    (3.5, 3.8),
    (3.0, 3.8),
    (2.5, 3.8),
    (2.0, 3.8),
    (1.5, 3.8),
    (1.0, 3.8),
    (0.5, 3.8),
    (0.2, 3.8),
    (0.2, 3.5),
    (0.2, 3.0),
    (0.2, 2.5),
    (0.2, 2.0),
    (0.2, 1.5),
    (0.2, 1.0),
    (0.2, 0.5),
    (0.2, 0.2),
]

pos_lock = threading.Lock()
drone_x  = DOCK_X
drone_y  = DOCK_Y
drone_z  = DOCK_Z

def move_drone_fast(x, y, z):
    req = f'name: "drone", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}'
    cmd = [
        'gz', 'service',
        '-s', '/world/room/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '500',
        '--req', req
    ]
    try:
        subprocess.run(cmd, timeout=0.6,
                      stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL)
    except:
        pass

class Drone1PatrolNode(Node):
    def __init__(self):
        super().__init__('drone_patrol_node')

        self.pose_pub    = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.land_pub    = self.create_publisher(Bool, '/drone/land', 10)
        self.status_pub  = self.create_publisher(String, '/drone1/status', 10)

        self.battery_pct  = 1.0
        self.is_charging  = False
        self.is_flying    = False
        self.patrol_count = 0

        # Listen to drone2 status
        self.drone2_on_dock = True
        self.create_subscription(
            String, '/drone2/status', self.drone2_status_cb, 10)

        self.create_timer(0.5,  self.battery_tick)
        self.create_timer(0.05, self.publish_pose)

        self.get_logger().info('Drone1PatrolNode ready')

    def drone2_status_cb(self, msg):
        self.drone2_on_dock = msg.data == 'on_dock'

    def battery_tick(self):
        if self.is_charging:
            # Charge only when on dock
            self.battery_pct = min(1.0, self.battery_pct + 0.01)
        elif self.is_flying:
            # Drain only when flying
            self.battery_pct = max(0.0, self.battery_pct - 0.002)
        # If on dock and not charging — battery stays the same

        msg = BatteryState()
        msg.percentage = self.battery_pct
        self.battery_pub.publish(msg)

    def publish_pose(self):
        global drone_x, drone_y, drone_z
        with pos_lock:
            x, y, z = drone_x, drone_y, drone_z
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def set_pos(self, x, y, z):
        global drone_x, drone_y, drone_z
        with pos_lock:
            drone_x, drone_y, drone_z = x, y, z

    def needs_recharge(self):
        return self.battery_pct < RECHARGE_THRESHOLD and not self.is_charging

    def is_fully_charged(self):
        return self.battery_pct >= FULL_CHARGE

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    def signal_landing(self):
        self.is_flying = False
        msg = Bool()
        msg.data = True
        self.land_pub.publish(msg)

    def signal_takeoff(self):
        self.is_flying = True
        msg = Bool()
        msg.data = False
        self.land_pub.publish(msg)

def smooth_move(node, tx, ty, tz, speed=0.04):
    global drone_x, drone_y, drone_z
    with pos_lock:
        cx, cy, cz = drone_x, drone_y, drone_z
    dist = math.sqrt((tx-cx)**2 + (ty-cy)**2 + (tz-cz)**2)
    if dist < 0.001:
        return
    steps = max(int(dist / speed), 1)
    for i in range(1, steps + 1):
        t      = i / steps
        t_ease = t * t * (3 - 2 * t)
        x = round(cx + (tx - cx) * t_ease, 4)
        y = round(cy + (ty - cy) * t_ease, 4)
        z = round(cz + (tz - cz) * t_ease, 4)
        move_drone_fast(x, y, z)
        node.set_pos(x, y, z)
        time.sleep(STEP_DELAY)

def takeoff(node):
    print("Drone1: Taking off...")
    node.is_charging = False
    node.is_flying   = True
    node.publish_status('flying')
    node.signal_takeoff()
    smooth_move(node, DOCK_X, DOCK_Y, FLY_Z, speed=0.03)
    print("Drone1: Airborne!")

def land_on_dock(node):
    print("Drone1: Returning to dock — signaling Drone2 to take off...")
    node.signal_landing()
    node.publish_status('returning')
    smooth_move(node, DOCK_X, DOCK_Y, FLY_Z, speed=0.05)
    smooth_move(node, DOCK_X, DOCK_Y, DOCK_Z, speed=0.03)
    node.is_flying   = False
    node.is_charging = True
    node.publish_status('charging')
    print("Drone1: Landed and charging!")

def patrol(node):
    print("Drone1: Waiting for Gazebo...")
    time.sleep(5.0)

    node.battery_pct = 1.0
    node.is_charging = False

    takeoff(node)

    while rclpy.ok():
        node.patrol_count += 1
        print(f"\n{'='*40}")
        print(f"Drone1: Patrol round {node.patrol_count}")
        print(f"{'='*40}")

        for i, (tx, ty) in enumerate(wall_waypoints):
            if node.needs_recharge():
                print(f"Drone1: Battery low at waypoint {i+1}!")
                break
            print(f"  Waypoint {i+1}/{len(wall_waypoints)}: "
                  f"({tx:.1f},{ty:.1f}) "
                  f"Batt:{node.battery_pct*100:.0f}%")
            smooth_move(node, tx, ty, FLY_Z, speed=0.04)

        if node.needs_recharge():
            # Land and signal drone2
            land_on_dock(node)

            # Charge until full
            print("Drone1: Charging — waiting for 100%...")
            while not node.is_fully_charged() and rclpy.ok():
                pct = node.battery_pct * 100
                print(f"  Drone1 charging: {pct:.0f}%")
                time.sleep(2.0)

            print("Drone1: Fully charged!")
            node.is_charging = False

            # Wait for drone2 to return to dock before taking off
            print("Drone1: Waiting for Drone2 to return to dock...")
            while not node.drone2_on_dock and rclpy.ok():
                print("  Drone1: Drone2 still flying — waiting...")
                time.sleep(2.0)

            print("Drone1: Drone2 on dock — taking off!")
            takeoff(node)
        else:
            print(f"\nDrone1: Patrol complete — "
                  f"battery {node.battery_pct*100:.0f}% continuing!")

def main():
    rclpy.init()
    node = Drone1PatrolNode()

    bridge = subprocess.Popen([
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    try:
        patrol(node)
    except KeyboardInterrupt:
        print("\nDrone1: Stopping...")
    finally:
        bridge.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()