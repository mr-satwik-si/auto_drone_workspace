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

DOCK2_X = 4.0
DOCK2_Y = 4.0
DOCK2_Z = 0.05
FLY_Z   = 1.2
STEP_DELAY = 0.1

RECHARGE_THRESHOLD = 0.30
FULL_CHARGE        = 1.00

inner_waypoints = [
    (2.0, 2.0),
    (1.0, 1.0),
    (1.0, 2.0),
    (1.0, 3.0),
    (2.0, 3.0),
    (3.0, 3.0),
    (3.0, 2.0),
    (3.0, 1.0),
    (2.0, 1.0),
    (2.0, 2.0),
]

pos_lock2 = threading.Lock()
drone2_x  = DOCK2_X
drone2_y  = DOCK2_Y
drone2_z  = DOCK2_Z

def move_drone2_fast(x, y, z):
    req = f'name: "drone2", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}'
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

class Drone2PatrolNode(Node):
    def __init__(self):
        super().__init__('drone2_patrol_node')

        self.pose_pub    = self.create_publisher(PoseStamped, '/drone2/pose', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/drone2/battery', 10)
        self.status_pub  = self.create_publisher(String, '/drone2/status', 10)
        self.cam_switch  = self.create_publisher(String, '/active_drone_cam', 10)

        self.battery_pct   = 1.0
        self.is_charging   = True
        self.is_flying     = False
        self.patrol_count  = 0
        self.drone1_status = 'flying'
        self.drone1_on_dock = False

        # Listen to drone1 signals
        self.create_subscription(Bool,   '/drone/land',    self.drone1_land_cb, 10)
        self.create_subscription(String, '/drone1/status', self.drone1_status_cb, 10)

        self.create_timer(0.5,  self.battery_tick)
        self.create_timer(0.05, self.publish_pose)

        self.get_logger().info('Drone2PatrolNode ready — standby mode')

    def drone1_land_cb(self, msg):
        if msg.data:
            self.drone1_status = 'returning'
        else:
            self.drone1_status = 'flying'

    def drone1_status_cb(self, msg):
        self.drone1_status = msg.data
        self.drone1_on_dock = msg.data in ['charging', 'on_dock']

    def battery_tick(self):
        if self.is_charging:
            # Charge only when on dock
            self.battery_pct = min(1.0, self.battery_pct + 0.015)
        elif self.is_flying:
            # Drain only when flying
            self.battery_pct = max(0.0, self.battery_pct - 0.002)
        # On dock not charging — battery stays same

        msg = BatteryState()
        msg.percentage = self.battery_pct
        self.battery_pub.publish(msg)

    def publish_pose(self):
        global drone2_x, drone2_y, drone2_z
        with pos_lock2:
            x, y, z = drone2_x, drone2_y, drone2_z
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    def set_pos(self, x, y, z):
        global drone2_x, drone2_y, drone2_z
        with pos_lock2:
            drone2_x, drone2_y, drone2_z = x, y, z

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def switch_camera(self, drone):
        msg = String()
        msg.data = drone
        self.cam_switch.publish(msg)

    def needs_recharge(self):
        return self.battery_pct < RECHARGE_THRESHOLD and not self.is_charging

    def is_fully_charged(self):
        return self.battery_pct >= FULL_CHARGE

def smooth_move2(node, tx, ty, tz, speed=0.04):
    global drone2_x, drone2_y, drone2_z
    with pos_lock2:
        cx, cy, cz = drone2_x, drone2_y, drone2_z
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
        move_drone2_fast(x, y, z)
        node.set_pos(x, y, z)
        time.sleep(STEP_DELAY)

def takeoff2(node):
    print("Drone2: Taking off!")
    node.is_charging = False
    node.is_flying   = True
    node.publish_status('flying')
    node.switch_camera('drone2')
    smooth_move2(node, DOCK2_X, DOCK2_Y, FLY_Z, speed=0.03)
    print("Drone2: Airborne — camera switched to Drone2!")

def land_on_dock2(node):
    print("Drone2: Returning to dock...")
    node.publish_status('returning')
    smooth_move2(node, DOCK2_X, DOCK2_Y, FLY_Z, speed=0.05)
    smooth_move2(node, DOCK2_X, DOCK2_Y, DOCK2_Z, speed=0.03)
    node.is_flying   = False
    node.is_charging = True
    node.publish_status('on_dock')
    node.switch_camera('drone1')
    print("Drone2: Landed — camera switched back to Drone1!")
    
def patrol2(node):
    print("Drone2: Waiting for Gazebo...")
    time.sleep(8.0)

    node.publish_status('on_dock')
    print("Drone2: Standby — waiting for Drone1 to start returning...")

    while rclpy.ok():
        # Take off when drone1 signals it is returning to dock
        if node.drone1_status == 'returning' and not node.needs_recharge():
            if node.is_charging:
                takeoff2(node)

            node.patrol_count += 1
            print(f"\nDrone2: Patrol round {node.patrol_count} — inner area")

            for i, (tx, ty) in enumerate(inner_waypoints):
                # Check if drone1 has taken off again and drone2 needs to return
                if node.needs_recharge():
                    print("Drone2: Battery low — returning to dock!")
                    break

                print(f"  Drone2 waypoint {i+1}: ({tx:.1f},{ty:.1f}) "
                      f"Batt:{node.battery_pct*100:.0f}%")
                smooth_move2(node, tx, ty, FLY_Z, speed=0.04)

            # Return to dock after patrol or battery low
            land_on_dock2(node)

            # Wait until fully charged
            print("Drone2: Charging...")
            while not node.is_fully_charged() and rclpy.ok():
                pct = node.battery_pct * 100
                print(f"  Drone2 charging: {pct:.0f}%")
                time.sleep(2.0)

            node.publish_status('on_dock')
            print("Drone2: Fully charged — standby!")

        else:
            # Stay on dock
            if not node.is_charging:
                land_on_dock2(node)
            time.sleep(0.5)

def main():
    rclpy.init()
    node = Drone2PatrolNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    try:
        patrol2(node)
    except KeyboardInterrupt:
        print("\nDrone2: Stopping...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()