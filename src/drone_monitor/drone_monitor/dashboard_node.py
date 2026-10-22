#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
from sensor_msgs.msg import BatteryState, Image
from geometry_msgs.msg import PoseStamped
import threading
import time
import base64
import numpy as np
import cv2
from flask import Flask, render_template, Response
from flask_socketio import SocketIO

TEMPLATE_DIR = '/home/stack/drone_monitor_ws/src/drone_monitor/drone_monitor/dashboard/templates'
STATIC_DIR   = '/home/stack/drone_monitor_ws/src/drone_monitor/drone_monitor/dashboard/static'

app = Flask(__name__,
    template_folder=TEMPLATE_DIR,
    static_folder=STATIC_DIR
)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')


state = {
    'temperature': 0.0,
    'sound_level': 0.0,
    'battery': 100.0,
    'battery2': 100.0,
    'alerts': [],
    'drone_x': 0.0,
    'drone_y': 0.0,
    'drone_z': 0.0,
    'drone2_x': 4.0,
    'drone2_y': 4.0,
    'drone2_z': 0.0,
    'fire_detected': False,
    'status': 'Patrolling',
    'status2': 'Standby',
}

# Latest camera frame (JPEG bytes)
latest_frame = None
latest_fire_frame = None
latest_fire2_frame = None
latest_frame2 = None

frame_lock = threading.Lock()

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.create_subscription(Float32, '/temperature', self.temp_cb, 10)
        self.create_subscription(Float32, '/sound_level', self.sound_cb, 10)
        self.create_subscription(String, '/alerts', self.alert_cb, 10)
        self.create_subscription(BatteryState, '/battery', self.battery_cb, 10)
        self.create_subscription(PoseStamped, '/drone/pose', self.pose_cb, 10)
        self.create_subscription(PoseStamped, '/drone2/pose', self.pose2_cb, 10)
        self.create_subscription(BatteryState, '/drone2/battery', self.battery2_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_cb, 1)
        self.create_subscription(Image, '/camera2/image_raw', self.camera2_cb, 1)
        self.create_subscription(String, '/active_drone_cam', self.cam_switch_cb, 10)
        self.active_cam = 'drone1'
        self.create_subscription(Image, '/fire/image', self.fire_camera_cb, 1)
        self.create_subscription(Image, '/fire2/image', self.fire2_camera_cb, 1)
        
        self.get_logger().info('DashboardNode started at http://localhost:5000')

    def temp_cb(self, msg):
        state['temperature'] = round(msg.data, 1)
        socketio.emit('update', state)

    def sound_cb(self, msg):
        state['sound_level'] = round(msg.data, 1)
        socketio.emit('update', state)

    def alert_cb(self, msg):
        entry = {'time': time.strftime('%H:%M:%S'), 'message': msg.data}
        state['alerts'].insert(0, entry)
        state['alerts'] = state['alerts'][:10]
        if 'FIRE' in msg.data:
            state['fire_detected'] = True
        socketio.emit('update', state)
        socketio.emit('new_alert', entry)

    def battery_cb(self, msg):
        new_batt = round(msg.percentage * 100.0, 1)
        old_batt = state['battery']
        state['battery'] = new_batt

        # Detect charging by battery going up
        if new_batt > old_batt:
            state['is_charging'] = True
        elif new_batt < old_batt:
            state['is_charging'] = False

        if state.get('is_charging', False):
            state['status'] = f'Charging {new_batt:.0f}%'
        elif new_batt < 30.0:
            state['status'] = 'Returning to dock'
        elif new_batt >= 100.0:
            state['status'] = 'Patrolling'
        else:
            state['status'] = 'Patrolling'

        socketio.emit('update', state)
    def pose_cb(self, msg):
        state['drone_x'] = round(msg.pose.position.x, 2)
        state['drone_y'] = round(msg.pose.position.y, 2)
        state['drone_z'] = round(msg.pose.position.z, 2)
        socketio.emit('update', state)

    def pose2_cb(self, msg):
        state['drone2_x'] = round(msg.pose.position.x, 2)
        state['drone2_y'] = round(msg.pose.position.y, 2)
        state['drone2_z'] = round(msg.pose.position.z, 2)
        socketio.emit('update', state)

    def battery2_cb(self, msg):
        state['battery2'] = round(msg.percentage * 100.0, 1)
        if state['battery2'] < 30.0:
            state['status2'] = 'Returning to dock'
        elif state.get('is_charging2', False):
            state['status2'] = f'Charging {state["battery2"]:.0f}%'
        else:
            state['status2'] = 'Patrolling'
        socketio.emit('update', state)

    def ros_image_to_jpeg(self, msg):
        """Convert ROS Image to JPEG bytes — handles rgb8 and bgr8."""
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1
            )
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                pass  # already BGR — no conversion needed
            elif msg.encoding == 'rgba8':
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            elif msg.encoding == 'mono8':
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            _, jpeg = cv2.imencode(
                '.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            return jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return None
        
    def camera_cb(self, msg):
        global latest_frame
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            # Apply stabilization — blend with previous frame
            if not hasattr(self, 'prev_frame') or self.prev_frame is None:
                self.prev_frame = img.copy()

            # Weighted blend: 70% current + 30% previous for smoothness
            stabilized = cv2.addWeighted(img, 0.7, self.prev_frame, 0.3, 0)
            self.prev_frame = stabilized.copy()

            # Apply slight gaussian blur to reduce jitter artifacts
            stabilized = cv2.GaussianBlur(stabilized, (3, 3), 0)

            _, jpeg = cv2.imencode(
                '.jpg', stabilized, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if jpeg is not None:
                with frame_lock:
                    latest_frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')

    def camera2_cb(self, msg):
        global latest_frame2
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            if not hasattr(self, 'prev_frame2') or self.prev_frame2 is None:
                self.prev_frame2 = img.copy()

            stabilized = cv2.addWeighted(img, 0.7, self.prev_frame2, 0.3, 0)
            self.prev_frame2 = stabilized.copy()
            stabilized = cv2.GaussianBlur(stabilized, (3, 3), 0)

            _, jpeg = cv2.imencode(
                '.jpg', stabilized, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if jpeg is not None:
                with frame_lock:
                    latest_frame2 = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f'Camera2 error: {e}')

    def cam_switch_cb(self, msg):
        self.active_cam = msg.data
        self.get_logger().info(f'Camera switched to {msg.data}')
        socketio.emit('cam_switch', {'active': msg.data})

    def fire_camera_cb(self, msg):
        global latest_fire_frame
        try:
            jpeg = self.ros_image_to_jpeg(msg)
            if jpeg:
                with frame_lock:
                    latest_fire_frame = jpeg
        except Exception as e:
            self.get_logger().error(f'Fire camera error: {e}')

    def fire2_camera_cb(self, msg):
        global latest_fire2_frame
        try:
            jpeg = self.ros_image_to_jpeg(msg)
            if jpeg:
                with frame_lock:
                    latest_fire2_frame = jpeg
        except Exception as e:
            self.get_logger().error(f'Fire2 camera error: {e}')

            
def generate_camera_feed():
    last_frame = None
    while True:
        with frame_lock:
            frame = latest_frame
        if frame and frame is not last_frame:
            last_frame = frame
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.01)

def generate_camera2_feed():
    last_frame = None
    while True:
        with frame_lock:
            frame = latest_frame2
        if frame and frame is not last_frame:
            last_frame = frame
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.01)

def generate_fire_feed():
    last_frame = None
    while True:
        with frame_lock:
            frame = latest_fire_frame
        if frame and frame is not last_frame:
            last_frame = frame
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.01)


def generate_fire2_feed():
    last_frame = None
    while True:
        with frame_lock:
            frame = latest_fire2_frame
        if frame and frame is not last_frame:
            last_frame = frame
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.01)


@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/camera_feed')
def camera_feed():
    return Response(
        generate_camera_feed(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/camera2_feed')
def camera2_feed():
    return Response(
        generate_camera2_feed(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/fire_feed')
def fire_feed():
    return Response(
        generate_fire_feed(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/fire2_feed')
def fire2_feed():
    return Response(
        generate_fire2_feed(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )



def run_ros(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    ros_thread = threading.Thread(target=run_ros, args=(node,), daemon=True)
    ros_thread.start()

    socketio.run(app, host='0.0.0.0', port=5000, debug=False)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
