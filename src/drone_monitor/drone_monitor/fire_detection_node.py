#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
import time

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')

        self.declare_parameter('alert_cooldown', 10.0)
        self.cooldown = self.get_parameter('alert_cooldown').value
        self.last_alert_time = 0.0

        # Subscribe to both drone cameras
        self.create_subscription(
            Image, '/camera/image_raw', self.camera1_cb, 1)
        self.create_subscription(
            Image, '/camera2/image_raw', self.camera2_cb, 1)

        self.alert_pub   = self.create_publisher(String, '/alerts', 10)
        self.fire1_pub   = self.create_publisher(Image, '/fire/image', 10)
        self.fire2_pub   = self.create_publisher(Image, '/fire2/image', 10)

        self.get_logger().info('FireDetectionNode started — monitoring both drones')

    def convert_image(self, msg):
        """Convert ROS Image to BGR numpy array."""
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            pass
        elif msg.encoding == 'rgba8':
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        return img

    def detect_fire(self, img):
        """Run fire detection on image. Returns annotated image and fire_pct."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Red lower
        lower1 = np.array([0,   100, 100])
        upper1 = np.array([15,  255, 255])
        mask1  = cv2.inRange(hsv, lower1, upper1)

        # Red upper wraparound
        lower2 = np.array([165, 100, 100])
        upper2 = np.array([180, 255, 255])
        mask2  = cv2.inRange(hsv, lower2, upper2)

        # Orange
        lower3 = np.array([15,  100, 100])
        upper3 = np.array([35,  255, 255])
        mask3  = cv2.inRange(hsv, lower3, upper3)

        fire_mask = cv2.bitwise_or(mask1, mask2)
        fire_mask = cv2.bitwise_or(fire_mask, mask3)

        total_pixels = img.shape[0] * img.shape[1]
        fire_pixels  = cv2.countNonZero(fire_mask)
        fire_pct     = (fire_pixels / total_pixels) * 100.0

        contours, _ = cv2.findContours(
            fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        annotated = img.copy()
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(annotated, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(annotated, 'FIRE',
                           (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX,
                           0.6, (0, 0, 255), 2)

        # Status overlay
        label = f'Fire: {fire_pct:.1f}%'
        color = (0, 0, 255) if fire_pct > 1.0 else (0, 200, 0)
        cv2.rectangle(annotated, (0, 0), (160, 20), (0, 0, 0), -1)
        cv2.putText(annotated, label, (4, 14),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        return annotated, fire_pct

    def publish_image(self, pub, img, msg):
        """Publish annotated image."""
        out          = Image()
        out.header   = msg.header
        out.height   = img.shape[0]
        out.width    = img.shape[1]
        out.encoding = 'bgr8'
        out.step     = img.shape[1] * 3
        out.data     = img.tobytes()
        pub.publish(out)

    def trigger_alert(self, drone_id, fire_pct):
        """Send fire alert with cooldown."""
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_alert_time > self.cooldown:
            self.last_alert_time = now
            alert = String()
            alert.data = (f'ALERT:FIRE:drone={drone_id}:'
                         f'pixels={fire_pct:.1f}%')
            self.alert_pub.publish(alert)
            self.get_logger().warn(
                f'FIRE DETECTED by Drone{drone_id}! {fire_pct:.1f}%')

    def camera1_cb(self, msg):
        try:
            img = self.convert_image(msg)
            annotated, fire_pct = self.detect_fire(img)
            self.publish_image(self.fire1_pub, annotated, msg)
            if fire_pct > 1.0:
                self.trigger_alert(1, fire_pct)
        except Exception as e:
            self.get_logger().error(f'Camera1 fire detection error: {e}')

    def camera2_cb(self, msg):
        try:
            img = self.convert_image(msg)
            annotated, fire_pct = self.detect_fire(img)
            self.publish_image(self.fire2_pub, annotated, msg)
            if fire_pct > 1.0:
                self.trigger_alert(2, fire_pct)
        except Exception as e:
            self.get_logger().error(f'Camera2 fire detection error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FireDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()