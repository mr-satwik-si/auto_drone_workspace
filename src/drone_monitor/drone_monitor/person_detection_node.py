#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
import time

class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__('person_detection_node')

        self.declare_parameter('alert_cooldown', 15.0)
        self.cooldown = self.get_parameter('alert_cooldown').value
        self.last_alert_time = 0.0

        self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.alert_pub  = self.create_publisher(String, '/alerts', 10)
        self.person_pub = self.create_publisher(Image, '/person/image', 10)

        self.get_logger().info('PersonDetectionNode started')

    def image_callback(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Detect blue torso — narrow range to match person model exactly
        # Person torso is RGB(0.2, 0.4, 0.8) = medium blue
        lower_blue = np.array([105, 120, 80])
        upper_blue = np.array([125, 255, 255])
        blue_mask  = cv2.inRange(hsv, lower_blue, upper_blue)

        # Detect skin color — head and arms
        # Person head is RGB(0.9, 0.7, 0.5) = light skin tone
        lower_skin = np.array([10, 40, 180])
        upper_skin = np.array([25, 100, 255])
        skin_mask  = cv2.inRange(hsv, lower_skin, upper_skin)

        # Clean up masks
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        skin_mask = cv2.morphologyEx(skin_mask, cv2.MORPH_CLOSE, kernel)

        # Find blue contours (torso candidates)
        blue_contours, _ = cv2.findContours(
            blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find skin contours (head candidates)
        skin_contours, _ = cv2.findContours(
            skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        person_count = 0
        person_detected_boxes = []

        # Key rule: only detect as person if BOTH blue torso AND skin
        # color are present in the same vertical region
        for b_cnt in blue_contours:
            b_area = cv2.contourArea(b_cnt)
            if b_area < 400:
                continue

            bx, by, bw, bh = cv2.boundingRect(b_cnt)

            # Check aspect ratio — torso should be taller than wide
            if bh < bw * 0.5:
                continue

            # Check if skin color exists ABOVE the blue region
            # Head should be above torso
            skin_found = False
            for s_cnt in skin_contours:
                s_area = cv2.contourArea(s_cnt)
                if s_area < 100:
                    continue

                sx, sy, sw, sh = cv2.boundingRect(s_cnt)

                # Skin must be above blue torso
                skin_center_y = sy + sh // 2
                blue_top_y    = by

                # Skin must be horizontally aligned with torso
                skin_center_x = sx + sw // 2
                blue_center_x = bx + bw // 2

                horizontal_aligned = abs(skin_center_x - blue_center_x) < bw
                above_torso        = skin_center_y < blue_top_y + 20
                close_enough       = skin_center_y > blue_top_y - bh * 1.5

                if horizontal_aligned and above_torso and close_enough:
                    skin_found = True
                    break

            if skin_found:
                person_count += 1
                # Draw bounding box around full person
                px = bx - 10
                py = by - int(bh * 0.8)
                pw = bw + 20
                ph = bh + int(bh * 0.8)

                px = max(0, px)
                py = max(0, py)

                person_detected_boxes.append((px, py, pw, ph))

                cv2.rectangle(img, (px, py), (px+pw, py+ph), (0, 255, 0), 2)
                cv2.putText(
                    img, 'PERSON DETECTED',
                    (px, py - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2
                )

        # Status overlay
        label = f'Persons: {person_count}'
        color = (0, 220, 0) if person_count > 0 else (120, 120, 120)
        cv2.rectangle(img, (5, 5), (210, 38), (0, 0, 0), -1)
        cv2.putText(img, label, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # Publish annotated image
        out          = Image()
        out.header   = msg.header
        out.height   = msg.height
        out.width    = msg.width
        out.encoding = 'bgr8'
        out.step     = msg.width * 3
        out.data     = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).tobytes()
        self.person_pub.publish(out)

        # Send alert only if person confirmed
        if person_count > 0:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_alert_time > self.cooldown:
                self.last_alert_time = now
                alert = String()
                alert.data = f'ALERT:PERSON:count={person_count}'
                self.alert_pub.publish(alert)
                self.get_logger().warn(
                    f'PERSON DETECTED! Count: {person_count}')

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()