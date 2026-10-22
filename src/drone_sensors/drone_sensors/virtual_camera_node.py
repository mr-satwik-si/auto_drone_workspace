#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import math

class VirtualCameraNode(Node):
    def __init__(self):
        super().__init__('virtual_camera_node')

        self.camera_pub = self.create_publisher(
            Image, '/camera/image_raw', 10)

        self.create_subscription(
            PoseStamped, '/drone/pose', self.pose_cb, 10)

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 1.2
        self.heading  = 0.0  # degrees

        self.W = 320
        self.H = 240

        # Room dimensions
        self.ROOM_W = 4.0
        self.ROOM_H = 4.0
        self.ROOM_Z = 2.0

        # Objects: cx, cy, cz, w, d, h, color_top, color_side, label
        self.objects = [
            {'name': 'floor',
             'type': 'plane',
             'color': (60, 55, 50)},
            {'name': 'ceiling',
             'type': 'ceiling',
             'color': (30, 30, 40)},
            {'name': 'wall_north',
             'type': 'wall', 'side': 'north',
             'color': (70, 70, 80)},
            {'name': 'wall_south',
             'type': 'wall', 'side': 'south',
             'color': (70, 70, 80)},
            {'name': 'wall_east',
             'type': 'wall', 'side': 'east',
             'color': (65, 65, 75)},
            {'name': 'wall_west',
             'type': 'wall', 'side': 'west',
             'color': (65, 65, 75)},
            {'name': 'table',
             'cx': 2.0, 'cy': 2.0, 'cz': 0.4,
             'w': 1.0,  'd': 0.6,  'h': 0.8,
             'color_top':  (40,  80, 120),
             'color_side': (30,  60,  90),
             'label': 'Table'},
            {'name': 'chair',
             'cx': 2.0, 'cy': 1.2, 'cz': 0.45,
             'w': 0.5,  'd': 0.5,  'h': 0.9,
             'color_top':  (60, 100, 150),
             'color_side': (45,  75, 110),
             'label': 'Chair'},
            {'name': 'sofa',
             'cx': 0.8, 'cy': 3.6, 'cz': 0.4,
             'w': 1.8,  'd': 0.7,  'h': 0.8,
             'color_top':  (40,  40, 160),
             'color_side': (30,  30, 120),
             'label': 'Sofa'},
            {'name': 'bookshelf',
             'cx': 3.7, 'cy': 3.8, 'cz': 0.9,
             'w': 0.3,  'd': 1.0,  'h': 1.8,
             'color_top':  (30,  70, 100),
             'color_side': (20,  50,  70),
             'label': 'Shelf'},
            {'name': 'tv_stand',
             'cx': 2.0, 'cy': 0.2, 'cz': 0.25,
             'w': 1.2,  'd': 0.4,  'h': 0.5,
             'color_top':  (25,  25,  25),
             'color_side': (15,  15,  15),
             'label': 'TV'},
            {'name': 'plant',
             'cx': 3.8, 'cy': 0.3, 'cz': 0.2,
             'w': 0.3,  'd': 0.3,  'h': 0.4,
             'color_top':  (20, 140,  20),
             'color_side': (10, 100,  10),
             'label': 'Plant'},
            {'name': 'box1',
             'cx': 1.5, 'cy': 1.5, 'cz': 0.15,
             'w': 0.3,  'd': 0.3,  'h': 0.3,
             'color_top':  (0,  200, 220),
             'color_side': (0,  150, 170),
             'label': 'Box'},
            {'name': 'box2',
             'cx': 3.0, 'cy': 1.0, 'cz': 0.2,
             'w': 0.4,  'd': 0.4,  'h': 0.4,
             'color_top':  (180, 100,   0),
             'color_side': (130,  70,   0),
             'label': 'Box'},
            {'name': 'fire_object',
             'cx': 2.0, 'cy': 2.0, 'cz': 0.25,
             'w': 0.4,  'd': 0.4,  'h': 0.5,
             'color_top':  (0,   80, 255),
             'color_side': (0,   50, 200),
             'label': 'FIRE',
             'is_fire': True},
            {'name': 'dock',
             'cx': 0.0, 'cy': 0.0, 'cz': 0.025,
             'w': 0.4,  'd': 0.4,  'h': 0.05,
             'color_top':  (0,  180,   0),
             'color_side': (0,  120,   0),
             'label': 'Dock'},
            {'name': 'person',
             'cx': 2.0, 'cy': 2.5, 'cz': 0.75,
             'w': 0.4,  'd': 0.3,  'h': 1.5,
             'color_top':  (180, 100,  50),
             'color_side': (50, 100, 180),
             'label': 'PERSON',
             'is_person': True},
        ]

        self.prev_x = 0.0
        self.prev_y = 0.0

        self.create_timer(0.033, self.render_frame)
        self.get_logger().info('VirtualCameraNode — first person view at 30fps')

    def pose_cb(self, msg):
        nx = msg.pose.position.x
        ny = msg.pose.position.y

        dx = nx - self.prev_x
        dy = ny - self.prev_y
        if abs(dx) > 0.001 or abs(dy) > 0.001:
            self.heading = math.degrees(math.atan2(dy, dx))

        self.drone_x = nx
        self.drone_y = ny
        self.drone_z = msg.pose.position.z
        self.prev_x  = nx
        self.prev_y  = ny

    def project(self, wx, wy, wz):
        """Project world point to screen using perspective."""
        # Translate to drone space
        dx = wx - self.drone_x
        dy = wy - self.drone_y
        dz = wz - self.drone_z

        # Rotate by heading
        h_rad = math.radians(self.heading)
        cos_h = math.cos(h_rad)
        sin_h = math.sin(h_rad)

        # Camera looks in heading direction
        forward =  dx * cos_h + dy * sin_h
        right   = -dx * sin_h + dy * cos_h
        up      = dz

        # Camera tilted down 30 degrees
        tilt    = math.radians(-20)
        cos_t   = math.cos(tilt)
        sin_t   = math.sin(tilt)
        new_up  = up * cos_t - forward * sin_t
        new_fwd = up * sin_t + forward * cos_t

        if new_fwd < 0.1:
            return None, None, None

        # Perspective projection
        fov = math.radians(70)
        f   = 1.0 / math.tan(fov / 2)

        sx = int(self.W/2 + (right   / new_fwd) * f * self.W/2)
        sy = int(self.H/2 - (new_up  / new_fwd) * f * self.H/2)

        return sx, sy, new_fwd

    def draw_box(self, img, obj):
        """Draw a 3D box in first-person perspective."""
        cx = obj['cx']
        cy = obj['cy']
        cz = obj['cz']
        w  = obj['w'] / 2
        d  = obj['d'] / 2
        h  = obj['h'] / 2

        # 8 corners of the box
        corners_world = [
            (cx-w, cy-d, cz-h),
            (cx+w, cy-d, cz-h),
            (cx+w, cy+d, cz-h),
            (cx-w, cy+d, cz-h),
            (cx-w, cy-d, cz+h),
            (cx+w, cy-d, cz+h),
            (cx+w, cy+d, cz+h),
            (cx-w, cy+d, cz+h),
        ]

        # Project all corners
        projected = []
        for (wx, wy, wz) in corners_world:
            sx, sy, depth = self.project(wx, wy, wz)
            projected.append((sx, sy, depth))

        # Check if any corner is visible
        valid = [(p[0], p[1], p[2]) for p in projected
                 if p[2] is not None and p[2] > 0.1]
        if not valid:
            return

        avg_depth = sum(p[2] for p in valid) / len(valid)

        # Draw faces
        faces = [
            ([4, 5, 6, 7], obj['color_top']),    # top
            ([0, 1, 5, 4], obj['color_side']),    # front
            ([1, 2, 6, 5], obj['color_side']),    # right
            ([2, 3, 7, 6], obj['color_side']),    # back
            ([3, 0, 4, 7], obj['color_side']),    # left
        ]

        for (indices, color) in faces:
            pts = []
            all_valid = True
            for idx in indices:
                sx, sy, depth = projected[idx]
                if depth is None or depth < 0.1:
                    all_valid = False
                    break
                pts.append([sx, sy])

            if not all_valid or len(pts) < 3:
                continue

            pts_arr = np.array(pts, dtype=np.int32)

            # Depth shading
            shade = max(0.3, min(1.0, 2.0 / max(avg_depth, 0.5)))
            shaded = tuple(int(c * shade) for c in color)

            cv2.fillPoly(img, [pts_arr], shaded)
            cv2.polylines(img, [pts_arr], True,
                         tuple(min(255, c+40) for c in shaded), 1)

        # Label
        cx_s, cy_s, _ = projected[6]
        if cx_s and 0 < cx_s < self.W and 0 < cy_s < self.H:
            label = obj.get('label', '')
            cv2.putText(img, label, (cx_s-20, cy_s-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                       (255, 255, 255), 1)
            if obj.get('is_fire'):
                cv2.putText(img, '🔥', (cx_s-10, cy_s-20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                           (0, 80, 255), 2)

        return avg_depth

    def render_frame(self):
        W, H = self.W, self.H
        img  = np.zeros((H, W, 3), dtype=np.uint8)

        # Sky / ceiling gradient
        for y in range(H//2):
            t = y / (H//2)
            b = int(30 + t * 20)
            img[y, :] = (b, b+5, b+10)

        # Floor gradient
        for y in range(H//2, H):
            t = (y - H//2) / (H//2)
            b = int(50 + t * 20)
            img[y, :] = (b-5, b-8, b-10)

        # Draw horizon line
        cv2.line(img, (0, H//2), (W, H//2), (80, 80, 90), 1)

        # Floor grid for depth perception
        for i in range(1, 10):
            d    = i * 0.5
            left_w  = (-2.0, self.drone_y + d, 0)
            right_w = ( 2.0, self.drone_y + d, 0)
            lx, ly, ld = self.project(
                self.drone_x - 2, self.drone_y + d, 0)
            rx, ry, rd = self.project(
                self.drone_x + 2, self.drone_y + d, 0)
            if ld and ld > 0.1 and rd and rd > 0.1:
                alpha = max(40, int(120 / (d + 0.5)))
                cv2.line(img, (max(0, lx), ly),
                        (min(W, rx), ry),
                        (alpha, alpha, alpha), 1)

        # Sort objects by distance — draw far first
        def obj_dist(obj):
            if 'cx' not in obj:
                return 0
            return math.sqrt(
                (obj['cx'] - self.drone_x)**2 +
                (obj['cy'] - self.drone_y)**2
            )

        box_objects = [o for o in self.objects if 'cx' in o]
        sorted_objs = sorted(box_objects, key=obj_dist, reverse=True)

        for obj in sorted_objs:
            self.draw_box(img, obj)

        # Draw person specially with head
        for obj in sorted_objs:
            if obj.get('is_person'):
                hx, hy, hd = self.project(
                    obj['cx'], obj['cy'],
                    obj['cz'] + obj['h']/2 + 0.12)
                if hd and hd > 0.1:
                    r = max(3, int(12 / max(hd, 0.5)))
                    cv2.circle(img, (hx, hy), r, (150, 180, 220), -1)
                    cv2.circle(img, (hx, hy), r, (200, 220, 255), 1)

        # HUD overlay
        cv2.rectangle(img, (0, 0), (W, 18), (0, 0, 0), -1)
        cv2.putText(
            img,
            f'Drone ({self.drone_x:.1f},{self.drone_y:.1f},'
            f'{self.drone_z:.1f}m) Heading:{self.heading:.0f}deg',
            (4, 13),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 220, 0), 1
        )

        # Crosshair
        cv2.line(img, (W//2-10, H//2), (W//2+10, H//2),
                (0, 255, 0), 1)
        cv2.line(img, (W//2, H//2-10), (W//2, H//2+10),
                (0, 255, 0), 1)

        # Publish
        msg             = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height      = H
        msg.width       = W
        msg.encoding    = 'bgr8'
        msg.step        = W * 3
        msg.data        = img.tobytes()
        self.camera_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()