#!/usr/bin/env python3
"""
Lane Bridge Node
Runs on ONE laptop
Receives JSON from Pi via TCP
Publishes all lane topics to ROS2 network
All other laptops see topics automatically
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import socket
import struct
import threading
import json

PI_IP   = '192.168.1.5'
PI_PORT = 9999


class LaneBridgeNode(Node):

    def __init__(self):
        super().__init__('lane_bridge_node')

        # ── Publishers ────────────────────────────────────
        self.pub_mask = self.create_publisher(
            Image,   '/lane_mask',       10)
        self.pub_center = self.create_publisher(
            Float32, '/lane_center',     10)
        self.pub_error = self.create_publisher(
            Float32, '/lane_error',      10)
        self.pub_lane = self.create_publisher(
            String,  '/current_lane',    10)
        self.pub_width = self.create_publisher(
            Float32, '/lane_width',      10)
        self.pub_curv = self.create_publisher(
            Float32, '/lane_curvature',  10)
        self.pub_conf = self.create_publisher(
            Float32, '/lane_confidence', 10)
        self.pub_error_cm = self.create_publisher(
            Float32, '/lane_error_cm', 10)
        self.pub_width_cm = self.create_publisher(
            Float32, '/lane_width_cm', 10)

        self.bridge = CvBridge()

        # ── Connect to Pi ─────────────────────────────────
        self.get_logger().info(
            f'Connecting to Pi '
            f'{PI_IP}:{PI_PORT}...')
        self.sock = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((PI_IP, PI_PORT))
        self.get_logger().info(
            'Connected to Pi successfully')

        # ── Start receive thread ───────────────────────────
        self.running = True
        self.thread  = threading.Thread(
            target=self.receive_loop,
            daemon=True)
        self.thread.start()

        self.get_logger().info(
            'Lane bridge publishing on:')
        self.get_logger().info(
            '  /lane_mask')
        self.get_logger().info(
            '  /lane_center')
        self.get_logger().info(
            '  /lane_error')
        self.get_logger().info(
            '  /current_lane')
        self.get_logger().info(
            '  /lane_width')
        self.get_logger().info(
            '  /lane_curvature')
        self.get_logger().info(
            '  /lane_confidence')

    # ── Receive loop ──────────────────────────────────────
    def receive_loop(self):
        data         = b''
        payload_size = struct.calcsize('>L')

        while self.running:
            try:
                # Get size header
                while len(data) < payload_size:
                    packet = self.sock.recv(4096)
                    if not packet:
                        self.get_logger().warn(
                            'Pi disconnected')
                        return
                    data += packet

                packed = data[:payload_size]
                data   = data[payload_size:]
                size   = struct.unpack(
                    '>L', packed)[0]

                # Get payload
                while len(data) < size:
                    packet = self.sock.recv(4096)
                    if not packet:
                        return
                    data += packet

                payload = data[:size]
                data    = data[size:]

                # Parse JSON
                geo = json.loads(
                    payload.decode('utf-8'))

                # Publish all topics
                self.publish_all(geo)

            except json.JSONDecodeError as e:
                self.get_logger().error(
                    f'JSON error: {e}')
            except Exception as e:
                if self.running:
                    self.get_logger().error(
                        f'Error: {e}')
                break

    # ── Publish all topics ────────────────────────────────
    def publish_all(self, geo):
        now = self.get_clock().now().to_msg()

        # /lane_mask — colored segmentation image
        if geo.get('mask') is not None:
            mask = np.array(
                geo['mask'], dtype=np.uint8)
            color_mask = np.zeros(
                (*mask.shape, 3),
                dtype=np.uint8)
            color_mask[mask == 1] = [0, 255, 0]
            color_mask[mask == 2] = [0, 255, 255]
            mask_full = cv2.resize(
                color_mask, (640, 480),
                interpolation=cv2.INTER_NEAREST)
            img_msg = self.bridge.cv2_to_imgmsg(
                mask_full, 'bgr8')
            img_msg.header.stamp    = now
            img_msg.header.frame_id = 'camera'
            self.pub_mask.publish(img_msg)

        # /lane_center — x position 0 to 320
        msg = Float32()
        msg.data = float(geo.get('center', 160.0))
        self.pub_center.publish(msg)

        # /lane_error — pixels from center
        # positive = car right of lane center
        # negative = car left of lane center
        # PID should steer left when positive
        # PID should steer right when negative
        msg = Float32()
        msg.data = float(geo.get('error', 0.0))
        self.pub_error.publish(msg)

        # /current_lane — 'outer', 'inner', 'unknown'
        msg = String()
        msg.data = str(geo.get('lane', 'unknown'))
        self.pub_lane.publish(msg)

        # /lane_width — pixels
        msg = Float32()
        msg.data = float(geo.get('width', 0.0))
        self.pub_width.publish(msg)

        # /lane_curvature — slope of separator
        # 0.0 = straight
        # positive = turning one way
        # negative = turning other way
        msg = Float32()
        msg.data = float(
            geo.get('curvature', 0.0))
        self.pub_curv.publish(msg)

        # /lane_confidence — 0.0 to 1.0
        # 1.0 = both lines detected
        # 0.5 = one line detected
        # 0.2 = using previous frame
        # 0.0 = no detection
        msg = Float32()
        msg.data = float(
            geo.get('confidence', 0.0))
        self.pub_conf.publish(msg)
        # /lane_error_cm
        msg = Float32()
        msg.data = float(geo.get('error_cm', 0.0))
        self.pub_error_cm.publish(msg)

        # /lane_width_cm
        msg = Float32()
        msg.data = float(geo.get('width_cm', 0.0))
        self.pub_width_cm.publish(msg)
        # Log every 7 frames (~1 Hz at 7 FPS)
        if not hasattr(self, '_cnt'):
            self._cnt = 0
        self._cnt += 1
        if self._cnt % 7 == 0:
            self.get_logger().info(
                f"Lane={geo.get('lane','?'):7s} | "
                f"Error={geo.get('error',0):+6.1f}px | "
                f"Conf={geo.get('confidence',0):.2f} | "
                f"Width={geo.get('width',0):.0f}px | "
                f"FPS={geo.get('fps',0):.1f}")

    def destroy_node(self):
        self.running = False
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
