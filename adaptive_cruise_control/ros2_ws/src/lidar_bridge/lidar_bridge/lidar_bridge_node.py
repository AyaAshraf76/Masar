#!/usr/bin/env python3
"""
LiDAR bridge: receives JSON scans from Pi lidar_stream over TCP,
publishes them as ROS2 sensor_msgs/LaserScan on /scan.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket, struct, json, math, threading

PI_IP = '192.168.1.5'      # <-- your Pi IP
TCP_PORT = 9997


class LidarBridge(Node):
    def __init__(self):
        super().__init__('lidar_bridge_node')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.sock = None
        self.connect()
        threading.Thread(target=self.receive_loop, daemon=True).start()

    def connect(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f'Connecting to Pi {PI_IP}:{TCP_PORT}...')
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((PI_IP, TCP_PORT))
                self.get_logger().info('Connected to Pi LiDAR stream')
                return
            except Exception as e:
                self.get_logger().warn(f'Connect failed: {e}, retrying in 2s')
                import time; time.sleep(2)

    def recv_exact(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def receive_loop(self):
        while rclpy.ok():
            try:
                raw_size = self.recv_exact(4)
                if not raw_size:
                    self.get_logger().warn('Pi disconnected, reconnecting...')
                    self.connect(); continue
                size = struct.unpack('>L', raw_size)[0]
                data = self.recv_exact(size)
                if not data:
                    continue
                points = json.loads(data.decode('utf-8'))
                self.publish_scan(points)
            except Exception as e:
                self.get_logger().error(f'Receive error: {e}')
                self.connect()

    def publish_scan(self, points):
        # points = list of [angle_deg, distance_mm]
        # Build a 360-bin LaserScan (1° resolution)
        ranges = [float('inf')] * 360
        for angle, dist in points:
            idx = int(round(angle)) % 360
            r = dist / 1000.0  # mm -> meters
            if r < ranges[idx]:
                ranges[idx] = r

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = math.pi / 180.0  # 1 degree
        msg.range_min = 0.10
        msg.range_max = 12.0
        msg.ranges = ranges
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
