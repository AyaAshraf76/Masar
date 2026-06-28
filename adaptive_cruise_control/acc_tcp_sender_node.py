#!/usr/bin/env python3
"""
ROS2 node that forwards ACC speed commands to the Pi.
Subscribes to /acc_speed_command and sends the value over
TCP to the Pi's acc_motor_server.py.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket
import json
import time

PI_IP = "192.168.1.5"
TCP_PORT = 9998


class AccTcpSender(Node):
    def __init__(self):
        super().__init__("acc_tcp_sender_node")
        self.sock = None
        self.connect()
        self.create_subscription(Float32, "/acc_speed_command", self.cb, 10)

    def connect(self):
        while rclpy.ok():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((PI_IP, TCP_PORT))
                self.get_logger().info("Connected to Pi ACC motor server")
                return
            except Exception as e:
                self.get_logger().warn(f"Connection failed: {e}")
                time.sleep(1)

    def cb(self, msg):
        try:
            payload = json.dumps({"speed": float(msg.data)}).encode("utf-8")
            self.sock.sendall(payload)
        except Exception:
            self.get_logger().warn("Lost connection to Pi, reconnecting...")
            self.connect()


def main():
    rclpy.init()
    node = AccTcpSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
