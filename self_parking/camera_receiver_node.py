import socket
import struct
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraReceiver(Node):

    def __init__(self):
        super().__init__("camera_receiver")

        self.publisher = self.create_publisher(
            Image,
            "/camera/image_raw",
            10
        )

        self.bridge = CvBridge()

        HOST = "0.0.0.0"
        PORT = 9999

        self.server = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        self.server.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        self.server.bind((HOST, PORT))
        self.server.listen(1)

        self.get_logger().info("Waiting for Raspberry Pi...")

        self.conn, addr = self.server.accept()

        self.get_logger().info(f"Connected: {addr}")

        self.timer = self.create_timer(
            0.01,
            self.receive_frame
        )

    def receive_frame(self):

        try:

            header = self.recvall(4)

            if header is None:
                return

            size = struct.unpack(">L", header)[0]

            data = self.recvall(size)

            if data is None:
                return

            frame = cv2.imdecode(
                np.frombuffer(data, np.uint8),
                cv2.IMREAD_COLOR
            )

            if frame is None:
                return

            msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding="bgr8"
            )

            self.publisher.publish(msg)

        except Exception as e:

            self.get_logger().error(str(e))

    def recvall(self, size):

        data = b''

        while len(data) < size:

            packet = self.conn.recv(size - len(data))

            if not packet:
                return None

            data += packet

        return data


def main():

    rclpy.init()

    node = CameraReceiver()

    rclpy.spin(node)

    node.conn.close()
    node.server.close()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
