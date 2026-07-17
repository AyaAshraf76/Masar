import socket
import struct

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class TCPBridge(Node):

    def __init__(self):

        super().__init__("tcp_bridge")

        # Raspberry Pi IP
        self.host = "192.168.1.5"

        # MUST match parking_bridge_server.py
        self.port = 5001

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.get_logger().info(
            f"Connecting to {self.host}:{self.port}"
        )

        self.sock.connect((self.host, self.port))

        self.get_logger().info(
            "Connected to Raspberry Pi."
        )

        self.create_subscription(
            Float32MultiArray,
            "/wheel_rpm",
            self.rpm_callback,
            10
        )

    def rpm_callback(self, msg):

        if len(msg.data) != 4:
            return

        try:

            packet = struct.pack(
                "ffff",
                float(msg.data[0]),
                float(msg.data[1]),
                float(msg.data[2]),
                float(msg.data[3])
            )

            self.sock.sendall(packet)

        except Exception as e:

            self.get_logger().error(
                f"Send Error: {e}"
            )

    def destroy_node(self):

        try:
            self.sock.close()
        except:
            pass

        super().destroy_node()


def main():

    rclpy.init()

    try:

        node = TCPBridge()

        rclpy.spin(node)

    except Exception as e:

        print(e)

    finally:

        try:
            node.destroy_node()
        except:
            pass

        rclpy.shutdown()


if __name__ == "__main__":
    main()
