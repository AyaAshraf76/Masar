import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32

from cv_bridge import CvBridge


class ParkingSlotNode(Node):

    def __init__(self):

        super().__init__("parking_slot")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            "/lane_classes",
            self.image_callback,
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/parking_slot_debug",
            10
        )

        self.available_pub = self.create_publisher(
            Bool,
            "/parking_slot_available",
            10
        )

        self.center_x_pub = self.create_publisher(
            Float32,
            "/parking_slot_center_x",
            10
        )

        self.center_y_pub = self.create_publisher(
            Float32,
            "/parking_slot_center_y",
            10
        )

        self.width_pub = self.create_publisher(
            Float32,
            "/parking_slot_width",
            10
        )

        self.height_pub = self.create_publisher(
            Float32,
            "/parking_slot_height",
            10
        )

        self.get_logger().info("Parking Slot Node Started")
    def image_callback(self, msg):

        classes = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="mono8"
        )

        parking = (classes == 2).astype(np.uint8)

        kernel = np.ones((5,5), np.uint8)

        parking = cv2.morphologyEx(
            parking,
            cv2.MORPH_CLOSE,
            kernel
        )

        parking = cv2.morphologyEx(
            parking,
            cv2.MORPH_OPEN,
            kernel
        )

        debug = cv2.cvtColor(
            parking * 255,
            cv2.COLOR_GRAY2BGR
        )

        contours, _ = cv2.findContours(
            parking,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detected = False
        center_x = 0.0
        center_y = 0.0
        width = 0.0
        height = 0.0

        if len(contours) > 0:

            largest = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(largest)

            if area > 300:

                x, y, w, h = cv2.boundingRect(
                    largest
                )

                detected = True

                center_x = x + w / 2.0
                center_y = y + h / 2.0

                width = float(w)
                height = float(h)

                cv2.rectangle(
                    debug,
                    (x, y),
                    (x + w, y + h),
                    (0,255,0),
                    2
                )

                cv2.circle(
                    debug,
                    (
                        int(center_x),
                        int(center_y)
                    ),
                    5,
                    (0,0,255),
                    -1
                )
        available_msg = Bool()
        available_msg.data = detected

        self.available_pub.publish(
            available_msg
        )

        if detected:

            msgx = Float32()
            msgx.data = center_x

            msgy = Float32()
            msgy.data = center_y

            msgw = Float32()
            msgw.data = width

            msgh = Float32()
            msgh.data = height

            self.center_x_pub.publish(msgx)
            self.center_y_pub.publish(msgy)
            self.width_pub.publish(msgw)
            self.height_pub.publish(msgh)

            cv2.putText(
                debug,
                "PARKING SLOT",
                (10,30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0,255,0),
                2
            )

            cv2.putText(
                debug,
                f"Center: ({int(center_x)}, {int(center_y)})",
                (10,60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255,255,255),
                2
            )

            cv2.putText(
                debug,
                f"Size: {int(width)} x {int(height)}",
                (10,90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255,255,255),
                2
            )

        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(
                debug,
                encoding="bgr8"
            )
        )


def main():

    rclpy.init()

    node = ParkingSlotNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
