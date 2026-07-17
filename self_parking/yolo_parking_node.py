import cv2
import numpy as np

from ultralytics import YOLO

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Bool

from cv_bridge import CvBridge


class YOLOParking(Node):

    def __init__(self):

        super().__init__("yolo_parking")

        self.bridge = CvBridge()

        self.get_logger().info("Loading YOLO...")

        self.model = YOLO(
            "/home/mariam/parking_ws/models/best.pt"
        )

        self.get_logger().info("YOLO Loaded")

        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/parking_detection",
            10
        )

        self.center_x_pub = self.create_publisher(
            Float32,
            "/parking_center_x",
            10
        )

        self.center_y_pub = self.create_publisher(
            Float32,
            "/parking_center_y",
            10
        )

        self.available_pub = self.create_publisher(
            Bool,
            "/parking_detected",
            10
        )

        self.width_pub = self.create_publisher(
            Float32,
            "/parking_width",
            10
        )

        self.height_pub = self.create_publisher(
            Float32,
            "/parking_height",
            10
        )

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        debug = frame.copy()

        results = self.model(
            frame,
            verbose=False
        )

        detected = False

        center_x = 0.0
        center_y = 0.0

        width = 0.0
        height = 0.0

        confidence = 0.0
        for result in results:

            boxes = result.boxes

            if boxes is None:
                continue

            for box in boxes:

                detected = True

                x1, y1, x2, y2 = (
                    box.xyxy[0]
                    .cpu()
                    .numpy()
                )

                confidence = float(
                    box.conf[0]
                )

                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0

                width = x2 - x1
                height = y2 - y1

                cv2.rectangle(
                    debug,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
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

                cv2.putText(
                    debug,
                    f"{confidence:.2f}",
                    (
                        int(x1),
                        int(y1)-10
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0,255,0),
                    2
                )

                break

            if detected:
                break

        available_msg = Bool()
        available_msg.data = detected

        self.available_pub.publish(
            available_msg
        )

        if detected:

            msgx = Float32()
            msgx.data = float(center_x)

            msgy = Float32()
            msgy.data = float(center_y)

            msgw = Float32()
            msgw.data = float(width)

            msgh = Float32()
            msgh.data = float(height)

            self.center_x_pub.publish(msgx)
            self.center_y_pub.publish(msgy)
            self.width_pub.publish(msgw)
            self.height_pub.publish(msgh)
        cv2.putText(
            debug,
            f"Detected : {detected}",
            (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0,255,0),
            2
        )

        if detected:

            cv2.putText(
                debug,
                f"Center : ({int(center_x)}, {int(center_y)})",
                (10,60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0,255,0),
                2
            )

            cv2.putText(
                debug,
                f"Size : {int(width)} x {int(height)}",
                (10,90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0,255,0),
                2
            )

            cv2.putText(
                debug,
                f"Conf : {confidence:.2f}",
                (10,120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0,255,0),
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

    node = YOLOParking()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
