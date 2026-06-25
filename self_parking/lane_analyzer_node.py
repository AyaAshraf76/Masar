import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class LaneAnalyzer(Node):

    def __init__(self):

        super().__init__("lane_analyzer")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            "/lane_classes",
            self.image_callback,
            10
        )

        self.center_pub = self.create_publisher(
            Float32,
            "/lane_center",
            10
        )

        self.error_pub = self.create_publisher(
            Float32,
            "/lane_error",
            10
        )

        self.width_pub = self.create_publisher(
            Float32,
            "/lane_width",
            10
        )

        self.heading_pub = self.create_publisher(
            Float32,
            "/lane_heading",
            10
        )

        self.confidence_pub = self.create_publisher(
            Float32,
            "/lane_confidence",
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/lane_debug",
            10
        )

        self.get_logger().info("Lane Analyzer Started")

    def image_callback(self, msg):

        lane = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="mono8"
        )

        lane = (lane == 1).astype(np.uint8)

        kernel = np.ones((5,5), np.uint8)

        lane = cv2.morphologyEx(
            lane,
            cv2.MORPH_OPEN,
            kernel
        )

        lane = cv2.morphologyEx(
            lane,
            cv2.MORPH_CLOSE,
            kernel
        )

        contours, _ = cv2.findContours(
            lane,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        debug = np.zeros(
            (lane.shape[0], lane.shape[1],3),
            dtype=np.uint8
        )

        if len(contours) == 0:

            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(
                    debug,
                    encoding="bgr8"
                )
            )

            return

        largest = max(
            contours,
            key=cv2.contourArea
        )

        cv2.drawContours(
            debug,
            [largest],
            -1,
            (0,255,0),
            2
        )

        mask = np.zeros_like(lane)

        cv2.drawContours(
            mask,
            [largest],
            -1,
            255,
            -1
        )

        centers = []
        widths = []
        left_points = []
        right_points = []

        rows = np.arange(
            int(mask.shape[0]*0.70),
            mask.shape[0],
            5
        )

        for y in rows:

            xs = np.where(mask[y] > 0)[0]

            if len(xs) < 2:
                continue

            left = xs[0]
            right = xs[-1]

            center = (left + right) / 2.0

            left_points.append(left)
            right_points.append(right)

            centers.append(center)
            widths.append(right-left)

            cv2.circle(
                debug,
                (int(center),int(y)),
                2,
                (0,0,255),
                -1
            )

            cv2.circle(
                debug,
                (left,int(y)),
                2,
                (255,0,0),
                -1
            )

            cv2.circle(
                debug,
                (right,int(y)),
                2,
                (255,255,0),
                -1
            )

        if len(centers) < 3:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(
                    debug,
                    encoding="bgr8"
                )
            )
            return

        center = float(np.mean(centers))
        width = float(np.mean(widths))

        image_center = mask.shape[1] / 2.0

        error = image_center - center

        ys = rows[:len(centers)]

        if len(ys) >= 2:

            p = np.polyfit(
                ys,
                centers,
                1
            )

            slope = p[0]

            heading = float(
                np.degrees(
                    np.arctan(slope)
                )
            )

            y1 = int(ys[0])
            y2 = int(ys[-1])

            x1 = int(np.polyval(p, y1))
            x2 = int(np.polyval(p, y2))

            cv2.line(
                debug,
                (x1, y1),
                (x2, y2),
                (0, 255, 255),
                2
            )

        else:

            heading = 0.0

        confidence = float(
            len(centers) / len(rows)
        )

        cmsg = Float32()
        cmsg.data = center

        emsg = Float32()
        emsg.data = error

        wmsg = Float32()
        wmsg.data = width

        hmsg = Float32()
        hmsg.data = heading

        confmsg = Float32()
        confmsg.data = confidence

        self.center_pub.publish(cmsg)
        self.error_pub.publish(emsg)
        self.width_pub.publish(wmsg)
        self.heading_pub.publish(hmsg)
        self.confidence_pub.publish(confmsg)

        cv2.line(
            debug,
            (int(image_center), 0),
            (int(image_center), debug.shape[0]),
            (255, 255, 255),
            2
        )

        cv2.line(
            debug,
            (int(center), 0),
            (int(center), debug.shape[0]),
            (0, 0, 255),
            2
        )

        cv2.putText(
            debug,
            f"Center : {center:.1f}",
            (10,30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255,255,255),
            2
        )

        cv2.putText(
            debug,
            f"Error : {error:.1f}",
            (10,60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255,255,255),
            2
        )

        cv2.putText(
            debug,
            f"Width : {width:.1f}",
            (10,90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255,255,255),
            2
        )

        cv2.putText(
            debug,
            f"Heading : {heading:.1f}",
            (10,120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255,255,255),
            2
        )

        cv2.putText(
            debug,
            f"Confidence : {confidence:.2f}",
            (10,150),
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

    node = LaneAnalyzer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
