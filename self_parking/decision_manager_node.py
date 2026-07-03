import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import String


class DecisionManager(Node):

    def __init__(self):

        super().__init__("decision_manager")

        self.lane_error = 0.0

        self.slot_detected = False
        self.slot_center_x = 0.0
        self.slot_width = 0.0
        self.slot_height = 0.0

        self.yolo_detected = False

        # FIX: nothing previously decided when the car had actually
        # arrived in the slot. path_planner would then keep issuing
        # REVERSE forever. We use the parking slot's bounding-box
        # area in the segmentation view as a simple proxy for "how
        # close are we": as the car backs toward the slot marking,
        # the visible slot patch grows. Once it crosses this
        # threshold we call it parked and latch that decision so it
        # doesn't flicker back to PARK on a noisy frame.
        #
        # NOTE: this threshold is a placeholder. Calibrate it by
        # driving the manual REVERSE sequence yourself, watching
        # /parking_slot_debug, and recording width*height at the
        # position you consider "fully parked".
        self.PARK_AREA_THRESHOLD = 35000.0

        self.parked = False

        self.create_subscription(
            Float32,
            "/lane_error",
            self.lane_callback,
            10
        )

        self.create_subscription(
            Bool,
            "/parking_slot_available",
            self.slot_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/parking_slot_center_x",
            self.slot_center_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/parking_slot_width",
            self.slot_width_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/parking_slot_height",
            self.slot_height_callback,
            10
        )

        self.create_subscription(
            Bool,
            "/parking_detected",
            self.yolo_callback,
            10
        )

        self.state_pub = self.create_publisher(
            String,
            "/parking_state",
            10
        )

        self.target_pub = self.create_publisher(
            Float32,
            "/parking_target_x",
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.update
        )

        self.get_logger().info(
            "Decision Manager Started"
        )

    def lane_callback(self, msg):
        self.lane_error = msg.data

    def slot_callback(self, msg):
        self.slot_detected = msg.data

    def slot_center_callback(self, msg):
        self.slot_center_x = msg.data

    def slot_width_callback(self, msg):
        self.slot_width = msg.data

    def slot_height_callback(self, msg):
        self.slot_height = msg.data

    def yolo_callback(self, msg):
        self.yolo_detected = msg.data

    def update(self):

        state = "SEARCH"
        target = 0.0

        if self.parked:

            state = "DONE"
            target = 0.0

        elif self.slot_detected:

            area = self.slot_width * self.slot_height

            if area <= self.PARK_AREA_THRESHOLD:

                self.parked = True
                state = "DONE"
                target = 0.0

                self.get_logger().info(
                    f"Parking complete (slot area={area:.0f})"
                )

            elif not self.yolo_detected:

                state = "PARK"
                target = self.slot_center_x

            else:

                state = "WAIT"

        else:

            state = "FOLLOW_LANE"
            target = self.lane_error

        state_msg = String()
        state_msg.data = state

        target_msg = Float32()
        target_msg.data = float(target)

        self.state_pub.publish(state_msg)
        self.target_pub.publish(target_msg)


def main():

    rclpy.init()

    node = DecisionManager()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
