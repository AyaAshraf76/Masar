import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import String


class PathPlanner(Node):

    def __init__(self):

        super().__init__("path_planner")

        self.state = "FOLLOW_LANE"

        self.target_x = 0.0
        self.heading = 0.0

        self.create_subscription(
            String,
            "/parking_state",
            self.state_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/parking_target_x",
            self.target_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/lane_heading",
            self.heading_callback,
            10
        )

        self.steering_pub = self.create_publisher(
            Float32,
            "/target_steering",
            10
        )

        self.speed_pub = self.create_publisher(
            Float32,
            "/target_speed",
            10
        )

        self.direction_pub = self.create_publisher(
            String,
            "/parking_direction",
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.update
        )

        self.get_logger().info(
            "Path Planner Started"
        )

    def state_callback(self, msg):
        self.state = msg.data

    def target_callback(self, msg):
        self.target_x = msg.data

    def heading_callback(self, msg):
        self.heading = msg.data

    def update(self):

        steering = 0.0
        speed = 0.0
        direction = "STOP"

        if self.state == "FOLLOW_LANE":

            # small heading term added on top of the pure error term,
            # otherwise /lane_heading was received but never used
            steering = (self.target_x * 0.01) + (self.heading * 0.05)
            speed = 0.50
            direction = "FORWARD"

        elif self.state == "PARK":

            steering = self.target_x * 0.015
            speed = 0.25
            direction = "REVERSE"

        elif self.state == "WAIT":

            steering = 0.0
            speed = 0.0
            direction = "STOP"

        elif self.state == "DONE":

            # FIX: previously there was no terminal state at all, so
            # the car would reverse indefinitely once in PARK.
            steering = 0.0
            speed = 0.0
            direction = "STOP"

        steering = max(min(steering, 30.0), -30.0)

        steering_msg = Float32()
        steering_msg.data = float(steering)

        speed_msg = Float32()
        speed_msg.data = float(speed)

        direction_msg = String()
        direction_msg.data = direction

        self.steering_pub.publish(steering_msg)
        self.speed_pub.publish(speed_msg)
        self.direction_pub.publish(direction_msg)


def main():

    rclpy.init()

    node = PathPlanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
