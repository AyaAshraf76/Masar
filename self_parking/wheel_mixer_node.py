import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


class WheelMixer(Node):

    def __init__(self):

        super().__init__("wheel_mixer")

        self.speed = 0.0
        self.steer = 0.0
        self.direction = "STOP"

        self.max_rpm = 120.0

        self.create_subscription(
            Float32,
            "/target_speed",
            self.speed_callback,
            10)

        self.create_subscription(
            Float32,
            "/target_steering",
            self.steer_callback,
            10)

        # FIX: this was missing entirely. Without it the car could
        # never reverse into a slot, since path_planner always sends
        # a positive speed for the PARK state and expects the
        # actuation layer to apply the direction flip.
        self.create_subscription(
            String,
            "/parking_direction",
            self.direction_callback,
            10)

        self.rpm_pub = self.create_publisher(
            Float32MultiArray,
            "/wheel_rpm",
            10)

        self.timer = self.create_timer(
            0.02,
            self.control_loop)

        self.get_logger().info("Wheel Mixer Started")

    def speed_callback(self, msg):
        self.speed = msg.data

    def steer_callback(self, msg):
        self.steer = msg.data

    def direction_callback(self, msg):
        self.direction = msg.data

    def control_loop(self):

        speed = self.speed

        if self.direction == "REVERSE":
            speed = -speed
        elif self.direction == "STOP":
            speed = 0.0

        left = speed - self.steer
        right = speed + self.steer

        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))

        left_rpm = left * self.max_rpm
        right_rpm = right * self.max_rpm

        # Skid-steer 4-motor mapping: left side pair, right side pair
        rpm1 = left_rpm
        rpm4 = left_rpm

        rpm2 = right_rpm
        rpm3 = right_rpm

        msg = Float32MultiArray()

        msg.data = [
            rpm1,
            rpm2,
            rpm3,
            rpm4
        ]

        self.rpm_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = WheelMixer()

    rclpy.spin(node)

    # FIX: node.bridge does not exist on this class, this was a
    # leftover/copy-paste crash on shutdown.
    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
