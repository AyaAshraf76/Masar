"""
Differential drive kinematic model.

This is the same math running on the STM32 for odometry, just here in
Python so we can simulate it quickly without re-flashing hardware every
time we change a parameter.

Two directions:
- forward kinematics: wheel speeds (v_left, v_right) -> robot motion (v, omega)
  Used for ODOMETRY (estimating where the robot is from wheel motion).
- inverse kinematics: desired robot motion (v, omega) -> wheel speeds (v_left, v_right)
  Used for CONTROL (converting a planner's desired motion into wheel commands).
"""

import math


class DiffDriveKinematics:
    def __init__(self, wheel_radius, track_width):
        self.r = wheel_radius   # meters, calibrated value
        self.L = track_width    # meters, distance between left/right wheel centers

    def inverse(self, v, omega):
        """
        Desired robot motion -> required wheel speeds.

        v     : desired linear velocity of the robot center (m/s)
        omega : desired angular velocity of the robot (rad/s), positive = turning left (CCW)

        Returns (v_left, v_right) in m/s -- the speed each side's wheels
        need to spin at to produce that motion.
        """
        v_left = v - (omega * self.L) / 2.0
        v_right = v + (omega * self.L) / 2.0
        return v_left, v_right

    def forward(self, v_left, v_right):
        """
        Wheel speeds -> resulting robot motion.

        v_left, v_right : current speed of left/right side wheels (m/s)

        Returns (v, omega) -- the robot's resulting linear and angular velocity.
        This is the direction used for odometry.
        """
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.L
        return v, omega

    def mps_to_rpm(self, speed_mps):
        """Convert a wheel's linear speed (m/s) to RPM, for sending to your PID targets."""
        wheel_circumference = 2 * math.pi * self.r
        rps = speed_mps / wheel_circumference  # revolutions per second
        return rps * 60.0

    def rpm_to_mps(self, rpm):
        """Convert RPM (from encoder feedback) to linear wheel speed (m/s)."""
        wheel_circumference = 2 * math.pi * self.r
        rps = rpm / 60.0
        return rps * wheel_circumference
