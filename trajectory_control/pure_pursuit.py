"""
Pure pursuit path-tracking controller.

The idea: imagine a point on the path some fixed distance ahead of the
car (the "lookahead point"). Steer toward that point as if chasing it.
The further ahead you look, the smoother/wider the turn; the closer you
look, the tighter/more aggressive the turn (but more prone to oscillating).

This module only needs:
- the car's current pose (x, y, theta)
- a list of waypoints describing the path to follow
It outputs the (v, omega) needed to steer toward the lookahead point.
"""

import math


class PurePursuit:
    def __init__(self, lookahead_distance, target_speed):
        self.Ld = lookahead_distance   # meters -- how far ahead to look on the path
        self.target_speed = target_speed  # m/s -- constant cruise speed (kept simple for now)

    def find_lookahead_point(self, x, y, waypoints, last_index):
        """
        Search forward along the waypoint list (starting from last_index,
        so we never search backwards) for the first point that is at
        least Ld meters away from the car's current position.

        Returns (point, index) where point is (wx, wy).
        If no point is far enough away (near the end of the path),
        returns the last waypoint.
        """
        for i in range(last_index, len(waypoints)):
            wx, wy = waypoints[i]
            dist = math.hypot(wx - x, wy - y)
            if dist >= self.Ld:
                return (wx, wy), i
        # Reached the end of the path without finding a point far enough away
        return waypoints[-1], len(waypoints) - 1

    def compute_control(self, x, y, theta, waypoints, last_index):
        """
        Core pure pursuit step. Given the car's current pose and the
        path, returns (v, omega, new_last_index, lookahead_point).

        Geometry explanation:
        - Transform the lookahead point into the car's own reference frame
          (i.e. "how far ahead and how far sideways is this point, from
          the car's point of view").
        - The sideways offset in this local frame tells us how much we
          need to curve to reach it.
        - curvature kappa = 2 * lateral_offset / Ld^2  (standard pure pursuit formula)
        - omega = v * kappa  (angular velocity needed to follow that curvature at speed v)
        """
        lookahead_pt, new_index = self.find_lookahead_point(x, y, waypoints, last_index)
        wx, wy = lookahead_pt

        # Vector from car to lookahead point, in global frame
        dx = wx - x
        dy = wy - y

        # Rotate that vector into the car's local frame (car's heading = theta)
        # local_x = forward distance, local_y = sideways distance (positive = left)
        local_x = dx * math.cos(-theta) - dy * math.sin(-theta)
        local_y = dx * math.sin(-theta) + dy * math.cos(-theta)

        # Actual distance to the lookahead point (may differ slightly from self.Ld
        # if we hit the end of the path before reaching Ld)
        Ld_actual = math.hypot(local_x, local_y)
        if Ld_actual < 1e-6:
            # Car is essentially on top of the lookahead point, avoid divide-by-zero
            return self.target_speed, 0.0, new_index, lookahead_pt

        # Pure pursuit curvature formula
        curvature = (2.0 * local_y) / (Ld_actual ** 2)

        v = self.target_speed
        omega = v * curvature

        return v, omega, new_index, lookahead_pt
