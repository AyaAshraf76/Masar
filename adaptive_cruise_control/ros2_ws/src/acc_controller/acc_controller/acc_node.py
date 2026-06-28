#!/usr/bin/env python3
"""
Adaptive Cruise Control node - with object clustering.

Detects the obstacle CAR (known ~6cm width) and rejects the
continuous WALL beside the track by clustering LiDAR points
into objects and keeping only car-sized clusters.

Subscribes:
  /scan          (LaserScan)  - from lidar_bridge
  /current_lane  (String)     - from lane bridge
Publishes:
  /acc_speed_command (Float32)
  /acc_zone          (String)  GREEN/YELLOW/RED
  /acc_obstacle_lane (String)  which lane the car is in
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
import math

# ── LiDAR orientation ─────────────────────────────────────
FORWARD_ANGLE_DEG = 175.0

# Front cone to search (± half around forward)
FRONT_CONE_DEG = 60

# ── Track window ──────────────────────────────────────────
MIN_FORWARD_CM = 5.0
MAX_FORWARD_CM = 100.0
# Lateral limits are ASYMMETRIC and depend on ego lane (below)

# ── Object (car) width discrimination ─────────────────────
CAR_WIDTH_CM      = 6.0     # known black car width
CAR_WIDTH_MIN_CM  = 2.5     # min cluster width to be a car
CAR_WIDTH_MAX_CM  = 16.0    # max cluster width to be a car
                             # (wider = wall, reject)
CLUSTER_GAP_CM    = 5.0     # points farther apart than this
                             # start a new cluster

# ── Distance zones (cm) ───────────────────────────────────
RED_CM    = 35.0
GREEN_CM  = 60.0
# < RED → RED(stop); RED..GREEN → YELLOW(slow); > GREEN → GREEN

# ── Lane geometry ─────────────────────────────────────────
LANE_WIDTH_CM     = 36.7
LANE_TOLERANCE_CM = 22.0

# Asymmetric lateral gates (cm). The WALL is on the OUTER side.
# When in OUTER lane: don't look far outward (toward wall),
#   but do look inward (toward separator / inner lane).
# When in INNER lane: safe to look both ways (wall is far).
OUTER_LANE_LAT_INWARD_CM   = 50.0   # toward inner lane (safe)
OUTER_LANE_LAT_OUTWARD_CM  = 22.0   # toward wall (tight!)
INNER_LANE_LAT_INWARD_CM   = 22.0   # toward center island
INNER_LANE_LAT_OUTWARD_CM  = 50.0   # toward outer lane
UNKNOWN_LAT_CM             = 22.0   # default tight

# ── Speeds ────────────────────────────────────────────────
SPEED_GREEN  = 1.0
SPEED_YELLOW = 0.4
SPEED_RED    = 0.0


class ACCNode(Node):
    def __init__(self):
        super().__init__('acc_node')
        self.current_lane = 'unknown'

        self.create_subscription(LaserScan, '/scan',
                                 self.scan_cb, 10)
        self.create_subscription(String, '/current_lane',
                                 self.lane_cb, 10)

        self.speed_pub = self.create_publisher(
            Float32, '/acc_speed_command', 10)
        self.zone_pub = self.create_publisher(
            String, '/acc_zone', 10)
        self.obs_lane_pub = self.create_publisher(
            String, '/acc_obstacle_lane', 10)

        self.get_logger().info('ACC node started (clustering + wall reject)')
        self.get_logger().info(
            f'Car width={CAR_WIDTH_CM}cm  '
            f'accept clusters {CAR_WIDTH_MIN_CM}-{CAR_WIDTH_MAX_CM}cm')
        self.get_logger().info(
            f'Zones: RED<{RED_CM} YELLOW<{GREEN_CM} GREEN>={GREEN_CM}')

    def lane_cb(self, msg):
        self.current_lane = msg.data

    def lateral_limits(self):
        """Return (inward_limit, outward_limit) based on lane.
        inward = toward track center/separator,
        outward = toward outer wall."""
        if self.current_lane == 'outer':
            return OUTER_LANE_LAT_INWARD_CM, OUTER_LANE_LAT_OUTWARD_CM
        elif self.current_lane == 'inner':
            return INNER_LANE_LAT_INWARD_CM, INNER_LANE_LAT_OUTWARD_CM
        else:
            return UNKNOWN_LAT_CM, UNKNOWN_LAT_CM

    def scan_cb(self, msg):
        # 1) Collect valid points in the front cone as (fwd, lat)
        pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            angle_deg = math.degrees(msg.angle_min +
                                     i * msg.angle_increment)
            rel = (angle_deg - FORWARD_ANGLE_DEG + 540) % 360 - 180
            if abs(rel) > FRONT_CONE_DEG / 2.0:
                continue
            dist_cm = r * 100.0
            rr = math.radians(rel)
            fwd = dist_cm * math.cos(rr)
            lat = dist_cm * math.sin(rr)
            if fwd < MIN_FORWARD_CM or fwd > MAX_FORWARD_CM:
                continue
            pts.append((rel, fwd, lat))

        # sort by angle so clustering walks around the arc
        pts.sort(key=lambda p: p[0])

        # 2) Cluster consecutive points that are close in space
        clusters = []
        current = []
        prev_xy = None
        for rel, fwd, lat in pts:
            if prev_xy is None:
                current = [(fwd, lat)]
            else:
                d = math.hypot(fwd - prev_xy[0], lat - prev_xy[1])
                if d <= CLUSTER_GAP_CM:
                    current.append((fwd, lat))
                else:
                    clusters.append(current)
                    current = [(fwd, lat)]
            prev_xy = (fwd, lat)
        if current:
            clusters.append(current)

        # 3) Evaluate each cluster: width + lateral gate
        in_lim, out_lim = self.lateral_limits()
        best = None   # (fwd, lat, width)
        for cl in clusters:
            fwds = [p[0] for p in cl]
            lats = [p[1] for p in cl]
            width = math.hypot(
                max(fwds) - min(fwds),
                max(lats) - min(lats))   # cluster extent
            cx_fwd = sum(fwds) / len(fwds)
            cx_lat = sum(lats) / len(lats)

            # width discrimination: car-sized only, reject wall
            if width < CAR_WIDTH_MIN_CM or width > CAR_WIDTH_MAX_CM:
                continue

            # asymmetric lateral gate (inward positive? depends
            # on lane). We treat outward = toward wall side.
            # For OUTER lane, wall is on the +lat OR -lat side
            # depending on LiDAR mounting; use symmetric-ish but
            # tight on the wall side. Simplify: apply the tighter
            # 'outward' limit on the side matching the wall.
            # Here we gate by magnitude using the larger of the
            # two only toward track, tighter toward wall.
            if cx_lat >= 0:
                lat_limit = in_lim
            else:
                lat_limit = out_lim
            if abs(cx_lat) > lat_limit:
                continue

            if best is None or cx_fwd < best[0]:
                best = (cx_fwd, cx_lat, width)

        # 4) Decide zone
        if best is None:
            zone, speed = 'GREEN', SPEED_GREEN
            obstacle_lane = 'none'
            self.zone_pub.publish(String(data=zone))
            self.speed_pub.publish(Float32(data=float(speed)))
            self.obs_lane_pub.publish(String(data=obstacle_lane))
            self.get_logger().info('Clear road (no car cluster) -> GREEN')
            return

        fwd, lat, width = best
        obstacle_lane = self.classify_lane(lat)

        if fwd < RED_CM:
            zone, speed = 'RED', SPEED_RED
        elif fwd < GREEN_CM:
            zone, speed = 'YELLOW', SPEED_YELLOW
        else:
            zone, speed = 'GREEN', SPEED_GREEN

        # Only brake for obstacle in OUR lane
        if obstacle_lane != self.current_lane and \
           obstacle_lane != 'unknown':
            zone, speed = 'GREEN', SPEED_GREEN

        self.zone_pub.publish(String(data=zone))
        self.speed_pub.publish(Float32(data=float(speed)))
        self.obs_lane_pub.publish(String(data=obstacle_lane))

        self.get_logger().info(
            f'CAR: {fwd:.0f}cm fwd, {lat:+.0f}cm lat, '
            f'width={width:.1f}cm, lane={obstacle_lane}, '
            f'ego={self.current_lane} -> {zone} speed={speed:.1f}')

    def classify_lane(self, lateral_cm):
        if abs(lateral_cm) < LANE_TOLERANCE_CM:
            return self.current_lane
        elif lateral_cm >= LANE_TOLERANCE_CM:
            return 'inner' if self.current_lane == 'outer' else 'outer'
        else:
            return 'outer' if self.current_lane == 'inner' else 'inner'


def main():
    rclpy.init()
    node = ACCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
