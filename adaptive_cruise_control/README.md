# Adaptive Cruise Control (LiDAR)

LiDAR-based ACC system. The Pi streams LiDAR scans to the laptop, ROS2 nodes process them into speed decisions, and the result is sent back to control the motors.

## Files

- `lidar_stream.py` — runs on Pi, reads RPLIDAR A1M8 and streams scans over TCP (port 9997)
- `acc_motor_server.py` — runs on Pi, receives speed commands from laptop via TCP and sends RPM targets to STM32 over UART
- `acc_tcp_sender_node.py` — ROS2 node on laptop, forwards `/acc_speed_command` to the Pi over TCP
- `ros2_ws/src/lidar_bridge/` — ROS2 node that converts the TCP scan stream to `/scan` topic
- `ros2_ws/src/acc_controller/` — ROS2 node that processes `/scan` into speed zones (GREEN/YELLOW/RED)

## How it works

The ACC node filters LiDAR points to a forward cone, clusters them, and keeps only car-sized clusters (rejects the track wall). It uses lane-aware lateral gates so the wall is never mistaken for a car. Publishes `/acc_speed_command` and `/acc_zone`.

## Running

```bash
# Pi terminal 1: LiDAR stream
source ~/lane_env/bin/activate
python3 lidar_stream.py

# Pi terminal 2: motor server (receives speed from laptop, sends to STM32)
python3 acc_motor_server.py

# Laptop terminal 1 — LiDAR bridge:
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run lidar_bridge lidar_bridge_node

# Laptop terminal 2 — ACC controller:
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run acc_controller acc_node

# Laptop terminal 3 — send speed back to Pi:
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run acc_tcp_sender acc_tcp_sender_node

# Monitor:
ros2 topic echo /acc_zone
ros2 topic echo /acc_speed_command
```

For quick tests without building ROS2 packages, `acc_node.py` can run as a standalone script — just source the ROS2 setup so `rclpy` is on the path.
