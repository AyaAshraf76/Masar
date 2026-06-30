# Running the LiDAR / ACC Stack

## Normal run (three terminals + the Pi)

```bash
# Pi (SSH):
ssh pi@192.168.1.5
source ~/lane_env/bin/activate
python3 /home/pi/lidar_stream.py

# Laptop, terminal 1 — LiDAR bridge:
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run lidar_bridge lidar_bridge_node

# Laptop, terminal 2 — ACC controller:
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run acc_controller acc_node

# Laptop, terminal 3 — send speed back to Pi:
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run acc_tcp_sender acc_tcp_sender_node

# Laptop, terminal 4 — monitor:
ros2 topic echo /acc_zone
ros2 topic echo /acc_speed_command
```

## Quick test without building ROS2 packages

For quick tests, `acc_node.py` can run as a standalone script — it just needs `rclpy` on the path (sourcing ROS2 setup is enough).
