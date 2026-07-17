# Self-Parking

ROS2 node pipeline that lets the car detect a parking slot, decide
whether to enter it, and drive itself in — all on the same racetrack
used for lane keeping/ACC, in the dedicated parking-slot area.

These currently live as plain node scripts (not yet wrapped into a
colcon package like `lane_bridge` / `acc_controller`) — see the
**Packaging** note at the bottom.

## Pipeline / data flow

```
camera_receiver_node ──▶ /camera/image_raw ──┬──▶ unet_node ──▶ /lane_mask, /lane_class
                                              │         │
                                              │         ▼
                                              │   lane_analyzer_node ──▶ /lane_center, /lane_error,
                                              │                          /lane_width, /lane_heading,
                                              │                          /lane_confidence
                                              │
                                              └──▶ yolo_parking_node ──▶ /parking_slot_available,
                                                    parking_slot_node       /parking_slot_center_x/y,
                                                                            /parking_slot_width/height

/lane_error + parking-slot topics ──▶ decision_manager_node ──▶ /parking_state, /parking_target_x

/parking_state, /parking_target_x, /lane_error ──▶ path_planner_node ──▶ /target_steering,
                                                                          /target_speed,
                                                                          /parking_direction

/target_steering, /target_speed, /parking_direction ──▶ wheel_mixer_node ──▶ /wheel_rpm

/wheel_rpm ──▶ tcp_bridge_node ──▶ (TCP) ──▶ Raspberry Pi / STM32 motors
```

## Files

| File | Role |
|---|---|
| `camera_receiver_node.py` | Receives camera frames over TCP/socket and publishes `sensor_msgs/Image` on `/camera/image_raw`. |
| `unet_node.py` | Runs the U-Net segmentation model on each frame, publishes the lane mask and class. |
| `lane_analyzer_node.py` | Turns the segmentation mask into lane center, error, width, heading, and confidence. |
| `yolo_parking_node.py` | YOLO-based detection of the parking slot marker/box. |
| `parking_slot_node.py` | Turns the YOLO detection into slot availability + geometry (`center_x`, `center_y`, `width`, `height`). |
| `decision_manager_node.py` | State machine: decides when to leave lane-following and commit to a parking maneuver, publishes `/parking_state` and `/parking_target_x`. |
| `path_planner_node.py` | Converts the current decision state + lane/slot geometry into steering, speed, and parking-direction commands. |
| `wheel_mixer_node.py` | Mixes steering/speed/direction into individual `/wheel_rpm` targets for all four motors. |
| `tcp_bridge_node.py` | Sends the final `/wheel_rpm` targets over TCP to the Raspberry Pi / STM32 motor firmware. |

## Packaging

For consistency with `perception/lane_detection/ros2_ws` and
`adaptive_cruise_control/ros2_ws`, these nine nodes should eventually
be wrapped into one or two proper `ament_python` ROS2 packages (e.g.
`self_parking_perception` for the camera/U-Net/YOLO/lane-analyzer
nodes and `self_parking_control` for decision-manager/path-planner/
wheel-mixer/tcp-bridge) with their own `package.xml` / `setup.py`. Left
as-is for now since that wasn't part of the requested reorganization —
happy to do that pass next if useful.
