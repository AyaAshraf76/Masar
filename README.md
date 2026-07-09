# Masar — Autonomous Self-Driving Car

Graduation project: a self-driving RC car with lane keeping, lane changing, adaptive cruise control (ACC), and self-parking. Built on a Raspberry Pi 5 + STM32 platform with LiDAR and camera, tested on a custom racetrack with two lanes and a parking area.

## Repo structure

| Folder | What's in it |
|---|---|
| `firmware/` | STM32 motor PID + encoder firmware |
| `trajectory_control/` | Kinematics, pure pursuit, trajectory controller (runs on Pi) |
| `perception/lane_detection/` | ONNX lane detection model + ROS2 lane_bridge + viewer |
| `adaptive_cruise_control/` | LiDAR streaming + ROS2 ACC nodes + motor server |
| `self_parking/` | ROS2 self-parking pipeline (camera, U-Net, YOLO, path planner) |
| `rl_lane_keeping/` | PPO training scripts for lane keeping in simulation (v3 → v3.3) |
| `simulation/` | Unity sim build + gym-donkeycar submodule |
| `car_design/` | Mechanical CAD + PCB files (to be added) |
| `dev_tools/` | Old motor test scripts, kept for reference |

## System overview

The car runs a Raspberry Pi 5 as the main computer and an STM32 for low-level motor control, connected over UART. ROS2 Humble runs on a laptop and handles the heavier processing (ACC, self-parking decisions). Sensor data goes from the Pi to the laptop over TCP, and speed commands come back the same way.

```
Raspberry Pi 5                         Laptop (ROS2)
───────────────                        ─────────────
lane_stream_final.py  ── TCP:9999 ──>  lane_bridge_node    -> /lane_error, /current_lane
                       └─ TCP:9998 ──>  lane_viewer.py

lidar_stream.py       ── TCP:9997 ──>  lidar_bridge_node   -> /scan
                                        acc_node            -> /acc_speed_command, /acc_zone
acc_motor_server.py   <── TCP:9998 ──  acc_tcp_sender_node

trajectory_angle_v11.py
  -> kinematics.py / pure_pursuit.py -> stm32_bridge_uart.py -- UART --> STM32 --> motors
```

## Hardware

- Raspberry Pi 5 — camera inference, trajectory control
- STM32 — motor PWM, encoder feedback, odometry
- Pi Camera — lane detection input (320×240 RGB → ONNX segmentation)
- RPLIDAR A1M8 — obstacle detection for ACC
- 4WD differential drive chassis, two lanes 35cm wide each

## Lane detection

3-class semantic segmentation (background / solid boundary / lane separator) running on the Pi at ~5 FPS. The model outputs pixel-level masks, post-processing extracts lane geometry (center, width, which lane, error in cm, curvature). Two calibration values per lane convert pixel error to real-world centimeters.

## Lane keeping + lane changing

The trajectory controller is a state machine:

- **LANE_KEEPING** — PID on `error_cm` from camera, with confidence-scaled low-pass, innovation clamp, leaky integral for corners, and corner slowdown
- **LANE_CHANGING** — pure pursuit following S-curve waypoints from current pose to adjacent lane
- **STOPPED** — motors off (ACC red zone or manual stop)

## ACC

LiDAR zones: GREEN (full speed), YELLOW (slow), RED (stop). The ACC node filters points to a forward cone, clusters them, and rejects wall-sized returns. Only brakes for obstacles in the car's own lane — if the other lane is clear, it can trigger a lane change instead.

## RL training (simulation)

PPO agent trained in DonkeyCar simulator. Key iterations:

- **v3** — switched to steering-rate actions (policy outputs a delta, not absolute angle) to fix bang-bang behavior. Made reward always ≥ 0 while alive so crashing is never better than staying on track
- **v3.2** — added damping bonus to penalize high lateral speed near the target. Added always-on speed term for the throttle head
- **v3.3** — widened damping window with glide-slope approach. Capped speed during lane changes to reduce entry oscillation

## Getting the code

```bash
git clone --recurse-submodules <repo-url>
# or if already cloned:
git submodule update --init --recursive
```
