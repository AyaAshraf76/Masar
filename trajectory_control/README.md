# Trajectory Control

High-level trajectory controller running on the Raspberry Pi. Reads lane detection data and ACC commands, computes motor speeds using PID lane keeping or pure pursuit lane changing, and sends them to the STM32 over UART.

## Files

- `trajectory_angle_v11.py` — main controller (state machine: LANE_KEEPING / LANE_CHANGING / STOPPED)
- `kinematics.py` — differential drive forward/inverse kinematics
- `pure_pursuit.py` — pure pursuit path follower for lane changes
- `stm32_bridge_uart.py` — UART interface to the STM32 (send targets, read odometry)

## Running

```bash
python3 trajectory_angle_v11.py
```

Connects to lane_stream_final.py on port 9999 and to the STM32 on `/dev/ttyAMA0`.
