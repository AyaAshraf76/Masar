# Legacy Motor Test Scripts

Early prototype firmware/scripts used while bringing up the drivetrain,
**before** the final PID+odometry firmware. Superseded by
`firmware/stm32_pid_uart/pid_realtime_uart.ino` +
`trajectory_control/stm32_bridge_uart.py`. Kept here for history only
— pin mappings in these files do **not** match the final control PCB.

| Folder | Purpose |
|---|---|
| `modes_feedback_motors/` | STM32 firmware (`modes_feedback_motors.ino`) + Pi-side script (`modes_pi.py`) for an interactive mode-select UART test: 4-wheel speed control vs. steering+speed control, with encoder feedback. Different pin mapping than the final firmware. |
| `motors_keyboard/` | Earliest open-loop test: STM32 firmware (`motors_keyboard.ino`, PWM/DIR only, no encoders/PID) driven from the Pi by raw keyboard input (`keyboard_motor.py`). |

If you don't need this history for the thesis appendix, this whole
folder can be deleted without affecting anything else in the repo.
