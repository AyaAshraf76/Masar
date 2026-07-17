# STM32 Firmware

PID motor control firmware for the STM32. Handles encoder reading, PID speed control for all 4 wheels, and UART communication with the Raspberry Pi.

## Communication protocol

Receives: `TARGET,rpm1,rpm2,rpm3,rpm4\n`
Sends back: `ODOM,x,y,theta,rpm1,rpm2,rpm3,rpm4\n`

Watchdog timeout: if no TARGET command received for ~200ms, motors stop automatically.
