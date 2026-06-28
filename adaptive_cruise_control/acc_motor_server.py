#!/usr/bin/env python3
"""
ACC motor server — runs on the Raspberry Pi.
Receives speed commands from the laptop via TCP and forwards
them as RPM targets to the STM32 via UART.

The laptop's ROS2 ACC node decides the speed based on LiDAR
zones, and this server translates that into motor commands.
"""

import socket
import json
import time
import serial
import threading

UART_PORT = "/dev/serial0"
BAUD = 115200

TCP_HOST = "0.0.0.0"
TCP_PORT = 9998

MAX_RPM = 35.0       # green zone speed
YELLOW_RPM = 18.0    # yellow zone (slow down)

current_rpm = 0.0
lock = threading.Lock()


def speed_to_rpm(speed):
    if speed <= 0.05:
        return 0.0
    elif speed < 0.7:
        return YELLOW_RPM
    else:
        return MAX_RPM


def uart_loop(ser):
    global current_rpm

    while True:
        with lock:
            rpm = current_rpm

        cmd = f"TARGET,{rpm:.1f},{rpm:.1f},{rpm:.1f},{rpm:.1f}\n"
        ser.write(cmd.encode("utf-8"))
        ser.flush()

        while ser.in_waiting:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print("STM32:", line)

        time.sleep(0.05)  # 20 Hz


def tcp_loop():
    global current_rpm

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(1)

    print(f"ACC motor server waiting on port {TCP_PORT}")

    while True:
        conn, addr = server.accept()
        print("Laptop connected:", addr)

        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                msg = json.loads(data.decode("utf-8"))
                speed = float(msg.get("speed", 0.0))
                rpm = speed_to_rpm(speed)

                with lock:
                    current_rpm = rpm

                print(f"ACC speed={speed:.2f} -> rpm={rpm:.1f}")

        except Exception as e:
            print("TCP error:", e)

        finally:
            with lock:
                current_rpm = 0.0
            conn.close()
            print("Laptop disconnected -> STOP")


def main():
    ser = serial.Serial(UART_PORT, BAUD, timeout=0.01)
    print("UART opened:", UART_PORT)

    threading.Thread(target=uart_loop, args=(ser,), daemon=True).start()
    tcp_loop()


if __name__ == "__main__":
    main()
