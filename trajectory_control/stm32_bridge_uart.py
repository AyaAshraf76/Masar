"""
UART version of the STM32 bridge.
Connects via GPIO UART pins (Pi RX <- STM32 TX on A9,
Pi TX -> STM32 RX on A10, shared GND).

Key differences from USB CDC version:
- Port is /dev/ttyAMA0 (or /dev/ttyS0 -- confirmed after checking Pi model)
- No 2-second USB enumeration delay needed
- UART on Pi 4 and earlier needs Bluetooth disabled to free ttyAMA0
- No hardware flow control (rtscts=False, dsrdtr=False)
"""

import serial
import threading
import time


class STM32Bridge:
    def __init__(self, port="/dev/ttyAMA0", baudrate=115200):
        self.ser = serial.Serial(
            port     = port,
            baudrate = baudrate,
            timeout  = 1.0,
            rtscts   = False,   # no hardware flow control on GPIO UART
            dsrdtr   = False,
        )
        time.sleep(0.5)  # short settle time for UART (no USB enumeration needed)

        self._lock = threading.Lock()
        self._latest_pose = (0.0, 0.0, 0.0)
        self._latest_rpms = (0.0, 0.0, 0.0, 0.0)
        self._latest_timestamp = None

        self._running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def _read_loop(self):
        while self._running:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception as e:
                print(f"[STM32Bridge] Read error: {e}")
                continue

            if not line:
                continue

            if line.startswith("ODOM,"):
                self._parse_odom_line(line)
            else:
                print(f"[STM32] {line}")

    def _parse_odom_line(self, line):
        parts = line.split(",")
        if len(parts) != 8:
            return
        try:
            x     = float(parts[1])
            y     = float(parts[2])
            theta = float(parts[3])
            r1    = float(parts[4])
            r2    = float(parts[5])
            r3    = float(parts[6])
            r4    = float(parts[7])
        except ValueError:
            return

        with self._lock:
            self._latest_pose      = (x, y, theta)
            self._latest_rpms      = (r1, r2, r3, r4)
            self._latest_timestamp = time.time()

    def get_latest_pose(self):
        with self._lock:
            x, y, theta = self._latest_pose
            age = time.time() - self._latest_timestamp \
                  if self._latest_timestamp else float("inf")
        return x, y, theta, age

    def get_latest_rpms(self):
        with self._lock:
            return self._latest_rpms

    def send_targets(self, rpm1, rpm2, rpm3, rpm4):
        line = f"TARGET,{rpm1:.2f},{rpm2:.2f},{rpm3:.2f},{rpm4:.2f}\n"
        self.ser.write(line.encode("utf-8"))

    def send_calibration_command(self, command):
        self.ser.write((command + "\n").encode("utf-8"))

    def close(self):
        self._running = False
        self._reader_thread.join(timeout=1.0)
        self.ser.close()
