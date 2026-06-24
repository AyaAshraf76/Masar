#!/usr/bin/env python3
"""
RPLIDAR A1M8 streamer - runs on Raspberry Pi 5
Reads LiDAR scans and sends them as JSON over TCP to the
laptop's lidar_bridge_node (which publishes ROS2 /scan).

Same architecture as lane_stream_modified.py:
  Pi = server, laptop connects in.
"""

import socket
import struct
import json
import time
import threading
from rplidar import RPLidar

# ── Configuration ─────────────────────────────────────────
LIDAR_PORT   = '/dev/ttyUSB0'
TCP_PORT     = 9997          # laptop lidar_bridge connects here
SCAN_MIN_QUALITY = 10        # ignore very weak returns

lidar = None
client_conn = None
conn_lock = threading.Lock()


def accept_connection(server):
    global client_conn
    while True:
        try:
            print(f'LIDAR: waiting for bridge on port {TCP_PORT}...')
            conn, addr = server.accept()
            print(f'LIDAR: bridge connected {addr}')
            client_conn = conn
        except Exception as e:
            print(f'LIDAR accept error: {e}')
            break


def send_scan(conn, scan_points):
    """Send one scan as length-prefixed JSON."""
    try:
        payload = json.dumps(scan_points).encode('utf-8')
        size = struct.pack('>L', len(payload))
        with conn_lock:
            conn.sendall(size + payload)
        return True
    except (BrokenPipeError, ConnectionResetError, OSError):
        return False


def main():
    global lidar, client_conn

    # Start TCP server
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', TCP_PORT))
    server.listen(1)

    threading.Thread(target=accept_connection,
                     args=(server,), daemon=True).start()

    # Start LiDAR
    lidar = RPLidar(LIDAR_PORT)
    print('LIDAR: starting motor...')
    time.sleep(2)
    info = lidar.get_info()
    print(f'LIDAR info: {info}')
    print(f'LIDAR health: {lidar.get_health()}')
    print(f'TCP port: {TCP_PORT} (connect laptop bridge here)')
    print()

    scan_count = 0
    try:
        for scan in lidar.iter_scans(max_buf_meas=5000):
            # scan = list of (quality, angle_deg, distance_mm)
            # Filter weak returns, convert to compact list
            points = [
                [round(angle, 2), round(dist, 1)]
                for (q, angle, dist) in scan
                if q >= SCAN_MIN_QUALITY and dist > 0
            ]

            scan_count += 1

            if client_conn is not None:
                if not send_scan(client_conn, points):
                    print('LIDAR: bridge disconnected')
                    client_conn = None

            if scan_count % 10 == 0:
                print(f'Scan {scan_count}: {len(points)} valid points')

    except KeyboardInterrupt:
        print('Stopping...')
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        server.close()


if __name__ == '__main__':
    main()
