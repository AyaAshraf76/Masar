#!/usr/bin/env python3
import cv2
import socket
import struct
import numpy as np

PI_IP = '192.168.1.5'
PORT  = 9998

def main():
    print(f'Connecting to {PI_IP}:{PORT}...')
    client = socket.socket(
        socket.AF_INET, socket.SOCK_STREAM)
    client.connect((PI_IP, PORT))
    print('Connected. Press Q to quit')

    data         = b''
    payload_size = struct.calcsize('>L')

    # Normal resizable window — 800x600
    cv2.namedWindow(
        'Lane Detection',
        cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Lane Detection', 800, 600)

    while True:
        while len(data) < payload_size:
            packet = client.recv(4096)
            if not packet:
                return
            data += packet

        packed_size = data[:payload_size]
        data        = data[payload_size:]
        msg_size    = struct.unpack(
            '>L', packed_size)[0]

        while len(data) < msg_size:
            packet = client.recv(4096)
            if not packet:
                return
            data += packet

        frame_data = data[:msg_size]
        data       = data[msg_size:]

        nparr = np.frombuffer(
            frame_data, np.uint8)
        frame = cv2.imdecode(
            nparr, cv2.IMREAD_COLOR)

        if frame is not None:
            cv2.imshow('Lane Detection', frame)

        # Press Q to quit
        # Press F for fullscreen
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('f'):
            cv2.setWindowProperty(
                'Lane Detection',
                cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN)

    client.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
