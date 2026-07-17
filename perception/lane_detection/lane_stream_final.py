#!/usr/bin/env python3
"""
Lane Detection - Combined Final Script
Runs on Raspberry Pi 5

Does TWO things simultaneously from ONE inference:
  1. Sends JSON data  ^f^r port 9999  ^f^r ROS2 bridge on laptop
  2. Sends visual stream  ^f^r port 9998  ^f^r viewer on laptop

error_cm is a pure post-calculation added on top of the
existing lane-aware pixel error. Lane detection logic,
ROI, voting, and smoothing are UNCHANGED.

Both lanes are 35cm wide in reality (confirmed).
"""

import cv2
import numpy as np
import onnxruntime as ort
import socket
import struct
import time
import json
import threading
from picamera2 import Picamera2
from libcamera import controls
from collections import deque, Counter


#  ^t^` ^t^` Configuration  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t>
MODEL_PATH    = '/home/pi/lane_model.onnx'
JSON_PORT     = 9999   # ROS2 bridge connects here
VIDEO_PORT    = 9998   # viewer on laptop connects here
INPUT_W       = 320
INPUT_H       = 240
ALPHA         = 0.3
SOLID_ALPHA   = 0.4
SEP_THRESHOLD = 110.0
LANE_HISTORY  = 10
MIN_VOTES     = 6

#  ^t^` ^t^` Real-world lane width (confirmed by measurement)  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^`
LANE_WIDTH_CM = 36.7

#  ^t^` ^t^` CM calibration (PLACEHOLDERS until you run the test)  ^t^` ^t^` ^t^`
# Run Test A (outer) and Test B (inner) below, then fill in
# real numbers here. Until then error_cm == error in px
# (offset 0, scale 1)  ^`^t completely harmless default.
ERROR_OFFSET_OUTER_PX = -8.5
ERROR_OFFSET_INNER_PX = -7.5
PIXELS_PER_CM_OUTER    = 2.19
PIXELS_PER_CM_INNER    = 2.51




#  ^t^` ^t^` Colors  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^`>
COLOR_WHITE = (255, 255, 255)
COLOR_GREEN = (0, 255, 0)
COLOR_CYAN  = (0, 255, 255)
COLOR_BLUE  = (255, 0, 0)
COLOR_BLACK = (0, 0, 0)

mask_colors = np.array([
    [50,  50,  50],
    [0,   255, 0 ],
    [0,   255, 255],
], dtype=np.uint8)


#  ^t^` ^t^` Load model  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` >
print('Loading model...')
sess_options = ort.SessionOptions()
sess_options.intra_op_num_threads = 4
sess_options.inter_op_num_threads = 4


sess = ort.InferenceSession(
    MODEL_PATH,
    sess_options=sess_options,
    providers=['CPUExecutionProvider'])

input_name = sess.get_inputs()[0].name
print(f'Model loaded: {input_name}')


#  ^t^` ^t^` State  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` >
prev_center    = None
prev_solid_pos = None
prev_sep_pos   = None
lane_history   = deque(maxlen=LANE_HISTORY)
width_history  = deque(maxlen=50)
current_lane   = 'unknown'

#  ^t^` ^t^` Calibration test buffer (terminal-only, no effect on
#    streaming)  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^>
calib_buffer = deque(maxlen=15)


#  ^t^` ^t^` Preprocess  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` >
def preprocess(frame_bgr):
    frame_rgb = cv2.cvtColor(
        frame_bgr, cv2.COLOR_BGR2RGB)
    inp = cv2.resize(
        frame_rgb, (INPUT_W, INPUT_H),
        interpolation=cv2.INTER_LINEAR)
    inp = inp.astype(np.float32) / 255.0
    inp = inp.transpose(2, 0, 1)[np.newaxis]
    return inp

#  ^t^` ^t^` Geometry  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t>
def extract_geometry(mask):
    global prev_center, prev_solid_pos
    global prev_sep_pos, current_lane

    h, w   = mask.shape
    img_cx = w / 2.0

    roi_top = int(h * 0.20)
    roi     = mask[roi_top:, :]

    solid_cols = np.where(roi == 1)[1]
    sep_cols   = np.where(roi == 2)[1]

    solid_count = len(solid_cols)
    sep_count   = len(sep_cols)

    # Track actual detection this frame
    actually_has_sep   = sep_count   > 8
    actually_has_solid = solid_count > 8

    sep_x   = float(np.mean(sep_cols)) \
               if actually_has_sep   else None
    solid_x = float(np.mean(solid_cols)) \
               if actually_has_solid else None

    # Smooth solid
    if solid_x is not None:
        if prev_solid_pos is not None:
            solid_x = SOLID_ALPHA * solid_x + \
                      (1-SOLID_ALPHA) * prev_solid_pos
        prev_solid_pos = solid_x
    elif prev_solid_pos is not None:
        solid_x        = prev_solid_pos
        prev_solid_pos = prev_solid_pos * 0.95

    # Smooth separator
    if sep_x is not None:
        if prev_sep_pos is not None:
            sep_x       = SOLID_ALPHA * sep_x + \
                          (1-SOLID_ALPHA) * prev_sep_pos
        prev_sep_pos = sep_x
    elif prev_sep_pos is not None:
        sep_x        = prev_sep_pos
        prev_sep_pos = prev_sep_pos * 0.95

    # Lane width
    if solid_x is not None and sep_x is not None:
        lw = abs(solid_x - sep_x)
        if 20 < lw < w * 0.9:
            width_history.append(lw)

    lw = float(np.median(width_history)) \
         if len(width_history) >= 5 else w * 0.25

    # Lane detection using separator position
    if actually_has_sep and sep_x is not None:
        if sep_x < SEP_THRESHOLD:
            lane_history.append('outer')
        else:
            lane_history.append('inner')

    counts = Counter(lane_history)
    if counts and len(lane_history) >= 3:
        best, n = counts.most_common(1)[0]
        if n >= MIN_VOTES:
            current_lane = best

    # Lane center using ACTUAL detection only
    # (this is already lane-aware in every branch  ^`^t unchanged)
    if actually_has_solid and actually_has_sep:
        center = (solid_x + sep_x) / 2
        conf   = 1.0
    elif actually_has_solid:
        if current_lane == 'outer':
            center = solid_x - lw / 2
        elif current_lane == 'inner':
            center = solid_x - lw / 2
        else:
            center = solid_x
        conf = 0.5
    elif actually_has_sep:
        if current_lane == 'outer':
            center = sep_x + lw / 2
        elif current_lane == 'inner':
            center = sep_x - lw / 2
        else:
            center = sep_x
        conf = 0.4
    elif prev_center is not None:
        center = prev_center
        conf   = 0.2
    else:
        center = img_cx
        conf   = 0.0

    if prev_center is None:
        prev_center = center
    center      = ALPHA * center + \
                  (1-ALPHA) * prev_center
    prev_center = center
    error       = center - img_cx

    #  ^t^` ^t^` CM conversion  ^`^t lane-aware, pure post-calculation  ^t^` ^t^`
    # Uses different offset/scale depending on current_lane.
    # Does not feed back into center/error/lane_history above.
    if current_lane == 'outer':
        offset_px = ERROR_OFFSET_OUTER_PX
        px_per_cm = PIXELS_PER_CM_OUTER
    elif current_lane == 'inner':
        offset_px = ERROR_OFFSET_INNER_PX
        px_per_cm = PIXELS_PER_CM_INNER
    else:
        # unknown lane  ^`^t fall back to outer calibration,
        # flagged via confidence already being low anyway
        offset_px = ERROR_OFFSET_OUTER_PX
        px_per_cm = PIXELS_PER_CM_OUTER

    error_calibrated = error - offset_px
    error_cm = error_calibrated / px_per_cm

    # Curvature
    curvature = 0.0
    if actually_has_sep and sep_count > 50:
        sep_rows = np.where(roi == 2)[0]
        sep_c    = np.where(roi == 2)[1]
        if len(sep_rows) > 10:
            coeffs    = np.polyfit(
                sep_rows, sep_c, 1)
            curvature = float(coeffs[0])

    # Track readings for the terminal calibration helper
    calib_buffer.append(error)

    return {
        'center':            float(center),
        'error':             float(error),
        'error_cm':          round(float(error_cm), 2),
        'lane':              current_lane,
        'width':             float(lw),
        'curvature':         float(curvature),
        'confidence':        float(conf),
        'solid_count':       int(solid_count),
        'sep_count':         int(sep_count),
        'sep_x':             float(sep_x)
                             if sep_x is not None
                             else None,
        'solid_x':           float(solid_x)
                             if solid_x is not None
                             else None,
        'actually_has_sep':  actually_has_sep,
        'actually_has_solid': actually_has_solid,
        'mask':              mask.tolist(),
    }


#  ^t^` ^t^` Annotate frame for visual stream  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^`
def annotate_frame(frame_bgr, mask, geo, fps):
    h, w = frame_bgr.shape[:2]

    # Color overlay
    mask_full = cv2.resize(
        mask.astype(np.uint8), (w, h),
        interpolation=cv2.INTER_NEAREST)
    color_mask = mask_colors[mask_full]
    color_mask_bgr = cv2.cvtColor(
        color_mask, cv2.COLOR_RGB2BGR)
    annotated = cv2.addWeighted(
        frame_bgr, 0.55,
      color_mask_bgr, 0.45, 0)

    # Lane center line (blue)
    if geo['center'] is not None:
        cx = int(geo['center'] * w / INPUT_W)
        cv2.line(annotated,
                 (cx, h//2), (cx, h),
                 COLOR_BLUE, 2)
        cv2.circle(annotated,
                   (cx, h-20),
                   8, COLOR_BLUE, -1)

    # Image center line (grey)
    icx = w // 2
    cv2.line(annotated,
             (icx, h//2), (icx, h),
             (128, 128, 128), 1)

   # Labels on detected lines
    if geo['solid_x'] is not None:
        sx = int(geo['solid_x'] * w / INPUT_W)
        cv2.putText(annotated, 'solid_boundary',
                    (max(0, sx-60), h//2+30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, COLOR_GREEN, 2)

    if geo['sep_x'] is not None:
        spx = int(geo['sep_x'] * w / INPUT_W)
        cv2.putText(annotated, 'lane_separator',
                    (max(0, spx-60), h//2+100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, COLOR_CYAN, 2)

    # Lane label between lines
    if geo['solid_x'] is not None and \
       geo['sep_x'] is not None:
        mid = int(
            ((geo['solid_x']+geo['sep_x'])/2)
            * w / INPUT_W)
        cv2.putText(
            annotated,
            geo['lane'].upper() + ' LANE',
            (max(0, mid-50), h//2-10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            COLOR_GREEN
            if geo['lane'] == 'outer'

            else (0, 200, 255),
            2)

    # Info panel background
    panel = annotated.copy()
    cv2.rectangle(panel, (0, 0), (340, 248),
                  COLOR_BLACK, -1)
    annotated = cv2.addWeighted(
        panel, 0.6, annotated, 0.4, 0)

    # Lane color
    lane_color = \
        COLOR_GREEN if geo['lane'] == 'outer' \
        else (0, 200, 255) \
        if geo['lane'] == 'inner' \
        else (128, 128, 128)

    # All info text on screen
    lines = [
        (f"FPS:    {fps:.1f}",
         COLOR_WHITE),
        (f"Lane:   {geo['lane'].upper()}",
         lane_color),
        (f"Error:  {geo['error']:+.1f} px",
         COLOR_WHITE),
        (f"Err_cm: {geo['error_cm']:+.1f} cm",
         COLOR_CYAN),
        (f"Conf:   {geo['confidence']:.2f}",
         COLOR_WHITE),
        (f"Solid:  {geo['solid_count']} ROI",
         COLOR_CYAN),
        (f"Width:  {geo['width']:.0f} px",
         COLOR_WHITE),
        (f"Curv:   {geo['curvature']:.3f}",
         COLOR_WHITE),
        (f"Sep_x:  "
         f"{geo['sep_x']:.0f}"
         if geo['sep_x'] is not None
         else "Sep_x:  None",
         COLOR_CYAN),
    ]

    for i, (txt, col) in enumerate(lines):
        cv2.putText(annotated, txt,
                    (10, 26 + i*23),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.56, col, 2)

    # Legend bottom right
    lx, ly = w-225, h-72
    cv2.rectangle(annotated,
                  (lx, ly), (w-5, h-5),
                  COLOR_BLACK, -1)
    cv2.putText(annotated,
                'GREEN=solid_boundary',
                (lx+5, ly+20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42, COLOR_GREEN, 1)
    cv2.putText(annotated,
                'CYAN =lane_separator',
                (lx+5, ly+40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42, COLOR_CYAN, 1)
    cv2.putText(annotated,

               'BLUE =lane center',
                (lx+5, ly+58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42, COLOR_BLUE, 1)

    return annotated


#  ^t^` ^t^` Send data safely  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^`>
def send_data(conn, lock, data, is_bytes=False):
    if conn is None:
        return False
    try:
        if is_bytes:
            payload = data
        else:
            payload = json.dumps(data)\
                .encode('utf-8')
        size = struct.pack('>L', len(payload))
        with lock:
            conn.sendall(size + payload)
        return True
    except (BrokenPipeError,
            ConnectionResetError,
            OSError):
        return False

#  ^t^` ^t^` Accept connections in background  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^`
def accept_json_connection(server):
    global json_conn
    while True:
        try:
            print('JSON: waiting for '
                  'ROS2 bridge on '
                  f'port {JSON_PORT}...')
            conn, addr = server.accept()
            print(f'JSON: bridge connected {addr}')
            json_conn = conn
        except Exception as e:
            print(f'JSON accept error: {e}')
            break


def accept_video_connection(server):
    global video_conn
    while True:
        try:
            print('VIDEO: waiting for '
                  'viewer on '
                  f'port {VIDEO_PORT}...')
            conn, addr = server.accept()
            print(f'VIDEO: viewer connected {addr}')
            video_conn = conn
        except Exception as e:
            print(f'VIDEO accept error: {e}')
            break

#  ^t^` ^t^` Shared connection state  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t>
json_conn  = None
video_conn = None
json_lock  = threading.Lock()
video_lock = threading.Lock()


#  ^t^` ^t^` Main  ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^t^` ^>
def main():
    global json_conn, video_conn

    # Start JSON server (for ROS2 bridge)
    json_server = socket.socket(
        socket.AF_INET, socket.SOCK_STREAM)
    json_server.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR, 1)
    json_server.bind(('0.0.0.0', JSON_PORT))
    json_server.listen(1)

    # Start video server (for laptop viewer)
    video_server = socket.socket(
        socket.AF_INET, socket.SOCK_STREAM)
    video_server.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR, 1)
    video_server.bind(('0.0.0.0', VIDEO_PORT))
    video_server.listen(1)

    # Accept connections in background threads
    t1 = threading.Thread(
        target=accept_json_connection,
        args=(json_server,), daemon=True)
    t2 = threading.Thread(
        target=accept_video_connection,
        args=(video_server,), daemon=True)
    t1.start()
    t2.start()

    # Setup camera
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (640, 480),
              "format": "XBGR8888"},
        buffer_count=4)
    picam2.configure(config)
    picam2.set_controls({
        "AeEnable":  True,
        "AwbEnable": True,
        "AwbMode":   controls.AwbModeEnum.Auto,
    })
    picam2.start()
    time.sleep(3)
    print('Camera started')
    print(f'JSON port:  {JSON_PORT} '
          f'(connect ROS2 bridge here)')
    print(f'VIDEO port: {VIDEO_PORT} '
          f'(connect laptop viewer here)')
    print(f'Lane width (real): {LANE_WIDTH_CM} cm')
    print('You can connect viewers '
          'in any order')
    print()
    print('=== CALIBRATION HELPER ===')

    print('Place car centered in a lane, wait a few')
    print('seconds, then this terminal will print a')
    print('running MEDIAN of the last 15 error_px')
    print('readings every 30 frames  ^`^t use that median')
    print('as your offset for that lane.')
    print('==========================')
    print()

    frame_count = 0
    start_time  = time.time()

    try:
        while True:
            # 1. Capture frame
            frame_rgba = picam2.capture_array()
            frame_bgr  = cv2.cvtColor(
                frame_rgba, cv2.COLOR_RGBA2BGR)

            # 2. Preprocess
            inp = preprocess(frame_bgr)

            # 3. Inference (runs ONCE per frame)
            out  = sess.run(
                None, {input_name: inp})[0]
            mask = np.argmax(

                out[0], axis=0).astype(np.uint8)

            # 4. Geometry (runs ONCE per frame)
            geo = extract_geometry(mask)

            # 5. FPS
            frame_count += 1
            elapsed = time.time() - start_time
            fps     = frame_count / elapsed
            geo['fps'] = round(fps, 1)

            # 6. Send JSON to ROS2 bridge
            if json_conn is not None:
                geo_send = {
                    k: v for k, v in geo.items()
                    if k != 'mask'
                }
                geo_send['mask'] = \
                    mask.tolist()
                ok = send_data(
                    json_conn,
                    json_lock,
                    geo_send)
                if not ok:
                    print('JSON: bridge '
                          'disconnected')
                    json_conn = None

            # 7. Send video to viewer
            if video_conn is not None:
                display = annotate_frame(
                    frame_bgr, mask, geo, fps)
                encode_param = [
                    cv2.IMWRITE_JPEG_QUALITY, 80]

                ok, buffer = cv2.imencode(
                    '.jpg', display,
                    encode_param)
                if ok:
                    ok2 = send_data(
                        video_conn,
                        video_lock,
                        buffer.tobytes(),
                        is_bytes=True)
                    if not ok2:
                        print('VIDEO: viewer '
                              'disconnected')
                        video_conn = None

            # 8. Print to Pi terminal  ^`^t includes calibration
            #    median helper, runs independently of streaming
            if frame_count % 30 == 0:
                median_val = float(
                    np.median(calib_buffer)) \
                    if len(calib_buffer) > 0 else 0.0
                print(
                    f'FPS:{fps:.1f} '
                    f'Lane:{geo["lane"]:7s} '
                    f'Err:{geo["error"]:+.1f}px '
                    f'({geo["error_cm"]:+.1f}cm) '
                    f'Conf:{geo["confidence"]:.2f} '
                    f'Sep:{geo["sep_count"]} '
                    f'Solid:{geo["solid_count"]} '
                    f'| CALIB_MEDIAN:{median_val:+.1f}px')

    except KeyboardInterrupt:
        print('Stopped')

    finally:
        picam2.stop()
        picam2.close()
        if json_conn:
            json_conn.close()
        if video_conn:
            video_conn.close()
        json_server.close()
        video_server.close()


if __name__ == '__main__':
    main()


