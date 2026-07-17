"""
Integrated trajectory controller - Raspberry Pi side.

States:
  LANE_KEEPING  -- normal driving, uses camera error_cm to stay centered
  LANE_CHANGING -- executing a pure-pursuit waypoint maneuver to shift lanes
  STOPPED       -- motors off (e.g. RED zone from ACC, or manual stop)

Inputs:
  - Camera JSON on TCP port 9999 (lane_stream_modified.py)
    fields used: lane, error_cm, confidence
  - ACC JSON on TCP port 9996 (friend's bridge, not ready yet -- placeholder)
    fields used: zone, obstacle_lane
  - Keyboard: press 'c' + Enter to trigger a lane change manually for testing

Outputs:
  - UART to STM32 (TARGET,rpm1,rpm2,rpm3,rpm4)
  - Odometry read back from STM32 (ODOM,x,y,theta,rpm1,rpm2,rpm3,rpm4)

v9 changes (from the 2026-07-05 logs; pair with lane_stream_v3.py):
  * alpha scales linearly with confidence, floor raised to 0.20
    (v8's conf^2 stacked with the innovation clamp into a
    1-1.5 cm/frame crawl exactly in low-conf corners).
  * Innovation clamp gets a TREND RELEASE: 3 consecutive
    same-sign clamped innovations open the clamp x4, so real
    sustained drifts pass in ~3 frames while single-frame
    spikes still can't move the filter more than 10 cm.
  * Lane-flip reseed is gated on the flip frame's confidence:
    conf>=0.8 snaps, 0.5-0.8 blends 50/50, below 0.5 the state
    is kept (v8 snapped to a conf=0.40 garbage frame and held
    saturated wrong-direction steering for ~3 s in run 2).

v10 changes (from the v9 logs; estimation is now healthy, both
remaining problems are CONTROL):
  * OUTER SPEED 0.10 -> 0.08 m/s, same as inner. The outer run
    shows a weave with GROWING amplitude (~±3 -> ±6 -> ±10 ->
    ±13 cm over ~40 frames) that ends with a corner landing on
    top of the swing, omega pinned at 0.300, and the car
    carrying its lateral momentum across the separator at
    +36 cm. A growing oscillation = the loop is out of phase
    margin, and the ONLY variable that differs between the
    stable (inner) and unstable (outer) lane is speed: at
    ~5 fps camera rate, 0.10 m/s is 25% more cm of drift per
    frame against the same LK_MAX_OMEGA and the same delay.
    One knob, matched to the configuration that already works.
  * `cv=` (raw camera curvature) and `ff=` (feed-forward
    omega) added to the [KEEP] log line so K_CURV can be
    calibrated. The inner-lane "kisses the separator" behavior
    is the controller needing ~half a lane of error (15-18 cm
    ~= half-lane 18.4 cm) before it commands full steering in
    corners; curvature feed-forward supplies the corner's
    omega from vision BEFORE the error builds. It stays
    DISABLED (K_CURV = 0.0) in this version -- v10 only makes
    it observable. One variable at a time.

v11 changes (from the v10 outer logs, 2026-07-06; pair with
lane_stream_v4.py -- BOTH files must be updated together):
  * PERSISTENT-FLIP reseed. In v10 run 2 every lane flip
    arrived on conf=0.40 (separator-only) frames. v9's rule
    ("don't reseed below conf 0.5") then never reseeded, so
    after each flip the filter stayed in the OLD lane's
    calibration basis (per-lane offset + px/cm) -- a ~35cm
    phantom error the controller dutifully chased at full
    authority. New rule: conf>=0.8 flip snaps immediately
    (unchanged); any flip reported on 2 consecutive frames is
    treated as real and reseeds regardless of confidence. A
    single-frame classifier flicker still can't move the
    filter past the innovation clamp.
  * (camera side, lane_stream_v4) lane votes now come from
    RELATIVE geometry (solid vs separator) on both-lines
    frames only, instead of comparing sep_x to a fixed image
    column. The fixed-column vote is what produced run 2's
    limit cycle: saturated steering yaws the car, the
    separator sweeps across column 110, the classifier flips,
    the error basis jumps a lane width, the controller
    reverses, the separator sweeps back. Repeat forever.

  K_CURV verdict from the v10 cv= data: DO NOT ENABLE yet.
    The signal is unusable as-is: it sits at +2.5..+4 on
    straights (perspective slope of the separator, not
    zero-centered), reads exactly 0.000 in most corners
    (separator lost right when feed-forward is needed), and
    spikes to +20 or flips sign on single frames. Usable
    feed-forward needs a camera-side baseline-subtracted,
    conf-gated curvature first. Keep K_CURV = 0.0.

  K_CURV calibration procedure (do this on track, K_CURV=0):
    1. Run 2-3 laps in EACH lane with v10 and save the logs.
    2. In the logs, find corner stretches: sustained large
       |w| (0.15-0.30) with conf mostly 1.00.
    3. Check the SIGN: in those stretches note the sign of
       `cv=` vs the sign of `w=`. If they match, K_CURV is
       positive; if opposite, negative. (cv is the px/px
       slope of the separator fit in image space -- its sign
       convention is not guaranteed to match omega's, that is
       exactly what this step verifies. Note cv reads 0.000
       whenever the separator is not detected, so only use
       frames where cv is nonzero.)
    4. Estimate the magnitude: pick several corner frames and
       compute w/cv for each. A typical corner needs
       |w| ~ 0.20-0.30 and cv will be some slope like 0.3-1.0,
       so expect K_CURV somewhere around |w/cv|. Start with
       HALF that value -- feed-forward should carry most of
       the corner and leave the rest to P/I, overshooting it
       re-creates the weave with the opposite sign.
    5. Enable it, run one lap per lane, and confirm in the
       logs that `ff=` has the same sign as `w=` in corners
       and that corner error no longer settles at -15..-18 cm.
       If the car now cuts INTO corners, halve K_CURV.

Motor mapping, kinematics, pure pursuit, integral: unchanged.
"""

import time
import math
import socket
import struct
import json
import threading
import sys
import select

from stm32_bridge_uart import STM32Bridge
from kinematics import DiffDriveKinematics
from pure_pursuit import PurePursuit

# ── Robot physical parameters (calibrated) ────────────────
WHEEL_RADIUS = 0.03367
TRACK_WIDTH  = 0.74761

# ── Track parameters ──────────────────────────────────────
LANE_WIDTH   = 0.45   # meters, physical center-to-center

# ── Speed settings ────────────────────────────────────────
TARGET_SPEED       = 0.08  # m/s base cruising speed (outer lane).
                           # v10: was 0.10. The v9 outer logs show a
                           # weave of growing amplitude ending in a
                           # separator crossing at +36cm and a boundary
                           # drift after the flip -- classic loss of
                           # phase margin. At ~5 fps, 0.10 m/s is 25%
                           # more lateral drift per camera frame against
                           # the same omega clamp and delay. The inner
                           # lane at 0.08 tracked cleanly; match it.
TARGET_SPEED_INNER = 0.08  # m/s in the inner lane. Its corner radius
                           # is smaller and the logs show omega pinned
                           # at the clamp for 10+ frames there: the
                           # car physically could not turn tighter.
                           # radius = v/omega, so 20% less speed buys
                           # a 20% tighter achievable radius for free.
LANE_CHANGE_SPEED  = 0.10  # m/s during the pure-pursuit lane change.
                           # v10: split out from TARGET_SPEED so lowering
                           # the outer lane-keeping speed does not
                           # silently change the maneuver that already
                           # works. (v9 reused TARGET_SPEED here.)
MAX_OMEGA        = 0.6    # rad/s clamp during LANE CHANGE (pure pursuit)
LOOKAHEAD        = 0.45   # meters, pure pursuit lookahead during lane change

# ── Lane keeping (PD) tuning ──────────────────────────────
# The drivetrain can only realize a limited omega before the RPM
# clamps saturate. With base speed 0.13 m/s (~37 RPM) and wheels
# limited to [0, 60] RPM, the max *achievable* omega is roughly:
#   d_v = (60-37)/60 * 2*pi*R  ≈ 0.081 m/s per wheel
#   omega_max ≈ 2*d_v / TRACK_WIDTH ≈ 0.21 rad/s
# Commanding more than this just pins one wheel and destroys
# forward speed -> overshoot -> oscillation. So we clamp there.
LK_KP          = 1.1    # omega per meter of error   (err_cm * KP / 100)
LK_KI          = 0.30   # integral gain (err_cm*s * KI / 100). Supplies the
                        # sustained omega a corner needs so the error can
                        # settle near zero instead of riding the separator.
LK_I_LEAK_TAU  = 8.0    # seconds. The integral decays toward zero with
                        # this time constant, so it follows the current
                        # curvature and self-clears after each corner.
LK_I_MAX       = 0.15   # rad/s cap on the integral term's contribution
LK_KD          = 0.60   # damping on error rate (cm/s * KD / 100).
                        # Raised to suppress the slow left-right weave
                        # on straights: damping directly resists the
                        # swinging motion.
LK_MAX_OMEGA   = 0.30   # rad/s. Above ~0.2 the wheel clamps kick in and the
                        # saturation-shift below automatically SLOWS the car
                        # while turning hard -- exactly what you want in corners.
LK_OMEGA_SLEW  = 1.5    # rad/s per second, max rate of change of omega
CORNER_SLOWDOWN = 0.45  # fraction of forward speed removed at full
                        # steering demand. Slower in corners = tighter
                        # radius for the same omega AND more camera
                        # frames per meter of corner.
LK_ERR_ALPHA   = 0.6    # low-pass on error (1.0 = no filtering)
LK_DERIV_MAX   = 60.0   # cm/s clamp on error derivative (kills 1-frame spikes)
CONF_THRESHOLD = 0.25
DEAD_ZONE_CM   = 2.0
MAX_REAL_ERROR = 35.0   # cm. Errors beyond this are CLAMPED (not ignored):
                        # a huge error means "steer hard", never "do nothing".
MAX_ERR_STEP   = 10.0   # cm. Innovation clamp: at 0.10 m/s the TRUE
                        # lateral error cannot change more than ~8-10cm
                        # between camera frames. Bigger jumps in the log
                        # (17cm, even 60cm around lane flips) are
                        # MEASUREMENT glitches, so limit how far a single
                        # frame can pull the filter. Unlike LK_DERIV_MAX
                        # (which only protects the D term) this protects
                        # the P and I terms too.
TREND_STEP_N   = 3      # v9: consecutive same-sign clamped innovations
                        # that count as a REAL trend instead of a glitch.
                        # v8's clamp treated a sustained 8cm/frame drift
                        # exactly like a one-frame spike, so the filter
                        # marched at ~1.5cm/frame while the car left the
                        # lane (run 2). A spike lasts 1 frame; a real
                        # geometry change pushes the same direction for
                        # many frames -- that asymmetry is the detector.
TREND_STEP_GAIN = 4.0   # clamp widens by this factor while a trend is
                        # active: a genuine change passes through in ~3
                        # frames, a lone spike still moves filt <= 10cm.
CONF_FULL_TRUST = 0.8   # below this confidence: don't accumulate the
                        # integral, and trust the measurement less.
                        # v9: alpha scales LINEARLY with conf (v8 used
                        # conf^2, which STACKED with the innovation clamp
                        # into a 1.0-1.5 cm/frame crawl -- and corners are
                        # exactly where conf drops AND fast response is
                        # needed, so v8 was slowest where it mattered most).
LK_ALPHA_MIN   = 0.20   # floor on the confidence-scaled alpha. Raised
                        # from 0.10: even a conf=0.4 stream must move the
                        # filter meaningfully, the clamp + trend logic
                        # (not near-zero alpha) is what rejects glitches.
K_CURV         = 0.0    # curvature feed-forward gain (rad/s per unit of
                        # camera 'curvature'). STILL DISABLED in v10:
                        # this version only makes curvature observable
                        # (cv= and ff= in the [KEEP] line). Follow the
                        # calibration procedure in the header docstring
                        # to pick the sign and magnitude, THEN enable.
                        # Note: 'curvature' from lane_stream_v3 is the
                        # px/px slope of a linear fit to the separator
                        # pixels, and is 0.0 whenever the separator is
                        # not detected (sep_count <= 50).
LOST_HOLD_SEC  = 1.0    # on low confidence, HOLD last omega this long
                        # before starting to decay toward straight

# ── Camera TCP client ──────────────────────────────────────
CAM_HOST = 'localhost'   # camera runs on same Pi
CAM_PORT = 9999

# ── ACC TCP client (placeholder until friend's bridge is ready) ──
ACC_HOST = 'localhost'
ACC_PORT = 9996

# ── UART ──────────────────────────────────────────────────
SERIAL_PORT     = '/dev/ttyAMA0'
CONTROL_RATE_HZ = 20
CONTROL_DT      = 1.0 / CONTROL_RATE_HZ
GOAL_TOLERANCE  = 0.10
MAX_POSE_AGE    = 0.3

# ── Lane change waypoints ─────────────────────────────────
# These shift the car by LANE_WIDTH in the y direction.
# Generated fresh each time a lane change is triggered,
# offset by the car's current odometry pose so the path
# starts exactly where the car is right now.
def make_lane_change_waypoints(start_x, start_y, start_theta, direction):
    """
    direction: +1 = shift toward positive y (left in our convention)
               -1 = shift toward negative y (right)
    Returns waypoints in global frame.
    """
    dy = direction * LANE_WIDTH
    # Local path shape (relative to start pose)
    local = [
        (0.0,  0.0),
        (0.2,  0.0),
        (0.4,  dy * 0.20),
        (0.6,  dy * 0.50),
        (0.8,  dy * 0.80),
        (1.0,  dy),
        (1.3,  dy),
        (1.6,  dy),
        (2.0,  dy),
    ]
    # Rotate local waypoints by start_theta and translate to global frame
    cos_t = math.cos(start_theta)
    sin_t = math.sin(start_theta)
    global_wps = []
    for lx, ly in local:
        gx = start_x + lx * cos_t - ly * sin_t
        gy = start_y + lx * sin_t + ly * cos_t
        global_wps.append((gx, gy))
    return global_wps


# ── Camera reader (TCP client) ─────────────────────────────
class CameraReader:
    """Connects to lane_stream_modified.py JSON stream on port 9999."""
    def __init__(self, host, port):
        self._lock   = threading.Lock()
        self._data   = {'lane': 'unknown', 'error_cm': 0.0,
                        'confidence': 0.0, 'curvature': 0.0,
                        'seq': 0, 'stamp': 0.0}
        self._host   = host
        self._port   = port
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _recv_exact(self, sock, n):
        buf = b''
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def _loop(self):
        seq = 0
        while True:
            try:
                print(f'[Camera] connecting to {self._host}:{self._port}...')
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self._host, self._port))
                print('[Camera] connected')
                while True:
                    raw = self._recv_exact(sock, 4)
                    if not raw:
                        break
                    size = struct.unpack('>L', raw)[0]
                    payload = self._recv_exact(sock, size)
                    if not payload:
                        break
                    data = json.loads(payload.decode('utf-8'))
                    seq += 1
                    with self._lock:
                        # Only keep the fields we need -- drop mask to save memory
                        self._data = {
                            'lane':       data.get('lane', 'unknown'),
                            'error_cm':   data.get('error_cm', 0.0),
                            'confidence': data.get('confidence', 0.0),
                            'curvature':  data.get('curvature', 0.0),
                            'seq':        seq,
                            'stamp':      time.time(),
                        }
            except Exception as e:
                print(f'[Camera] disconnected ({e}), retrying in 2s')
                time.sleep(2)

    def get(self):
        with self._lock:
            return dict(self._data)


# ── ACC reader (placeholder) ───────────────────────────────
class ACCReader:
    """
    Placeholder for friend's ACC bridge.
    Returns GREEN / no obstacle until the real bridge connects.
    When friend's TCP bridge is ready, it should send JSON:
      {"zone": "GREEN"|"YELLOW"|"RED", "obstacle_lane": "none"|"outer"|"inner"}
    """
    def __init__(self, host, port):
        self._lock   = threading.Lock()
        self._data   = {'zone': 'GREEN', 'obstacle_lane': 'none'}
        self._host   = host
        self._port   = port
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _recv_exact(self, sock, n):
        buf = b''
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def _loop(self):
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3)
                sock.connect((self._host, self._port))
                print('[ACC] bridge connected')
                while True:
                    raw = self._recv_exact(sock, 4)
                    if not raw:
                        break
                    size = struct.unpack('>L', raw)[0]
                    payload = self._recv_exact(sock, size)
                    if not payload:
                        break
                    data = json.loads(payload.decode('utf-8'))
                    with self._lock:
                        self._data = {
                            'zone':          data.get('zone', 'GREEN'),
                            'obstacle_lane': data.get('obstacle_lane', 'none'),
                        }
            except Exception:
                # ACC not connected yet -- stay at defaults silently
                time.sleep(5)

    def get(self):
        with self._lock:
            return dict(self._data)


# ── Keyboard input (non-blocking) ─────────────────────────
def key_pressed():
    """Returns the line typed if Enter was pressed, else None."""
    r, _, _ = select.select([sys.stdin], [], [], 0)
    if r:
        return sys.stdin.readline().strip().lower()
    return None


# ── Main controller ────────────────────────────────────────
def main():
    print('Starting trajectory controller...')

    bridge  = STM32Bridge(port=SERIAL_PORT, baudrate=115200)
    kin     = DiffDriveKinematics(wheel_radius=WHEEL_RADIUS, track_width=TRACK_WIDTH)
    camera  = CameraReader(CAM_HOST, CAM_PORT)
    acc     = ACCReader(ACC_HOST, ACC_PORT)

    print('Resetting odometry...')
    bridge.send_calibration_command('r')
    time.sleep(0.3)

    # State machine
    state        = 'LANE_KEEPING'   # LANE_KEEPING | LANE_CHANGING | STOPPED
    current_lane = 'unknown'        # updated from camera each loop
    waypoints    = []
    last_index   = 0

    # Lane-keeping controller state
    last_cam_seq    = -1     # detect NEW camera frames (loop runs faster than camera)
    filt_err        = 0.0    # low-passed error (cm)
    err_step_sign   = 0      # v9: sign of the current run of clamped
    err_step_streak = 0      # innovations, and its length (trend detector)
    lane_flip_streak = 0     # v11: consecutive frames reporting a lane
                             # DIFFERENT from prev_rep_lane. 2 in a row
                             # = real flip -> reseed at any confidence.
    err_deriv       = 0.0    # d(err)/dt in cm/s, computed per camera frame
    err_integral    = 0.0    # integral of error (cm*s), for corner holding
    last_frame_time = None
    last_good_time  = None   # last time we had a confident measurement
    omega_des       = 0.0    # desired omega from the PD law
    omega_cmd       = 0.0    # slew-limited omega actually sent
    omega_ff        = 0.0    # v10: curvature feed-forward term, kept as
                             # state so the [KEEP] line can always log it
                             # (it is only recomputed on trusted frames)
    prev_rep_lane   = None   # last lane REPORTED by camera. On a lane
                             # flip the error basis shifts (per-lane
                             # calibration + fallback geometry), so we
                             # re-seed the filter instead of letting the
                             # D term see a fake 50 cm/s spike.
    pp           = PurePursuit(lookahead_distance=LOOKAHEAD, target_speed=LANE_CHANGE_SPEED)

    print('Running. Commands: c=change lane  s=stop  g=go (resume)  q=quit')

    try:
        while True:
            loop_start = time.time()

            # ── Read all sensors ──────────────────────────
            x, y, theta, pose_age = bridge.get_latest_pose()
            cam  = camera.get()
            acc_data = acc.get()

            current_lane  = cam['lane']
            error_cm      = cam['error_cm']
            cam_confidence = cam['confidence']
            cam_curvature  = cam['curvature']
            acc_zone      = acc_data['zone']
            obstacle_lane = acc_data['obstacle_lane']

            # ── Keyboard commands ─────────────────────────
            key = key_pressed()
            if key == 'q':
                print('Quitting.')
                break
            elif key == 's':
                state = 'STOPPED'
                print('[Manual] STOPPED')
            elif key == 'g':
                state = 'LANE_KEEPING'
                err_integral = 0.0
                err_step_sign = 0
                err_step_streak = 0
                lane_flip_streak = 0
                print('[Manual] Resuming LANE_KEEPING')
            elif key == 'c':
                if state != 'LANE_CHANGING':
                    # Determine direction: if in outer lane go inner (left = +y),
                    # if in inner lane go outer (right = -y)
                    direction = +1 if current_lane == 'outer' else -1
                    waypoints  = make_lane_change_waypoints(x, y, theta, direction)
                    last_index = 0
                    state      = 'LANE_CHANGING'
                    print(f'[Manual] Lane change triggered: {current_lane} -> '
                          f'{"inner" if direction > 0 else "outer"}')

            # ── Stale pose safety check ───────────────────
            if pose_age > MAX_POSE_AGE:
                print(f'WARNING: pose stale ({pose_age:.2f}s)')
                bridge.send_targets(0, 0, 0, 0)
                time.sleep(CONTROL_DT)
                continue

            # ── State machine ─────────────────────────────

            if state == 'STOPPED':
                bridge.send_targets(0, 0, 0, 0)

            elif state == 'LANE_KEEPING':
                # PD lane keeping, synchronized to CAMERA frames.
                #
                # The control loop runs at 20 Hz but the camera only
                # delivers ~4-5 frames/s, so we compute a new desired
                # omega ONLY when a new frame arrives. Between frames
                # we hold it (and let the slew limiter finish ramping).
                #
                # Sign convention (unified, both lanes):
                #   error_cm > 0 = car LEFT  of centerline -> omega > 0 (steer RIGHT)
                #   error_cm < 0 = car RIGHT of centerline -> omega < 0 (steer LEFT)
                new_frame = (cam['seq'] != last_cam_seq)
                if new_frame:
                    last_cam_seq = cam['seq']
                    now = cam['stamp']

                    if cam_confidence >= CONF_THRESHOLD:
                        # CLAMP huge errors instead of ignoring them.
                        # A big error (corner entry, drifting off line)
                        # means "steer hard NOW", never "do nothing".
                        meas = max(-MAX_REAL_ERROR, min(MAX_REAL_ERROR, error_cm))

                        # Lane flip: the error basis just shifted
                        # (per-lane calibration + fallback geometry).
                        # v11: PERSISTENT-FLIP reseed (replaces v9's
                        # confidence-only gate). The v10 outer logs
                        # showed every flip arriving on conf=0.40
                        # sep-only frames; v9's rule then NEVER reseeded,
                        # so the filter sat in the WRONG lane's
                        # calibration basis for entire stretches and the
                        # controller chased a ~35cm phantom offset.
                        # New rule: a flip on a conf>=0.8 frame snaps
                        # immediately (unchanged), and a flip reported on
                        # 2 CONSECUTIVE frames is real regardless of
                        # confidence -- single-frame classifier flicker
                        # still can't move the filter more than the
                        # innovation clamp allows.
                        if current_lane in ('inner', 'outer'):
                            if (prev_rep_lane is not None
                                    and current_lane != prev_rep_lane):
                                lane_flip_streak += 1
                                if (cam_confidence >= CONF_FULL_TRUST
                                        or lane_flip_streak >= 2):
                                    filt_err        = meas  # reseed in new basis
                                    err_deriv       = 0.0
                                    err_step_sign   = 0
                                    err_step_streak = 0
                                    lane_flip_streak = 0
                                    prev_rep_lane   = current_lane
                            else:
                                lane_flip_streak = 0
                                prev_rep_lane    = current_lane

                        # Innovation clamp WITH TREND RELEASE (v9).
                        # A single wild frame still moves the filter at
                        # most MAX_ERR_STEP -- but a run of TREND_STEP_N
                        # same-sign clamped innovations is a real,
                        # sustained change (spikes don't repeat with the
                        # same sign), so the clamp opens by
                        # TREND_STEP_GAIN and the filter catches up in a
                        # couple of frames instead of crawling.
                        innov = meas - filt_err
                        s = 1 if innov > 0 else (-1 if innov < 0 else 0)
                        if s != 0 and abs(innov) > MAX_ERR_STEP:
                            if s == err_step_sign:
                                err_step_streak += 1
                            else:
                                err_step_sign   = s
                                err_step_streak = 1
                        else:
                            err_step_sign   = 0
                            err_step_streak = 0
                        step_limit = MAX_ERR_STEP * \
                            (TREND_STEP_GAIN
                             if err_step_streak >= TREND_STEP_N else 1.0)
                        meas = max(filt_err - step_limit,
                                   min(filt_err + step_limit, meas))

                        # Low-pass the error to knock down camera noise.
                        # v9: trust scales LINEARLY with confidence
                        # (conf^2 stacked with the clamp = the crawl).
                        alpha_eff = max(LK_ALPHA_MIN,
                                        LK_ERR_ALPHA * cam_confidence)
                        prev_filt = filt_err
                        filt_err  = alpha_eff * meas + (1.0 - alpha_eff) * filt_err

                        # Derivative of error, per camera frame (cm/s).
                        # This is the damping term: as the car swings back
                        # toward center, d(err)/dt opposes the P term and
                        # eases off the steering BEFORE it overshoots.
                        # Clamped so a single-frame jump can't whip the car.
                        dt_f = None
                        if last_frame_time is not None:
                            dt_f = max(1e-3, min(0.5, now - last_frame_time))
                            err_deriv = (filt_err - prev_filt) / dt_f
                            err_deriv = max(-LK_DERIV_MAX, min(LK_DERIV_MAX, err_deriv))
                        last_frame_time = now
                        last_good_time  = now

                        e = filt_err if abs(filt_err) > DEAD_ZONE_CM else 0.0

                        # Leaky integral term (corner holding).
                        # The leak (time constant LK_I_LEAK_TAU) makes this
                        # track the CURRENT track curvature: it builds up
                        # in a corner and self-clears on the next straight.
                        # Without the leak it latched at its cap after
                        # corners and kept steering while the car was
                        # already centered. Note it integrates filt_err
                        # (NOT the dead-zoned e) so it can unwind even
                        # when the car is inside the dead zone.
                        i_term = (LK_KI * err_integral) / 100.0
                        if dt_f is not None:
                            err_integral *= max(0.0, 1.0 - dt_f / LK_I_LEAK_TAU)
                            # anti-windup: accumulate only while the
                            # command isn't saturated AND the
                            # measurement is trustworthy. Winding the
                            # integral on fallback estimates is how a
                            # biased fallback drags the car across a
                            # line and KEEPS it there.
                            if (abs(omega_des) < LK_MAX_OMEGA * 0.98
                                    and cam_confidence >= CONF_FULL_TRUST):
                                err_integral += filt_err * dt_f
                            i_limit = LK_I_MAX * 100.0 / LK_KI
                            err_integral = max(-i_limit, min(i_limit, err_integral))
                            i_term = (LK_KI * err_integral) / 100.0

                        # Curvature feed-forward (disabled while
                        # K_CURV == 0): supply the corner's sustained
                        # omega immediately, from vision, instead of
                        # waiting several frames for the integral.
                        # Only on trustworthy frames.
                        if K_CURV != 0.0 and cam_confidence >= CONF_FULL_TRUST:
                            omega_ff = K_CURV * cam_curvature
                        else:
                            omega_ff = 0.0

                        omega_des = (LK_KP * e + LK_KD * err_deriv) / 100.0 \
                                    + i_term + omega_ff
                    else:
                        # Low confidence (common in corners): HOLD the last
                        # correction. Corners produce sustained low-conf
                        # frames and the needed omega is sustained too --
                        # decaying immediately made the car drive straight
                        # off the track. Only decay after LOST_HOLD_SEC of
                        # continuously bad measurements.
                        last_frame_time = now
                        if last_good_time is None or (now - last_good_time) > LOST_HOLD_SEC:
                            omega_des *= 0.9

                    omega_des = max(-LK_MAX_OMEGA, min(LK_MAX_OMEGA, omega_des))

                # Slew limiter: omega_cmd chases omega_des at a bounded
                # rate. Kills the step-jumps that whip the chassis around.
                max_step = LK_OMEGA_SLEW * CONTROL_DT
                delta = omega_des - omega_cmd
                omega_cmd += max(-max_step, min(max_step, delta))

                # Corner slowdown: turn radius = v / omega, so cutting
                # v at high steering demand tightens the corner.
                # Lane-dependent base speed: the inner lane's smaller
                # corner radius needs either more omega (impossible,
                # the wheels saturate) or less v (free).
                base_speed = TARGET_SPEED_INNER if current_lane == 'inner' \
                             else TARGET_SPEED
                v_cmd = base_speed * (1.0 - CORNER_SLOWDOWN *
                                      min(1.0, abs(omega_cmd) / LK_MAX_OMEGA))

                v_left, v_right = kin.inverse(v_cmd, omega_cmd)
                rpm_left  = kin.mps_to_rpm(v_left)
                rpm_right = kin.mps_to_rpm(v_right)

                # Saturation handling: if a wheel would exceed its limit,
                # shift BOTH wheels equally. This sacrifices forward speed
                # but preserves the RPM *difference* -- i.e. the turn rate
                # is delivered as commanded instead of being distorted.
                MAX_WHEEL_RPM = 60.0
                MIN_WHEEL_RPM = -12.0   # slight inner-wheel reverse during hard
                                        # cornering. Without reverse the minimum
                                        # turn radius is TRACK/2 = 0.37m at ANY
                                        # speed; with -12 RPM + corner slowdown
                                        # it tightens to ~0.18m at 0.10 m/s.
                hi = max(rpm_left, rpm_right)
                if hi > MAX_WHEEL_RPM:
                    shift = hi - MAX_WHEEL_RPM
                    rpm_left  -= shift
                    rpm_right -= shift
                lo = min(rpm_left, rpm_right)
                if lo < MIN_WHEEL_RPM:
                    shift = MIN_WHEEL_RPM - lo
                    rpm_left  += shift
                    rpm_right += shift
                rpm_left  = max(MIN_WHEEL_RPM, min(MAX_WHEEL_RPM, rpm_left))
                rpm_right = max(MIN_WHEEL_RPM, min(MAX_WHEEL_RPM, rpm_right))

                rpm1 = rpm_left; rpm2 = rpm_right
                rpm3 = rpm_right; rpm4 = rpm_left

                bridge.send_targets(rpm1, rpm2, rpm3, rpm4)

                if new_frame:
                    print(f'[KEEP] lane={current_lane:7s} err={error_cm:+.1f}cm '
                          f'(filt={filt_err:+.1f} d={err_deriv:+.1f} '
                          f'i={(LK_KI*err_integral)/100.0:+.3f}) '
                          f'conf={cam_confidence:.2f} '
                          f'cv={cam_curvature:+.3f} ff={omega_ff:+.3f} '
                          f'w={omega_cmd:.3f} '
                          f'L={rpm_left:.1f} R={rpm_right:.1f} '
                          f'acc={acc_zone}')

            elif state == 'LANE_CHANGING':
                if not waypoints:
                    state = 'LANE_KEEPING'
                    continue

                # Check if goal reached
                goal_x, goal_y = waypoints[-1]
                dist_to_goal = math.hypot(goal_x - x, goal_y - y)

                if dist_to_goal < GOAL_TOLERANCE:
                    print(f'[CHANGE] Complete. Now in '
                          f'{"inner" if current_lane != "outer" else "outer"} lane.')
                    state      = 'LANE_KEEPING'
                    waypoints  = []
                    last_index = 0
                    err_integral = 0.0
                    err_step_sign = 0
                    err_step_streak = 0
                    lane_flip_streak = 0
                else:
                    v, omega, last_index, _ = pp.compute_control(
                        x, y, theta, waypoints, last_index
                    )
                    omega = -omega   # physical flip
                    omega = max(-MAX_OMEGA, min(MAX_OMEGA, omega))

                    v_left, v_right = kin.inverse(v, omega)
                    rpm_left  = kin.mps_to_rpm(v_left)
                    rpm_right = kin.mps_to_rpm(v_right)
                    rpm1 = rpm_left; rpm2 = rpm_right
                    rpm3 = rpm_right; rpm4 = rpm_left

                    bridge.send_targets(rpm1, rpm2, rpm3, rpm4)

                    print(f'[CHANGE] goal={dist_to_goal:.3f}m '
                          f'w={omega:.3f} '
                          f'Lrpm={rpm_left:.1f} Rrpm={rpm_right:.1f}')

            # ── Timing ────────────────────────────────────
            elapsed = time.time() - loop_start
            sleep_t = max(0.0, CONTROL_DT - elapsed)
            time.sleep(sleep_t)

    except KeyboardInterrupt:
        print('Interrupted.')
    finally:
        print('Stopping motors.')
        bridge.send_targets(0, 0, 0, 0)
        time.sleep(0.1)
        bridge.close()


if __name__ == '__main__':
    main()
