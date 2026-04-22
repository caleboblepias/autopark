import sys
sys.path.append('/home/pi/TurboPi/')
import time
import signal
import math
import HiwonderSDK.mecanum as mecanum
from enum import Enum
import zmq
import json

sys.path.append('/home/pi/TurboPi/HiwonderSDK')
import ros_robot_controller_sdk as rrc

# ─── Thresholds ───────────────────────────────────────────────
LATERAL_THRESHOLD    = 0.05   # normalized [-0.5, 0.5]
HEADING_THRESHOLD    = 15.0   # degrees — wider to account for bang-bang overshoot
DISTANCE_THRESHOLD_1 = 400    # mm — switch to CREEP
DISTANCE_THRESHOLD_2 = 100    # mm — stop

# ─── Bang-bang constants ───────────────────────────────────────
W_CMD        = 0.2    # minimum w to actuate rotation
VY_CMD       = 25     # minimum vy to actuate lateral strafe

# ─── Approach constants ────────────────────────────────────────
D1K_P = 0.75
D2K_P = 0.25

# ─── Pulse tuning ─────────────────────────────────────────────
# measure empirically: how many degrees does robot rotate in 100ms at w=0.2?
DEG_PER_100MS  = 8.0   # update this after measuring
MIN_PULSE      = 0.05  # seconds
MAX_PULSE      = 0.20  # seconds
SETTLE_TIME    = 0.15  # seconds — wait after pulse for robot to stop

# ─── Hardware ─────────────────────────────────────────────────
chassis = mecanum.MecanumChassis()
board   = rrc.Board()

# ─── Servo state ──────────────────────────────────────────────
pan_angle   = 1500
pan_dir     = 100     # +100 = sweeping right, -100 = sweeping left
pan_rounds  = 0
PAN_MIN     = 550
PAN_MAX     = 2450
PAN_CENTER  = 1500

# ─── Perception ───────────────────────────────────────────────
class Perception:
    def __init__(self):
        self.context = zmq.Context()
        self.sub = self.context.socket(zmq.SUB)
        self.sub.connect("tcp://localhost:5555")  # sonar
        self.sub.connect("tcp://localhost:5556")  # vision
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")

        self.SPOT_FOUND   = False
        self.LATERAL_ERR  = None
        self.HEADING_ERR  = None
        self.DISTANCE_ERR = None
        self.LAST_SONAR_TS  = None
        self.LAST_VISION_TS = None

    def update(self):
        """Drain all queued messages, keep latest per type"""
        while True:
            try:
                msg  = self.sub.recv_string(flags=zmq.NOBLOCK)
                data = json.loads(msg)

                if data['type'] == 'sonar':
                    self.DISTANCE_ERR   = data['distance_err']
                    self.LAST_SONAR_TS  = data['timestamp']

                elif data['type'] == 'vision':
                    self.SPOT_FOUND     = data['has_detection']
                    self.LAST_VISION_TS = data['timestamp']
                    if self.SPOT_FOUND:
                        self.LATERAL_ERR = data['lateral_err']
                        self.HEADING_ERR = data['heading_err']
                    else:
                        self.LATERAL_ERR = None
                        self.HEADING_ERR = None

            except zmq.Again:
                break
            except Exception as e:
                print(f"Perception error: {e}")
                break

# ─── Stop signal ──────────────────────────────────────────────
running = True

def handle_stop(signum, frame):
    global running
    running = False
    print('Stopping...')
    board.pwm_servo_set_position(1, [[1, PAN_CENTER], [2, PAN_CENTER]])
    chassis.reset_motors()

signal.signal(signal.SIGINT, handle_stop)

# ─── State machine ────────────────────────────────────────────
class State(Enum):
    SEARCH        = 0
    ALIGN_HEADING = 1
    ALIGN_LATERAL = 2
    APPROACH      = 3
    CREEP         = 4
    STOP          = 5

# ─── Motor command ────────────────────────────────────────────
class MotorCommand:
    def __init__(self, vx=0, vy=0, w=0):
        self.vx = vx
        self.vy = vy
        self.w  = w

    def __str__(self):
        return f"vx={self.vx:.2f}  vy={self.vy:.2f}  w={self.w:.3f}"

# ─── Helpers ──────────────────────────────────────────────────
def pulse_rotate(direction, heading_err_deg):
    """
    Rotate for a duration scaled to error magnitude then settle.
    direction: +1 or -1
    """
    error    = abs(heading_err_deg)
    duration = MIN_PULSE + (MAX_PULSE - MIN_PULSE) * min(error / 90.0, 1.0)
    duration *= 0.75  # intentional undershoot — better to take two small pulses

    chassis.set_velocity_cartesian(0, 0, direction * W_CMD)
    time.sleep(duration)
    chassis.set_velocity_cartesian(0, 0, 0)
    time.sleep(SETTLE_TIME)  # let robot physically stop before reading heading again

def pulse_strafe(direction):
    """
    Strafe for a fixed short duration then settle.
    direction: +1 or -1
    """
    chassis.set_velocity_cartesian(direction * VY_CMD, 0, 0)
    time.sleep(0.08)
    chassis.set_velocity_cartesian(0, 0, 0)
    time.sleep(0.1)

# ─── FSM update ───────────────────────────────────────────────
def update_state(state, p):
    try:
        match state:
            case State.SEARCH:
                if p.SPOT_FOUND:
                    print(">>> SEARCH → ALIGN_HEADING")
                    return State.ALIGN_HEADING
                return State.SEARCH

            case State.ALIGN_HEADING:
                # lost marker — go back to search
                if not p.SPOT_FOUND or p.HEADING_ERR is None:
                    print(">>> ALIGN_HEADING → SEARCH (marker lost)")
                    return State.SEARCH
                if abs(p.HEADING_ERR) < HEADING_THRESHOLD:
                    print(">>> ALIGN_HEADING → ALIGN_LATERAL")
                    chassis.set_velocity_cartesian(0, 0, 0)
                    time.sleep(0.3)  # settle before lateral correction
                    return State.ALIGN_LATERAL
                return State.ALIGN_HEADING

            case State.ALIGN_LATERAL:
                # lost marker — go back to search
                if not p.SPOT_FOUND or p.LATERAL_ERR is None:
                    print(">>> ALIGN_LATERAL → SEARCH (marker lost)")
                    return State.SEARCH
                if abs(p.LATERAL_ERR) < LATERAL_THRESHOLD:
                    print(">>> ALIGN_LATERAL → APPROACH")
                    chassis.set_velocity_cartesian(0, 0, 0)
                    time.sleep(0.3)
                    return State.APPROACH
                return State.ALIGN_LATERAL

            case State.APPROACH:
                if p.DISTANCE_ERR is not None and p.DISTANCE_ERR < DISTANCE_THRESHOLD_1:
                    print(">>> APPROACH → CREEP")
                    return State.CREEP
                return State.APPROACH

            case State.CREEP:
                if p.DISTANCE_ERR is not None and p.DISTANCE_ERR < DISTANCE_THRESHOLD_2:
                    print(">>> CREEP → STOP")
                    return State.STOP
                return State.CREEP

            case State.STOP:
                return State.STOP

    except Exception as e:
        print(f"update_state error: {e}")
    return state

# ─── Command compute ──────────────────────────────────────────
def compute_cmd(state, p):
    global pan_angle, pan_dir, pan_rounds
    cmd = MotorCommand()

    try:
        match state:

            # ── SEARCH: pan camera, drive forward periodically ──
            case State.SEARCH:
                cmd.w  = 0
                cmd.vx = 0
                cmd.vy = 0

                # pan sweep
                if pan_angle >= PAN_MAX:
                    pan_dir = -100
                elif pan_angle <= PAN_MIN:
                    pan_dir = 100
                    pan_rounds += 1

                # after 2 full sweeps with no detection, drive forward a bit
                if pan_rounds >= 2 and pan_angle == PAN_CENTER:
                    chassis.set_velocity_cartesian(0, 20, 0)
                    time.sleep(2.0)
                    chassis.set_velocity_cartesian(0, 0, 0)
                    pan_rounds = 0

                pan_angle += pan_dir
                pan_angle  = max(PAN_MIN, min(PAN_MAX, pan_angle))
                board.pwm_servo_set_position(0.02, [[2, pan_angle]])

            # ── ALIGN_HEADING: bang-bang rotation with pulse timing ──
            case State.ALIGN_HEADING:
                # center servo before rotating body
                board.pwm_servo_set_position(0.2, [[2, PAN_CENTER]])
                pan_angle = PAN_CENTER

                if p.HEADING_ERR is None:
                    return cmd

                if abs(p.HEADING_ERR) > HEADING_THRESHOLD:
                    direction = 1 if p.HEADING_ERR > 0 else -1
                    pulse_rotate(direction, p.HEADING_ERR)

                cmd.w  = 0
                cmd.vx = 0
                cmd.vy = 0

            # ── ALIGN_LATERAL: bang-bang strafe ──
            case State.ALIGN_LATERAL:
                pan_angle = PAN_CENTER

                if p.LATERAL_ERR is None:
                    return cmd

                if abs(p.LATERAL_ERR) > LATERAL_THRESHOLD:
                    direction = 1 if p.LATERAL_ERR > 0 else -1
                    pulse_strafe(direction)

                cmd.vx = 0
                cmd.vy = 0
                cmd.w  = 0

            # ── APPROACH: proportional forward drive ──
            case State.APPROACH:
                if p.DISTANCE_ERR is None:
                    return cmd
                cmd.vx = D1K_P * p.DISTANCE_ERR
                cmd.vy = 0
                cmd.w  = 0

            # ── CREEP: slower proportional forward drive ──
            case State.CREEP:
                if p.DISTANCE_ERR is None:
                    return cmd
                cmd.vx = D2K_P * p.DISTANCE_ERR
                cmd.vy = 0
                cmd.w  = 0

            # ── STOP ──
            case State.STOP:
                chassis.set_velocity_cartesian(0, 0, 0)
                cmd.vx = 0
                cmd.vy = 0
                cmd.w  = 0

    except Exception as e:
        print(f"compute_cmd error: {e}")

    # clamp outputs
    cmd.vx = max(-30, min(30, cmd.vx))
    cmd.vy = max(-30, min(30, cmd.vy))
    cmd.w  = max(-0.2, min(0.2, cmd.w))

    print(f"[{state.name}] {cmd}")
    return cmd

# ─── Main loop ────────────────────────────────────────────────
if __name__ == '__main__':
    perception   = Perception()
    current_state = State.SEARCH

    # center servo on startup
    board.pwm_servo_set_position(1, [[2, PAN_CENTER]])
    time.sleep(1)

    while running:
        # read all pending sensor messages
        perception.update()

        # update FSM
        current_state = update_state(current_state, perception)

        # compute motor command
        cmd = compute_cmd(current_state, perception)

        # send command — note axis mapping: (vy, vx, w)
        chassis.set_velocity_cartesian(cmd.vy, cmd.vx, cmd.w)

        time.sleep(0.05)  # 20Hz main loop

    print("Shutdown complete")