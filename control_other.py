import sys
import time
import signal
import math
from enum import Enum
import zmq
import json

# Ensure paths are correct for TurboPi
sys.path.append('/home/pi/TurboPi/')
sys.path.append('/home/pi/TurboPi/HiwonderSDK')
import HiwonderSDK.mecanum as mecanum
import ros_robot_controller_sdk as rrc

# ─── Thresholds & Tuning ──────────────────────────────────────
LATERAL_THRESHOLD    = 0.05
HEADING_THRESHOLD    = 15.0
DISTANCE_THRESHOLD_1 = 400
DISTANCE_THRESHOLD_2 = 100

W_CMD  = 0.2
VY_CMD = 25
PAN_CENTER = 1500
PAN_MIN    = 550
PAN_MAX    = 2450

# ─── Hardware ─────────────────────────────────────────────────
chassis = mecanum.MecanumChassis()
board   = rrc.Board()

class State(Enum):
    SEARCH         = 0
    CONFIRM_SIGHT  = 1  # New state: Stop and stare to ensure marker is real
    ALIGN_HEADING  = 2
    ALIGN_LATERAL  = 3
    APPROACH       = 4
    CREEP          = 5
    STOP           = 6

class Perception:
    def __init__(self):
        self.context = zmq.Context()
        self.sub = self.context.socket(zmq.SUB)
        self.sub.connect("tcp://localhost:5555")
        self.sub.connect("tcp://localhost:5556")
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")

        self.SPOT_FOUND    = False
        self.LATERAL_ERR   = None
        self.HEADING_ERR   = None
        self.DISTANCE_ERR  = None
        
        # --- NEW: Confidence Logic ---
        self.confidence_threshold = 5  # Must see marker for 5 frames
        self.lost_threshold = 10       # Must lose marker for 10 frames to drop it
        self.frames_seen = 0
        self.frames_lost = 0

    def update(self):
        while True:
            try:
                msg  = self.sub.recv_string(flags=zmq.NOBLOCK)
                data = json.loads(msg)

                if data['type'] == 'sonar':
                    self.DISTANCE_ERR = data['distance_err']

                elif data['type'] == 'vision':
                    if data['has_detection']:
                        self.frames_seen += 1
                        self.frames_lost = 0
                        self.LATERAL_ERR = data['lateral_err']
                        self.HEADING_ERR = data['heading_err']
                    else:
                        self.frames_lost += 1
                        self.frames_seen = max(0, self.frames_seen - 1)

                    # Update Boolean Sight
                    if self.frames_seen >= self.confidence_threshold:
                        self.SPOT_FOUND = True
                    if self.frames_lost >= self.lost_threshold:
                        self.SPOT_FOUND = False
                        self.LATERAL_ERR = None
                        self.HEADING_ERR = None

            except zmq.Again:
                break

# ─── Global State ─────────────────────────────────────────────
perception = Perception()
pan_angle  = 1500
pan_dir    = 40  # Slower increment for "Step-and-Stare"
running    = True

def handle_stop(signum, frame):
    global running
    running = False
    chassis.reset_motors()
    board.pwm_servo_set_position(1, [[1, 1500], [2, 1500]])

signal.signal(signal.SIGINT, handle_stop)

def update_state(state, p):
    if state == State.SEARCH and p.SPOT_FOUND:
        return State.CONFIRM_SIGHT
    
    if state == State.CONFIRM_SIGHT:
        if not p.SPOT_FOUND: return State.SEARCH
        # Stay in confirm for a moment (handled in compute_cmd)
        return State.ALIGN_HEADING if p.frames_seen > 15 else State.CONFIRM_SIGHT

    if state in [State.ALIGN_HEADING, State.ALIGN_LATERAL, State.APPROACH]:
        if not p.SPOT_FOUND:
            print(">>> Lost target. Returning to SEARCH.")
            return State.SEARCH

    # Standard transitions
    if state == State.ALIGN_HEADING and abs(p.HEADING_ERR or 0) < HEADING_THRESHOLD:
        return State.ALIGN_LATERAL
    if state == State.ALIGN_LATERAL and abs(p.LATERAL_ERR or 0) < LATERAL_THRESHOLD:
        return State.APPROACH
    if state == State.APPROACH and p.DISTANCE_ERR and p.DISTANCE_ERR < DISTANCE_THRESHOLD_1:
        return State.CREEP
    if state == State.CREEP and p.DISTANCE_ERR and p.DISTANCE_ERR < DISTANCE_THRESHOLD_2:
        return State.STOP
    
    return state

def compute_cmd(state, p):
    global pan_angle, pan_dir
    vx, vy, w = 0, 0, 0

    if state == State.SEARCH:
        # STEP-AND-STARE logic
        pan_angle += pan_dir
        if pan_angle >= PAN_MAX or pan_angle <= PAN_MIN:
            pan_dir *= -1
        board.pwm_servo_set_position(0.1, [[2, pan_angle]])
        time.sleep(0.1) # Give camera time to capture a frame while stationary

    elif state == State.CONFIRM_SIGHT:
        # Stop everything and look directly at it
        chassis.set_velocity_cartesian(0, 0, 0)

    elif state == State.ALIGN_HEADING:
        # Snap camera to center so error is relative to chassis
        board.pwm_servo_set_position(0.2, [[2, PAN_CENTER]])
        if p.HEADING_ERR:
            w = W_CMD if p.HEADING_ERR > 0 else -W_CMD
            # Pulse for 100ms then stop to check
            chassis.set_velocity_cartesian(0, 0, w)
            time.sleep(0.1)
            chassis.set_velocity_cartesian(0, 0, 0)
            time.sleep(0.2)

    elif state == State.ALIGN_LATERAL:
        if p.LATERAL_ERR:
            vy_val = VY_CMD if p.LATERAL_ERR > 0 else -VY_CMD
            chassis.set_velocity_cartesian(vy_val, 0, 0)
            time.sleep(0.1)
            chassis.set_velocity_cartesian(0, 0, 0)
            time.sleep(0.2)

    elif state == State.APPROACH:
        vx = max(15, min(30, 0.75 * (p.DISTANCE_ERR or 0)))

    elif state == State.CREEP:
        vx = 15 # Constant slow creep

    return vx, vy, w

if __name__ == '__main__':
    current_state = State.SEARCH
    board.pwm_servo_set_position(0.5, [[2, PAN_CENTER]])
    
    while running:
        perception.update()
        current_state = update_state(current_state, perception)
        vx, vy, w = compute_cmd(current_state, perception)
        
        # Only apply continuous velocity in Approach/Creep
        if current_state in [State.APPROACH, State.CREEP]:
            chassis.set_velocity_cartesian(vy, vx, w)
        
        time.sleep(0.05)