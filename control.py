import sys
sys.path.append('/home/pi/TurboPi/')
import time
import signal
import math
import HiwonderSDK.mecanum as mecanum
from enum import Enum
import zmq
import json

# threshold definition
CONFIDENCE_THRESHOLD = 1;
LATERAL_THRESHOLD = 1;
HEADING_THRESHOLD = 1;
DISTANCE_THRESHOLD_1 = 1;
DISTANCE_THRESHOLD_2 = 0.5;

# PID constants
LK_P = 3;
LK_I = 2;
LK_D = 1;

HK_P = 3;
HK_I = 2;
HK_D = 1;

D1K_P = 3;
D1K_I = 2;
D1K_D = 1;

D2K_P = 3;
D2K_I = 2;
D2K_D = 1;

# MecanumChassis instance
chassis = mecanum.MecanumChassis()

# perception
class Perception:
    def __init__(self):
        self.context = zmq.Context()

        self.sub = self.context.socket(zmq.SUB)
        self.sub.connect("tcp://localhost:5555") # sonar
        self.sub.connect("tcp://localhost:5556") # vision
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")

        self.SPOT_FOUND = None
        self.CONFIDENCE = None
        self.LATERAL_ERR = None
        self.HEADING_ERR = None
        self.DISTANCE_ERR = None
        self.LAST_SONAR_TS = None
        self.LAST_VISION_TS = None

    def update_perception(self):
        while True:
            try:
                msg = self.sub.recv_string(flags=zmq.NOBLOCK)
                data = json.loads(msg)

                if data["type"] == "sonar":
                    self.DISTANCE_ERR = data["distance_err"]
                    self.LAST_SONAR_TS = data["timestamp"]
                elif data["type"] == "vision":
                    self.LATERAL_ERR = data["lateral_err"]
                    self.HEADING_ERR = data["heading_err"]
                    self.LAST_VISION_TS = data["timestamp"]

                print(msg)
                print(self.DISTANCE_ERR)
            except zmq.Again:
                break # no messages


start = True

# stop motors
def stop(signum, frame):
    global start

    start = False
    print('Motors stopped')
    chassis.reset_motors()

# redirect Ctrl-C to stop function
signal.signal(signal.SIGINT, stop)

# state definition
class State(Enum):
    SEARCH = 0
    ALIGN = 1
    APPROACH = 2
    CREEP = 3
    STOP = 4

# initialize state
current_state = State.APPROACH

# initialize perception
perception = Perception()

# fsm based on sensor readings
def update_state(state, perception):
    match state:
        case State.SEARCH:
            print("SEARCH Mode")
            if (perception.SPOT_FOUND and perception.CONFIDENCE > CONFIDENCE_THRESHOLD):
                return State.ALIGN
            else:
                return State.SEARCH
        case State.ALIGN:
            print("ALIGN Mode")
            if (perception.LATERAL_ERR < LATERAL_THRESHOLD and perception.HEADING_ERR < HEADING_THRESHOLD):
                return State.APPROACH
            else:
                return State.ALIGN
        case State.APPROACH:
            print("APPROACH Mode")
            if (perception.DISTANCE_ERR < DISTANCE_THRESHOLD_1):
                return State.CREEP
            else:
                return State.APPROACH
        case State.CREEP:
            print("CREEP Mode")
            if (perception.DISTANCE_ERR < DISTANCE_THRESHOLD_2):
                return State.STOP
            else:
                return State.CREEP
        case State.STOP:
            print("STOP Mode")
            return State.STOP

# cmd
class MotorCommand:
    def __init__(self, vx, vy, w):
        self.vx = vx
        self.vy = vy
        self.w = w

    def __str__(self):
        return f"MotorCommand: {self.vx}, {self.vy}, {self.w}"

# PID control
def compute_cmd(state, perception):
    cmd = MotorCommand(0, 0, 0)
    match state:
        case State.SEARCH:
            cmd.vx = 0
            cmd.vy = 0
            cmd.w = 0
        case State.ALIGN:
            cmd.vx = -LK_P * perception.LATERAL_ERR * math.cos(perception.HEADING_ERR)
            cmd.vy = -LK_P * perception.LATERAL_ERR * math.sin(perception.HEADING_ERR)
            cmd.w = -LK_P * perception.HEADING_ERR
        case State.APPROACH:
            cmd.vx = -D1K_P * perception.DISTANCE_ERR
            cmd.vy = 0
            cmd.w = 0
        case State.CREEP:
            cmd.vx = -D2K_P * perception.DISTANCE_ERR
            cmd.vy = 0
            cmd.w = 0
        case State.STOP:
            cmd.vx = 0
            cmd.vy = 0
            cmd.w = 0
    print(cmd)
    return cmd


if __name__ == '__main__':
    #chassis.set_velocity_cartesian(80, 0, 0) 
    while True:
        
        # read sensors
        perception.update_perception()

        # update state
        #current_state = update_state(current_state, perception)

        # compute command
        #cmd = compute_cmd(current_state, perception)

        # send motor command
        #chassis.set_velocity_cartesian(cmd.vx, cmd.vy, cmd.w)
        
        if not start:
            
            print('Motors stopped (main)')
            break

        time.sleep(0.05)


