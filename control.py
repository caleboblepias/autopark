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
LATERAL_THRESHOLD = 0.05;
HEADING_THRESHOLD = 5;
DISTANCE_THRESHOLD_1 = 400;
DISTANCE_THRESHOLD_2 = 100;

# PID constants
LK_P = 80;
LK_I = 2;
LK_D = 20;

HK_P = 0.005;
HK_I = 2;
HK_D = 1;

D1K_P = 0.75;
D1K_I = 2;
D1K_D = 1;

D2K_P = 0.25;
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

                if data['type'] == 'sonar':
                    self.DISTANCE_ERR = data["distance_err"]
                    self.LAST_SONAR_TS = data["timestamp"]
                elif data["type"] == "vision":
                    self.SPOT_FOUND = data["has_detection"]
                    if self.SPOT_FOUND:
                        self.LATERAL_ERR = data["lateral_err"]
                        self.HEADING_ERR = data["heading_err"]
                        self.LAST_VISION_TS = data["timestamp"]
                        print(self.LATERAL_ERR)
                
            except zmq.Again:
                break # no messages


start = True

# stop motors
def stop(signum, frame):
    global start

    start = False
    print('Motors stopped')
    chassis.reset_motors()
    time.sleep(2)

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
current_state = State.SEARCH

# initialize perception
perception = Perception()

# fsm based on sensor readings
def update_state(state, perception):
    try:
        match state:
            case State.SEARCH:
                print("SEARCH Mode")
                if (perception.SPOT_FOUND):
                    return State.ALIGN
                else:
                    return State.SEARCH
            case State.ALIGN:
                print("ALIGN Mode")
                if (abs(perception.LATERAL_ERR) < LATERAL_THRESHOLD and abs(perception.HEADING_ERR) < HEADING_THRESHOLD):
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

            case _:
                print("Default")
                return current_state
    except:
        print("update_state failed")
        return current_state

# cmd
class MotorCommand:
    def __init__(self, vx, vy, w):
        self.vx = vx
        self.vy = vy
        self.w = w
        self.vx_prev = None
        self.vy_prev = None
        self.w_prev = None
    def __str__(self):
        return f"MotorCommand: {self.vx}, {self.vy}, {self.w}"



# PID control
def compute_cmd(state, perception):
    cmd = MotorCommand(0, 0, 0)
    try:
        match state:
            case State.SEARCH:
                cmd.vx = 0
                cmd.vy = 0
                cmd.w = 0
            case State.ALIGN:
                cmd.vx = LK_P * perception.LATERAL_ERR * math.cos(math.radians(perception.HEADING_ERR)) 
                cmd.vy = LK_P * perception.LATERAL_ERR * math.sin(math.radians(perception.HEADING_ERR))
                cmd.w = LK_P * perception.HEADING_ERR - (cmd.w - cmd.w_prev) / 0.05
            case State.APPROACH:
                cmd.vx = D1K_P * perception.DISTANCE_ERR
                cmd.vy = 0
                cmd.w = 0
            case State.CREEP:
                cmd.vx = D2K_P * perception.DISTANCE_ERR
                cmd.vy = 0
                cmd.w = 0
            case State.STOP:
                cmd.vx = 0
                cmd.vy = 0
                cmd.w = 0

    except:
        print("compute_cmd failed")
    if (cmd.vx > 40):
        cmd.vx = 40
    if (cmd.vy > 40):
        cmd.vy = 40
    if (abs(cmd.w) > 0.2):
        if (cmd.w > 0):
            cmd.w = 0.2
        else:
            cmd.w = -0.2
    print(cmd)
    return cmd


if __name__ == '__main__':
    #chassis.set_velocity_cartesian(20, 0, 0) 
    
    while True:
        
        # read sensors
        perception.update_perception()

        # update state
        current_state = update_state(current_state, perception)

        # compute command
        cmd = compute_cmd(current_state, perception)
        
        if not start:
            
            print('Motors stopped (main)')
            break

        # send motor command
        #chassis.set_velocity_cartesian(cmd.vy, cmd.vx, cmd.w)
        
        if not start:
            
            print('Motors stopped (main)')
            break

        time.sleep(0.05)

    


