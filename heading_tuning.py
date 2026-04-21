import sys
sys.path.append('/home/pi/TurboPi/')
import time
import signal
import math
import HiwonderSDK.mecanum as mecanum
from enum import Enum
import zmq
import json

KP = 1;
KI = 1;
KD = 1;

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


