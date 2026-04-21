import HiwonderSDK.mecanum as mecanum
import time
import signal
import sys
chassis = mecanum.MecanumChassis()
start = True
def stop(signum, frame):
    global start

    start = False
    print('Motors stopped')
    chassis.reset_motors()
signal.signal(signal.SIGINT, stop)

if __name__ == '__main__':
    chassis.set_velocity_cartesian(0,0,0.2)
    while True:
       if not start:
            print('Motors stopped (main)')
            break
