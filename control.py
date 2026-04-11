import sys
sys.path.append('/home/pi/TurboPi/')
import time
import signal
import HiwonderSDK.mecanum as mecanum

chassis = mecanum.MecanumChassis()

chassis.set_velocity_cartesian(10, 0, 0)
