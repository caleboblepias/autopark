import sys
# Make sure this points to the directory containing ros_robot_controller_sdk
sys.path.append('/home/pi/TurboPi/HiwonderSDK/') 

import cv2
import os
import time
import ros_robot_controller_sdk as rrc
import HiwonderSDK.mecanum as mecanum

# Initialize the board
board = rrc.Board()
chassis = mecanum.MecanumChassis()


# Setup folder
folder = "parking_data"
if not os.path.exists(folder):
    os.makedirs(folder)

# Camera Setup
cap = cv2.VideoCapture(0)
# Note: Your camera is YUYV, so it will likely default to 640x480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Error: Could not open camera")
    exit()

print("Camera live!")
print("Controls: WASD to move camera, Z to save image, Q to quit")

# Default positions (1500 is typically center for TurboPi PWM servos)
pan = 1500
tilt = 1500

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Draw crosshairs
    h, w, _ = frame.shape
    cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 1)
    cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 1) 
    
    # Display the frame
    cv2.imshow("camera frame", frame)

    # Check for key presses (waits 1ms)
    key = cv2.waitKey(1) & 0xFF

    # Servo Control (Servo 1: Pan, Servo 2: Tilt)
    if key == ord('w'):
        tilt = max(500, tilt - 500)
        board.pwm_servo_set_position(0.02, [[1, tilt]])
        print(f"Tilting Up: {tilt}")
    elif key == ord('s'):
        tilt = min(2000, tilt + 500)
        board.pwm_servo_set_position(0.02, [[1, tilt]])
        print(f"Tilting Down: {tilt}")
    elif key == ord('a'):
        pan = min(2500, pan + 500)
        board.pwm_servo_set_position(0.02, [[2, pan]])
        print(f"Panning Left: {pan}")
    elif key == ord('d'):
        pan = max(500, pan - 500)
        board.pwm_servo_set_position(0.02, [[2, pan]])
        print(f"Panning Right: {pan}")
    elif key == ord('p'):
        chassis.set_velocity_cartesian(80, 0, 0)  # Move forward

    # Save and Quit logic
    elif key == ord('z'):
        img_name = f"img_{int(time.time())}.jpg"
        save_path = os.path.join(folder, img_name)
        cv2.imwrite(save_path, frame)
        print(f"Saved: {save_path}")
    elif key == ord('q'):
        break

chassis.set_velocity_cartesian(0, 0, 0)  # Move forward
cap.release()
cv2.destroyAllWindows()