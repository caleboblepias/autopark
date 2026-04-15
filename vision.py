# vision.py
import cv2
import cv2.aruco as aruco
import numpy as np
import zmq
import pickle
import time

# --- Load calibration ---
with open('camera_calibration.pkl', 'rb') as f:
    calib = pickle.load(f)
camera_matrix = calib['camera_matrix']
dist_coeffs = calib['dist_coeffs']

# --- ArUco setup ---
MARKER_SIZE = 0.0889  # meters — measure your printed marker exactly
MARKER_ID = 0

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

# --- ZeroMQ setup ---
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")
time.sleep(0.5)  # slow joiner fix

# --- Camera ---
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Vision running, publishing on port 5555")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    msg = {'has_detection': False, 'timestamp': time.time()}

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == MARKER_ID:
                # Pose estimation
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i:i+1], MARKER_SIZE, camera_matrix, dist_coeffs
                )

                # tvec = [x, y, z] in meters relative to camera
                x = float(tvec[0][0][0])   # lateral offset
                y = float(tvec[0][0][1])   # vertical offset
                z = float(tvec[0][0][2])   # distance (depth)

                # Heading error from rotation vector
                R, _ = cv2.Rodrigues(rvec)
                heading_error = float(np.arctan2(R[0, 2], R[2, 2]))

                # Lateral error normalized
                img_w = frame.shape[1]
                cx = (corners[i][0][:, 0].mean()) / img_w
                lateral_error = float(cx - 0.5)

                msg = {
                    'has_detection': True,
                    'timestamp': time.time(),
                    'lateral_error': lateral_error,
                    'heading_error': heading_error,
                    'distance': z,          # meters to marker
                    'tvec_x': x,
                    'tvec_y': y,
                    'tvec_z': z,
                }
                break  # only care about first matching marker

    socket.send_json(msg)