# vision.py
import cv2
import cv2.aruco as aruco
import numpy as np
import zmq
import pickle
import time

# --- Load calibration ---
with open('./calibration/camera_calibration.pkl', 'rb') as f:
    calib = pickle.load(f)
camera_matrix = calib['camera_matrix']
dist_coeffs = calib['dist_coeffs']

# --- ArUco setup ---
MARKER_SIZE = 0.0508  # meters
MARKER_ID = 0

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

# --- ZeroMQ setup ---
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5556")
time.sleep(0.5)

# --- Camera ---
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Vision running")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    msg = {'type': 'vision', 'has_detection': False, 'timestamp': time.time()}

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == MARKER_ID:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i:i+1], MARKER_SIZE, camera_matrix, dist_coeffs
                )

                x = float(tvec[0][0][0])
                y = float(tvec[0][0][1])
                z = float(tvec[0][0][2])

                R, _ = cv2.Rodrigues(rvec)
                heading_error = float(np.degrees(np.arctan2(R[0, 2], R[2, 2])))

                img_w = frame.shape[1]
                cx = (corners[i][0][:, 0].mean()) / img_w
                lateral_error = float(cx - 0.5)

                msg = {
                    'type': 'vision',
                    'has_detection': True,
                    'timestamp': time.time(),
                    'lateral_err': lateral_error,
                    'heading_err': heading_error,
                    'distance': z,
                    'tvec_x': x,
                    'tvec_y': y,
                    'tvec_z': z,
                }
                break

    socket.send_json(msg)