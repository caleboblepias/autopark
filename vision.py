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
MARKER_SIZE = 0.0889  # meters
MARKER_ID = 0

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

# --- ZeroMQ setup ---
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")
time.sleep(0.5)

# --- Camera ---
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Vision running — press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    msg = {'has_detection': False, 'timestamp': time.time()}
    debug = frame.copy()

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == MARKER_ID:
                # Pose estimation
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i:i+1], MARKER_SIZE, camera_matrix, dist_coeffs
                )

                x = float(tvec[0][0][0])
                y = float(tvec[0][0][1])
                z = float(tvec[0][0][2])

                R, _ = cv2.Rodrigues(rvec)
                heading_error = float(np.arctan2(R[0, 2], R[2, 2]))

                img_w = frame.shape[1]
                cx = (corners[i][0][:, 0].mean()) / img_w
                lateral_error = float(cx - 0.5)

                msg = {
                    'has_detection': True,
                    'timestamp': time.time(),
                    'lateral_error': lateral_error,
                    'heading_error': heading_error,
                    'distance': z,
                    'tvec_x': x,
                    'tvec_y': y,
                    'tvec_z': z,
                }

                # --- Debug visuals ---
                # Draw marker border
                aruco.drawDetectedMarkers(debug, corners[i:i+1], ids[i:i+1])

                # Draw coordinate axes on marker
                cv2.drawFrameAxes(debug, camera_matrix, dist_coeffs,
                                  rvec, tvec, MARKER_SIZE * 0.5)

                # Overlay text
                cv2.putText(debug, f"Lateral: {lateral_error:+.3f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(debug, f"Heading: {heading_error:+.3f} rad",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(debug, f"Distance: {z:.3f} m",
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(debug, f"X: {x:+.3f} Y: {y:+.3f}",
                            (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                break

    else:
        cv2.putText(debug, "No marker detected",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Draw image center reference line
    h, w = debug.shape[:2]
    cv2.line(debug, (w//2, 0), (w//2, h), (255, 255, 0), 1)

    cv2.imshow('Vision Debug', debug)
    socket.send_json(msg)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()