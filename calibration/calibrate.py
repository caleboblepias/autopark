# calibrate.py
import cv2
import numpy as np
import pickle

CHECKERBOARD = (9, 6)  # inner corners
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = []  # 3D points
imgpoints = []  # 2D points

cap = cv2.VideoCapture(0)
saved = 0

print("Press 's' to save a frame, 'q' when done (aim for 20+ frames)")

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if found:
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        display = frame.copy()
        cv2.drawChessboardCorners(display, CHECKERBOARD, corners2, found)
        cv2.putText(display, f"Saved: {saved} | Press 's' to capture",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.imshow('Calibration', display)
    else:
        cv2.putText(frame, "No checkerboard found",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        cv2.imshow('Calibration', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') and found:
        objpoints.append(objp)
        imgpoints.append(corners2)
        saved += 1
        print(f"Saved frame {saved}")
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Calibrate
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# Save to file — you only ever do this once
with open('camera_calibration.pkl', 'wb') as f:
    pickle.dump({'camera_matrix': camera_matrix, 'dist_coeffs': dist_coeffs}, f)

print(f"Calibration complete. Reprojection error: {ret:.4f}")
print("Saved to camera_calibration.pkl")