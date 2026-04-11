import cv2
import os
import time

folder = "parking_data"
if not os.path.exists(folder):
    os.makedirs(folder)


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

if not cap.isOpened():
    print("error: could not opne camera")
    exit()

print("Camera live! press q to quit")
print("Press s to save an image!")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape
    cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 1)
    cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 1) 
    cv2.imshow("camera frame", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        img_name = f"img_{int(time.time())}.jpg"
        save_path = os.path.join(folder, img_name)
        cv2.imwrite(save_path, frame)
        print(f"Saved: {save_path}")
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
