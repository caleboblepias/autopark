# generate_marker.py
import cv2
import cv2.aruco as aruco

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# ID 0, 400x400 pixels
marker = aruco.generateImageMarker(dictionary, 0, 200)
cv2.imwrite('parking_marker.png', marker)
print("Saved parking_marker.png — print this and attach to your sign")