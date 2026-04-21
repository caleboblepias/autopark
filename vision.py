import cv2
import cv2.aruco as aruco
import numpy as np
import zmq
import pickle
import time
import threading
from queue import Queue


class VisionPublisher:
    def __init__(self, calibration_path, marker_size, marker_id, port=5556):
        self.marker_size = marker_size
        self.marker_id = marker_id
        self.running = True

        # calibration
        self.camera_matrix, self.dist_coeffs = self._load_calibration(calibration_path)

        # aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(dictionary, parameters)

        # zmq
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://*:{port}")
        time.sleep(0.5)

        # camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # queues — maxsize=1 means always process latest frame
        self.frame_queue = Queue(maxsize=1)
        self.result_queue = Queue(maxsize=1)

        print(f"Vision running on port {port}")

    def _load_calibration(self, path):
        with open(path, 'rb') as f:
            calib = pickle.load(f)
        return calib['camera_matrix'], calib['dist_coeffs']

    """Thread samples camera as fast as possible, always keeps latest frame"""
    def _capture_thread(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            # drop old frame if detection hasn't consumed it yet
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except:
                    pass

            self.frame_queue.put(frame)

    """Runs ArUco detection on latest frame, always puts result even if None"""
    def _detection_thread(self):
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=1.0)
            except:
                continue

            result = self._detect(frame)

            # always put result including None so publish thread knows marker is gone
            if self.result_queue.full():
                try:
                    self.result_queue.get_nowait()
                except:
                    pass

            self.result_queue.put(result)

    """Publishes at fixed 50Hz, always uses most recent detection"""
    def _publish_thread(self):
        publish_interval = 1.0 / 50.0
        last_result = None

        while self.running:
            start = time.time()

            try:
                detected = self.result_queue.get_nowait()
                if detected is not None:
                    last_result = detected   # valid detection — update
                else:
                    last_result = None       # marker lost — clear immediately
            except:
                pass  # no new result this cycle — reuse last_result as-is

            if last_result:
                self.socket.send_json(last_result)
            else:
                self.socket.send_json({
                    'type': 'vision',
                    'has_detection': False,
                    'timestamp': time.time()
                })

            # sleep remainder to maintain 50Hz
            elapsed = time.time() - start
            sleep_time = publish_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return None

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == self.marker_id:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i:i+1], self.marker_size,
                    self.camera_matrix, self.dist_coeffs
                )

                x = float(tvec[0][0][0])
                y = float(tvec[0][0][1])
                z = float(tvec[0][0][2])

                R, _ = cv2.Rodrigues(rvec)
                heading_err = float(np.degrees(np.arctan2(R[0, 2], R[2, 2])))

                img_w = frame.shape[1]
                cx = corners[i][0][:, 0].mean() / img_w
                lateral_err = float(cx - 0.5)

                return {
                    'type': 'vision',
                    'has_detection': True,
                    'timestamp': time.time(),
                    'lateral_err': lateral_err,
                    'heading_err': heading_err,
                    'distance': z,
                    'tvec_x': x,
                    'tvec_y': y,
                    'tvec_z': z,
                }

        return None

    def run(self):
        threads = [
            threading.Thread(target=self._capture_thread, daemon=True),
            threading.Thread(target=self._detection_thread, daemon=True),
            threading.Thread(target=self._publish_thread, daemon=True),
        ]

        for t in threads:
            t.start()

        try:
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Vision shutting down")
        finally:
            self.running = False
            self.cleanup()

    def cleanup(self):
        self.cap.release()
        self.socket.close()
        self.context.term()


if __name__ == '__main__':
    vision = VisionPublisher(
        calibration_path='./calibration/camera_calibration.pkl',
        marker_size=0.0508,
        marker_id=0,
        port=5556
    )
    vision.run()