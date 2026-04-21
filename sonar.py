import zmq
import json
import time
import HiwonderSDK.Sonar as Sonar

context = zmq.Context()
socket = context.socket(zmq.PUB)

socket.bind("tcp://*:5555")
s = Sonar.Sonar()

while True:
    msg = {
        "type": "sonar",
        "timestamp": time.time(),
        "distance_err": s.getDistance()
    }
    print(msg)
    socket.send_string(json.dumps(msg))
    time.sleep(0.02) # 50 Hz
