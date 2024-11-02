import os
import time
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing

DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")

def test_connection():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    if DRONE_RTSP_PORT is not None:
        drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"
    
    assert drone(TakeOff()).wait().success()
    time.sleep(10)
    assert drone(Landing()).wait().success()    
    drone.disconnect()

if __name__ == "__main__":
    test_connection()
