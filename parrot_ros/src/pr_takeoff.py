#!/home/praetor/parrot/bin/python

import rospy
import olympe
import os
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing


DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")

def test_takeoff():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()
    assert drone(TakeOff()).wait().success()
    time.sleep(10)
    assert drone(Landing()).wait().success()
    drone.disconnect()



if __name__ == "__main__":

    rospy.init_node('sphinx_takeoff_node', anonymous= False)
    rospy.loginfo("Takeoff Initiated")

    test_takeoff()

