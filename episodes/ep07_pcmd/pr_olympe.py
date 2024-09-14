#!/home/legatus/parrot/bin/python

import sys
import rospy
from loguru import logger
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import olympe
import os
import time
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD

DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")

class OlympeBridge():
    def __init__(self):
        rospy.init_node("olympe_node", anonymous=False)
        self.cmd_msg_sub = rospy.Subscriber("/parrot/cmd_status", String, self._cmd_msg_callback)
        self.cmd_vel_sub = rospy.Subscriber("/parrot/cmd_vel", Twist, self._cmd_vel_callback)
       
        self.drone = olympe.Drone(DRONE_IP)

        self.cmd_vel = Twist()

    def connect_parrot(self):
        logger.info("Connecting to {}".format(DRONE_IP))
        self.drone.connect()
        self.isDroneConnected = self.drone.connected

        if not self.isDroneConnected:
            logger.error("Could not connect to the drone at {}".format(DRONE_IP))
            exit(0)

        else:
            logger.success("Connected to the drone at {}".format(DRONE_IP))


    def _cmd_vel_callback(self, msg: Twist):
        self.cmd_vel.linear.x = msg.linear.x
        self.cmd_vel.linear.y = msg.linear.y
        self.cmd_vel.linear.z = msg.linear.z

        self.cmd_vel.angular.z = msg.angular.z

        logger.info("cmd_vel received!")

        self.drone(
            PCMD(1,
                int(-self.cmd_vel.linear.y),
                int(self.cmd_vel.linear.x),
                int(-self.cmd_vel.angular.z),
                int(self.cmd_vel.linear.z),
                timestampAndSeqNum = 0 
            )
        )


    def _cmd_msg_callback(self, msg):
        self.cmd_msg = msg.data
        if self.cmd_msg == "takeoff":
            assert self.drone(TakeOff()).wait().success()
            logger.success("Takeoff successful!")

        elif self.cmd_msg == "land":
            assert self.drone(Landing()).wait().success()
            logger.success("Landing successful!")

def main(args):
    olympe_bridge = OlympeBridge()
    olympe_bridge.connect_parrot()

    if olympe_bridge.isDroneConnected:
        rospy.spin()


if __name__ == "__main__":
    main(sys.argv)