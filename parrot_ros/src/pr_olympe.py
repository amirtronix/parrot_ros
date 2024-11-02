#!/home/praetor/parrot/bin/python

import os
import sys
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import csv
import math
import os
import queue
import shlex
import subprocess
import tempfile
import threading

from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
import signal


import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD


DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")

class OlympeBridge():

    def __init__(self):
        rospy.init_node('olympe_bridge_node', anonymous=False)

        self.drone = olympe.Drone(DRONE_IP)
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        
        self.h264_frame_stats = []
        self.h264_stats_file = open(os.path.join(self.tempd, "h264_stats.csv"), "w+")
        self.h264_stats_writer = csv.DictWriter(self.h264_stats_file, ["fps", "bitrate"])
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.yuv_frame_processing)
        self.renderer = None
        self.hoveringStatus = False


        rospy.Subscriber('/drone/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/drone/takeoff', String, self.takeoff_callback)
        rospy.Subscriber('/drone/land', String, self.land_callback)
        self.image_pub = rospy.Publisher("/drone/image_raw",Image, queue_size=10)


        self.bridge = CvBridge()
        self.cv2_cvt_color_flag = {
                    olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                    olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
                }
        
        self.rate = rospy.Rate(10)

        self.cmd_vel = Twist()
        self.control = False

    def start(self):
        # Connect the the drone
        assert self.drone.connect(retry=3)

        if DRONE_RTSP_PORT is not None:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"

        assert self.drone(TakeOff()).wait().success()
        self.hoveringStatus = True

        # Setup your callback functions to do some live video processing
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
        )
        # Start video streaming
        self.drone.streaming.start()
        # self.renderer = PdrawRenderer(pdraw=self.drone.streaming)
        self.running = True
        self.processing_thread.start()
    
    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.

            :type yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def yuv_frame_processing(self):
        while self.running:
            try:
                yuv_frame = self.frame_queue.get(timeout=0.1)
                x = yuv_frame.as_ndarray()
                [yuv_frame.format()]
                cv2frame = cv2.cvtColor(x, self.cv2_cvt_color_flag[yuv_frame.format()]) 
                cv2.imwrite("/home/accurpress/catkin_ws/src/deep_ros/repo/frames/img_sphinx.jpg", cv2frame)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv2frame, "bgr8"))
                if self.hoveringStatus:
                    print(cv2frame.shape)

                

            except queue.Empty:
                continue
            # You should process your frames here and release (unref) them when you're done.
            # Don't hold a reference on your frames for too long to avoid memory leaks and/or memory
            # pool exhaustion.
            yuv_frame.unref()

    def cmd_vel_callback(self, msg):
        self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.linear.z = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.cmd_vel.angular.x, self.cmd_vel.angular.y, self.cmd_vel.angular.z = [msg.angular.x, msg.angular.y, msg.angular.z]

    def takeoff_callback(self, msg):
        print("Asserting Takeoff!")
        assert self.drone(TakeOff()).wait().success()
        print("Takeoff Successful!")
        self.hoveringStatus = True

    def land_callback(self, msg):
        print("Asserting Landing!")
        assert self.drone(Landing()).wait().success()
        print("Landing Successful!")
        self.hoveringStatus = False


    def run(self):

        self.drone(
            PCMD(
                1,
                int(self.cmd_vel.linear.x),
                int(self.cmd_vel.linear.y),
                int(-self.cmd_vel.angular.z),
                int(self.cmd_vel.linear.z),
                timestampAndSeqNum=0,
            )
        )
        self.control  = False

        if self.hoveringStatus:
            print("x_cmd = ", int(self.cmd_vel.linear.x), end = ' ')
            print("y_cmd = ", int(self.cmd_vel.linear.y), end = ' ')
            print("z_cmd = ", int(self.cmd_vel.linear.z), end = ' ')
            print("Yaw_cmd = ", -int(self.cmd_vel.angular.z))

        time.sleep(0.05)

def main(args):

    olympe_bridge = OlympeBridge()
    olympe_bridge.start()

    while not rospy.is_shutdown():

        olympe_bridge.run()
        olympe_bridge.rate.sleep()

    olympe_bridge.drone(Landing())
    print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)

