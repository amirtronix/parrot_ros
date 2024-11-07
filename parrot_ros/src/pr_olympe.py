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
from loguru import logger

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
        self.isDroneConnected = False


        self.cmd_msg_sub = rospy.Subscriber("/parrot/cmd_status", String, self._cmd_msg_callback)
        self.cmd_vel_sub = rospy.Subscriber("/parrot/cmd_vel", Twist, self._cmd_vel_callback)
       
        self.image_pub = rospy.Publisher("/parrot/image_raw",Image, queue_size=10)


        self.bridge = CvBridge()
        self.cv2_cvt_color_flag = {
                    olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                    olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
                }
        
        self.rate = rospy.Rate(10)

        self.cmd_vel = Twist()
        self.control = False

    def connect_parrot(self):
        logger.info("Connecting to {}".format(DRONE_IP))
        self.drone.connect()
        self.isDroneConnected = self.drone.connected

        if not self.isDroneConnected:
            logger.error("Could not connect to the drone at {}".format(DRONE_IP))
            exit(0)

        else:
            logger.success("Connected to the drone at {}".format(DRONE_IP))


    def start(self):
        if self.isDroneConnected:
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
            logger.success("Streaming initiated!")
        
    def stop(self):
        self.running = False
        self.processing_thread.join()
        if self.renderer is not None:
            self.renderer.stop()
        # Properly stop the video stream and disconnect
        assert self.drone.streaming.stop()
        assert self.drone.disconnect()
        self.h264_stats_file.close()

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

                

            except queue.Empty:
                continue
            # You should process your frames here and release (unref) them when you're done.
            # Don't hold a reference on your frames for too long to avoid memory leaks and/or memory
            # pool exhaustion.
            yuv_frame.unref()

    def _cmd_vel_callback(self, msg):
        self.cmd_vel.linear.x = msg.linear.x
        self.cmd_vel.linear.y = msg.linear.y
        self.cmd_vel.linear.z = msg.linear.z

        self.cmd_vel.angular.z = msg.angular.z

        logger.info("cmd_vel received!")

    def _cmd_msg_callback(self, msg):
        self.cmd_msg = msg.data
        if self.cmd_msg == "takeoff":
            assert self.drone(TakeOff()).wait().success()
            logger.success("Takeoff successful!")

        elif self.cmd_msg == "land":
            assert self.drone(Landing()).wait().success()
            logger.success("Landing successful!")

    def run(self):
        self.drone(
            PCMD(1,
                int(-self.cmd_vel.linear.y),
                int(self.cmd_vel.linear.x),
                int(-self.cmd_vel.angular.z),
                int(self.cmd_vel.linear.z),
                timestampAndSeqNum = 0 
            )
        )
        self.control  = False

        time.sleep(0.05)

    
def main(args):

    olympe_bridge = OlympeBridge()
    olympe_bridge.connect_parrot()
    olympe_bridge.start()

    try:
        logger.info("in loop")
        while not rospy.is_shutdown():

            olympe_bridge.run()
            olympe_bridge.rate.sleep()


    except KeyboardInterrupt:
        print("\nShutting down")
        cv2.destroyAllWindows()

    finally:
        olympe_bridge.stop()


    

if __name__ == "__main__":
    main(sys.argv)

