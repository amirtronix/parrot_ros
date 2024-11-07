#!/home/praetor/parrot/bin/python

import sys
import rospy
import rospkg
import yaml
import datetime
import os
from loguru import logger

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import torch
from pathlib import Path
import matplotlib.pyplot as plt
from ultralytics import YOLO

rospack = rospkg.RosPack()
config_path = rospack.get_path('parrot_ros') + '/config/param.yaml'

print(config_path)

with open(config_path, 'r') as stream:
    config = yaml.safe_load(stream)

IMAGE_WIDTH = config['image_width']
IMAGE_HEIGHT = config['image_height']
ROS_RATE = config['ros_rate']
CAMERA_TOPIC = config['camera_topic']



class Detector():
    def __init__(self, save_video=False, save_frame=False):
        rospy.init_node('sphinx_stream_node')
        
        self.image_pub = rospy.Publisher("/parrot/image_annotated",Image, queue_size=10)

        self.model_yolo = YOLO("/home/praetor/catkin_ws/src/parrot_ros/parrot_ros/src/yolov8n.pt")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.image_callback)
        self.save_video = save_video
        self.save_frame = save_frame
        self.current_time = datetime.datetime.now()
        self.formatted_time = self.current_time.strftime("%Y-%m-%dT%H:%M:%S")

        
        self.frame_count = 0
        self.rate = rospy.Rate(ROS_RATE)

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.cv_image = cv2.resize(img, (0,0), fx=0.75, fy=0.75)
        except Exception as e:
            print(e)
            return
        
        self.predict()
        self.annotate()

        self.frame_count += 1

        
        
        logger.info(f"image count: {self.frame_count}")
        self.rate.sleep()
        
        
    def run(self):
        pass

    def predict(self):
        self.results =  self.model_yolo(self.cv_image, verbose=False)


    def annotate(self):
        img = self.results[0].plot()  # This plots the detections on the image
        cv2.imshow('Screen', img)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        cv2.waitKey(3)

    def __del__(self):
        pass


def main(args):
    detector = Detector(save_video= True, save_frame=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)