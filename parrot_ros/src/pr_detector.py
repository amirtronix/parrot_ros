#!/home/praetor/parrot/bin/python

import sys
import rospy
import rospkg
import yaml
import datetime
import os
from loguru import logger
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


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

        self.logger_time = 0

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.cv_image = cv2.resize(img, (0,0), fx=0.4, fy=0.4)
        except Exception as e:
            print(e)
            return
        
        self.predict()
        self.annotate()

        self.frame_count += 1

        
        if (time.process_time() - self.logger_time) > 0.5:
            logger.success(f"Image processed: {self.frame_count}")
            for img_result in self.results:
                for box_item in img_result.boxes.cpu().numpy():
                    item_class = int(box_item.cls)
                    print("Detected Item: ", img_result.names[item_class], box_item.xyxy)
            
            self.logger_time = time.process_time()
        
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