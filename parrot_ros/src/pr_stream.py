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

rospack = rospkg.RosPack()
config_path = rospack.get_path('parrot_ros') + '/config/param.yaml'

print(config_path)

with open(config_path, 'r') as stream:
    config = yaml.safe_load(stream)

IMAGE_WIDTH = config['image_width']
IMAGE_HEIGHT = config['image_height']
ROS_RATE = config['ros_rate']
CAMERA_TOPIC = config['camera_topic']



class VideoStream():
    def __init__(self, save_video=False, save_frame=False):
        rospy.init_node('sphinx_stream_node')
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.image_callback)
        self.save_video = save_video
        self.save_frame = save_frame
        self.current_time = datetime.datetime.now()
        self.formatted_time = self.current_time.strftime("%Y-%m-%dT%H:%M:%S")

        self.rate = rospy.Rate(ROS_RATE)
        
        video_title  = f"{'video'}_{self.formatted_time}.{'mp4'}"
        
        if self.save_video:
            self.video_path = rospack.get_path('parrot_ros') + "/repo/videos/" + video_title
            self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(self.video_path, self.fourcc, float(ROS_RATE), (IMAGE_WIDTH, IMAGE_HEIGHT))

        if self.save_frame:
            frame_dir = f"{self.formatted_time}"
            self.frame_path = rospack.get_path('parrot_ros') + "/repo/frames/" + frame_dir
            if not os.path.exists(self.frame_path):
                
                os.makedirs(self.frame_path)
                os.chmod(self.frame_path, 0o755)
                
                print(f"Directory '{self.frame_path}' created successfully.")
            else:
                print(f"Directory '{self.frame_path}' already exists.")
                
        self.frame_count = 0

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(e)
            return
        
        cv2.imshow("Image Subscriber", self.cv_image)
        cv2.waitKey(1)

        if self.save_video:
            self.saveVideo()

        if self.save_frame:
            self.saveFrame()

        logger.info(f"image count: {self.frame_count}")

        self.rate.sleep()
        
        
    def run(self):
        pass

    def saveFrame(self):
        frame_title  = f"{'img'}_{self.frame_count:04d}.{'jpg'}"
        cv2.imwrite(os.path.join(self.frame_path, frame_title), self.cv_image)

        self.frame_count += 1

    def saveVideo(self):
        self.video_writer.write(self.cv_image)


    def __del__(self):
        if self.save_video:
            self.video_writer.release()


def main(args):
    video_Stream = VideoStream(save_video= False, save_frame=False)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
