#!/home/legatus/parrot/bin/python

import sys
import rospy
import rospkg
from geometry_msgs.msg import Pose, Twist
import yaml
import os
from loguru import logger
from tf.transformations import quaternion_from_euler

rospack = rospkg.RosPack()
config_path = rospack.get_path('parrot_ros') + '/config/param.yaml'

print(config_path)

with open(config_path, 'r') as stream:
    config = yaml.safe_load(stream)

ROS_RATE = config['ros_rate']


class PulseGenerator:
    def __init__(self, pulse_width, amplitude, offset = 0):
        
        self._pulse_width = pulse_width
        self._amplitude = amplitude
        self._offset = offset


    def _even_odd(self, num):
        
        if(int(num)%2 == 0):
            return 1.0
        
        else:
            return -1.0
    

    def generate(self, time):
        
        signal = self._even_odd(time/self._pulse_width)*self._amplitude + self._offset
        return signal


class PulseNode():
    def __init__(self):
        rospy.init_node('pulse_generator')

        self.pulse_mode = rospy.get_param("~pulse_mode")
        self.axis = rospy.get_param("~axis")
        self.pulse_amp = rospy.get_param("~pulse_amp")
        self.pulse_width = rospy.get_param("~pulse_width")
        self.pulse_offset = rospy.get_param("~pulse_offset")

        self.pose_signal = Pose() # Command Pose
        self.twist_signal = Twist() # Command Velocity

        self.pose_publisher = rospy.Publisher('/drone/cmd_pos', Pose, queue_size = 10)
        self.twist_publisher = rospy.Publisher('/drone/cmd_vel', Twist, queue_size = 10)

        self.pulse_generator = PulseGenerator(self.pulse_width, self.pulse_amp, self.pulse_offset)


    def _get_pulse(self, secs):
        signal = self.pulse_generator.generate(secs)
        print(signal)

    def _get_msg(self, signal):
        if self.pulse_mode == "cmd_pos":
            if self.axis == "x":
                self.pose_signal.position.x = signal
            elif self.axis == "y":
                self.pose_signal.position.y = signal
            elif self.axis == "z":
                self.pose_signal.position.z = signal
            elif self.axis == "yaw":
                self.pose_signal.orientation = quaternion_from_euler(0, 0, signal)

            print(self.pose_signal)
            return self.pose_signal

        elif self.pulse_mode == "cmd_vel":
            if self.axis == "x":
                self.twist_signal.linear.x = signal
            elif self.axis == "y":
                self.twist_signal.linear.y = signal
            elif self.axis == "z":
                self.twist_signal.linear.z = signal
            elif self.axis == "yaw":
                self.twist_signal.angular.z = signal

            print(self.twist_signal)
            return self.twist_signal
        else:
            rospy.logerr("Incorrect Pulse Mode!")
            return None

    def _publish(self, signal):
        msg = self._get_msg(signal)

        if (self.pulse_mode == "cmd_pos") and (msg is not None):
            self.pose_publisher.publish(msg)
        elif (self.pulse_mode == "cmd_vel") and (msg is not None):
            self.twist_publisher.publish(msg)

    def process(self, _t):
        signal = self.pulse_generator.generate(_t)
        self._publish(signal)

def main(args):

    pulse_node = PulseNode()
    rate = rospy.Rate(ROS_RATE)

    while not rospy.is_shutdown():

        # t is Epoch time 1-1-1970
        t = rospy.get_time()
        pulse_node.process(t)

        rate.sleep()



if __name__ == "__main__":
    main(sys.argv)
