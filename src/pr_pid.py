#!/home/username/parrot/bin/python

import sys
import rospy
import rospkg
from geometry_msgs.msg import Pose, Twist
import tf

import threading
import cv2
import os
from loguru import logger
import yaml

rospack = rospkg.RosPack()
config_path = rospack.get_path('parrot_ros') + '/config/param.yaml'

print(config_path)

with open(config_path, 'r') as stream:
    config = yaml.safe_load(stream)

ROS_RATE = config['ros_rate']

class Gains:
    def __init__(self):
        self.kp = 0
        self.ki = 0
        self.kd = 0

class PidController:
    def __init__(self, _gains, _sample_time, _anti_windup=None):
        
        self.gains = Gains()
        self._setGains(_gains)
        self._ts = _sample_time
        self._sum = 0
        self._prev_error = 0
        self._antiWindupActivation = _anti_windup


    def _setGains(self, _gains_list):
        self.gains.kp = _gains_list[0]
        self.gains.ki = _gains_list[1]
        self.gains.kd = _gains_list[2]

    def _antiWindup(self, val):
        if val > 50:
            return 50
        elif val < -50:
            return -50
        else:
            return val


    def _cmd_sat(self, val):
        if val > 100:
            return 100
        elif val < -100:
            return -100
        else:
            return val

    def _integral(self, error):
        self._sum = self._sum + error*self._ts

        if self._antiWindupActivation is None:
            return self._sum
        else:
            return self._antiWindup(self._sum)

    def _derivateive(self, error):
        derivative = (error-self._prev_error)/self._ts
        self._prev_error = error
        return 0
    
    def compute(self, error):
        _contorlSignal = self.gains.kp*error + \
            self.gains.ki*self._integral(error) + \
            self.gains.kd*self._derivateive(error)
        return self._cmd_sat(_contorlSignal)

class FlightController():
    def __init__(self):
        rospy.init_node('flight_controller')

        rospy.Subscriber('/drone/cmd_pos', Pose, self._cmd_pos_callback)
        self.cmd_pub = rospy.Publisher("/drone/cmd_vel",Twist, queue_size=10)

        self.sample_time = 1/ROS_RATE

        # self.gains = [[4200,62,3600], [2100,60,2200], [1650,1477,32], [4000,70,100]]
        self.gains = [[0,0,0], [0,0,0], [1650,1477,32], [0,0,0]]

        self.gui_param = GuiThread(self.gains)
        self.gui_param.start()

        self.cmd_zero = Twist()
        self.cmd_vel = Twist()
        self.cmd_pos = Pose()
        self.drone_pos = Pose()

        

        self.pid_x = PidController(self.gains[0], self.sample_time, True)
        self.pid_y = PidController(self.gains[1], self.sample_time, True)
        self.pid_z = PidController(self.gains[2], self.sample_time, True)
        self.pid_yaw = PidController(self.gains[3], self.sample_time, True)

    def generate_cmd(self, trans, rot):
        if self._is_topic("/drone/cmd_pos"):
            self._get_pos(trans, rot)
            errors = self._get_error()

            self.gui_param.update_gains(self.pid_x.gains,
                                        self.pid_y.gains,
                                        self.pid_z.gains,
                                        self.pid_yaw.gains)
            
            self.cmd_vel.linear.x = self.pid_x.compute(errors[0])
            self.cmd_vel.linear.y = self.pid_y.compute(errors[1])
            self.cmd_vel.linear.z = self.pid_z.compute(errors[2])
            self.cmd_vel.angular.z = self.pid_yaw.compute(errors[3])

            self.cmd_pub.publish(self.cmd_vel)

        else:
            self.cmd_pub.publish(self.cmd_zero)

    def _get_pos(self, trans, rot):
        self.drone_pos.position.x = trans[0]
        self.drone_pos.position.x = trans[1]
        self.drone_pos.position.x = trans[2]

        self.drone_pos.orientation.x = rot[0]
        self.drone_pos.orientation.y = rot[1]
        self.drone_pos.orientation.z = rot[2]
        self.drone_pos.orientation.w = rot[3]

    def _cmd_pos_callback(self, msg: Pose):
        self.cmd_pos.position = msg.position
        self.cmd_pos.orientation = msg.orientation

    def _get_error(self):
        error_x = self.cmd_pos.position.x - self.drone_pos.position.x
        error_y = self.cmd_pos.position.y - self.drone_pos.position.y
        error_z = self.cmd_pos.position.z - self.drone_pos.position.z

        quaternion_cmd = (
            self.cmd_pos.orientation.x,
            self.cmd_pos.orientation.y,
            self.cmd_pos.orientation.z,
            self.cmd_pos.orientation.w
        )

        quaternion_drone = (
            self.drone_pos.orientation.x,
            self.drone_pos.orientation.y,
            self.drone_pos.orientation.z,
            self.drone_pos.orientation.w
        )

        yaw_cmd = tf.transformations.euler_from_quaternion(quaternion_cmd)[2]
        yaw_drone = tf.transformations.euler_from_quaternion(quaternion_drone)[2]
        error_yaw = yaw_cmd - yaw_drone

        return [error_x, error_y, error_z, error_yaw]

    def _is_topic(slef, topic_name):
        topics = rospy.get_published_topics()

        for topic, topic_type in topics:
            if topic == topic_name:
                return True

        return False

class GuiThread(threading.Thread):
    def __init__(self, _init_gains, _title = "PID GUI", _max = 100):
        super(GuiThread, self).__init__()

        self.title_window = _title
        self.max = _max
        self.init_gains = _init_gains

        self.row_titles= ["kp (x)", "ki (x)", "kd (x)",
                          "kp (y)", "ki (y)", "kd (y)",
                          "kp (z)", "ki (z)", "kd (z)",
                          "kp (yaw)", "ki (yaw)", "kd (yaw)"]
        1
        self.row_max = [10000, 100000, 10000,
                        10000, 100000, 10000,
                        10000, 100000, 10000,
                        10000, 100000, 10000]
        
        self.kp_factor = 100
        self.ki_factor = 1000
        self.kd_factor = 100


    def update_gains(self, Gx: Gains, Gy: Gains, Gz: Gains, Gyaw: Gains):
        Gx.kp = cv2.getTrackbarPos(self.row_titles[0], self.title_window)/self.kp_factor
        Gx.ki = cv2.getTrackbarPos(self.row_titles[1], self.title_window)/self.ki_factor
        Gx.kd = cv2.getTrackbarPos(self.row_titles[2], self.title_window)/self.kd_factor

        Gy.kp = cv2.getTrackbarPos(self.row_titles[3], self.title_window)/self.kp_factor
        Gy.ki = cv2.getTrackbarPos(self.row_titles[4], self.title_window)/self.ki_factor
        Gy.kd = cv2.getTrackbarPos(self.row_titles[5], self.title_window)/self.kd_factor

        Gz.kp = cv2.getTrackbarPos(self.row_titles[6], self.title_window)/self.kp_factor
        Gz.ki = cv2.getTrackbarPos(self.row_titles[7], self.title_window)/self.ki_factor
        Gz.kd = cv2.getTrackbarPos(self.row_titles[8], self.title_window)/self.kd_factor

        Gyaw.kp = cv2.getTrackbarPos(self.row_titles[9], self.title_window)/self.kp_factor
        Gyaw.ki = cv2.getTrackbarPos(self.row_titles[10], self.title_window)/self.ki_factor
        Gyaw.kd = cv2.getTrackbarPos(self.row_titles[11], self.title_window)/self.kd_factor


    def run(self):
        cv2.namedWindow(self.title_window)

        for index, (trackbar_name, trackbar_max) in enumerate(zip(self.row_titles, self.row_max)):
            row_index = index // 3
            col_index = index % 3

            print(col_index, row_index)
            
            cv2.createTrackbar(trackbar_name, 
                               self.title_window , 
                               self.init_gains[row_index][col_index], 
                               trackbar_max, 
                               self.on_trackbar)

        while not rospy.is_shutdown():
            cv2.waitKey(3)
            
    def on_trackbar(self, value):
        pass

    def __del__(self):
        cv2.destroyAllWindows()


def main(args):

    flight_controller = FlightController()
    listener = tf.TransformListener()
    drone_pose = Pose()

    rate = rospy.Rate(ROS_RATE)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('map', '/vicon/parrot/parrot', rospy.Time(0))
            t = rospy.get_time()
            flight_controller.generate_cmd(trans, rot)

            rospy.loginfo("Command published!")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # rospy.logerr("No transform found!")
            continue



        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
