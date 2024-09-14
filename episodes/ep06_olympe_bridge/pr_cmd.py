#!/home/legatus/parrot/bin/python

import sys
import rospy
from loguru import logger

from std_msgs.msg import String

class Cmd():
    def __init__(self):
        rospy.init_node("cmd_node", anonymous=False)
        # self.cmd_pub = rospy.Publisher("/parrot/cmd_vel", Int16, queue_size=10)
        self.cmd_msg_pub = rospy.Publisher("/parrot/cmd_status", String, queue_size=10)

        self.rate = rospy.Rate(10)
        self.cmd_msg = String()

        self.cmd_list = ["takeoff", "land"]

    def user_prompt(self):
        logger.info("Enter flight command [takeoff] or [land] or [exit]")
        input_msg = input()

        if input_msg == "exit":
            exit(0)

        elif not input_msg in str(self.cmd_list):
            logger.error("Invalid input, try again!")
            self.user_prompt()

        else:
            self.publish(input_msg)

    def publish(self, input_msg):
        self.cmd_msg.data = input_msg
        self.cmd_msg_pub.publish(self.cmd_msg)
        logger.success("{} command published".format(input_msg))

def main(args):
    cmd_node = Cmd()

    while not rospy.is_shutdown():
        cmd_node.user_prompt()
        


if __name__ == "__main__":
    main(sys.argv)