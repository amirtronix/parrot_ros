#!/home/legatus/parrot/bin/python

from re import A
from signal import SIG_DFL
import roslib
import rospy
import sys
import rospkg
import yaml

import tf
import turtlesim.msg
import pysphinx

rospack = rospkg.RosPack()
config_path = rospack.get_path('parrot_ros') + '/config/param.yaml'

print(config_path)

with open(config_path, 'r') as stream:
    config = yaml.safe_load(stream)

ROS_RATE = config['ros_rate']


class SphinxBridge():
    def __init__(self):
        rospy.init_node('sphinx_broadcaster')
        self.object_name = rospy.get_param('~object')
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(ROS_RATE)
        self.connection_status = False

        self.pose_dic = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            
        }
        
        self.dronePose = dict(self.pose_dic)
        self.truckPose = dict(self.pose_dic)

    def connect(self):
        self.sphinx = pysphinx.Sphinx(ip="127.0.0.1", port=8383)

    def getDronePose(self):
        try:
            comp = self.sphinx.get_component('anafi_ai', 'omniscient', 'tools')
            tlm = comp.get_tlm()
            tlm.fetch_sample()

            self.dronePose['x'] =  tlm['worldPosition.x']
            self.dronePose['y'] = tlm['worldPosition.y']
            self.dronePose['z'] = tlm['worldPosition.z']
            self.dronePose['roll'] = tlm["worldAttitude.x"]
            self.dronePose['pitch'] = tlm["worldAttitude.y"]
            self.dronePose['yaw'] = tlm["worldAttitude.z"]
            print(self.dronePose['x'], self.dronePose['y'])
            rospy.loginfo("Drone pose published!")

        except:
            rospy.logerr("Truck pose not published!")


    def getTruckPose(self):
        try:
            self.sphinx_dict = self.sphinx.get_info()
            config_data = self.sphinx_dict['world']['components']
            for config_item in config_data:
                for spawn_point in config_item['spawn_points']:
                    if spawn_point['name'] == 'pickup':
                        truck_dict = spawn_point['pose']
                        print(spawn_point['pose'])

                        self.truckPose['x'] =  truck_dict['x']
                        self.truckPose['y'] = truck_dict['y']
                        self.truckPose['z'] = truck_dict['z']
                        self.truckPose['roll'] = truck_dict['roll']
                        self.truckPose['pitch'] = truck_dict['pitch']
                        self.truckPose['yaw'] = truck_dict['yaw']
                        rospy.loginfo("Turck pose published!")

        except:
            pass

    def broadcastDronePose(self):
        self.br.sendTransform((self.dronePose['x'], self.dronePose['y'], self.dronePose['z']),
                tf.transformations.quaternion_from_euler(self.dronePose['roll'], self.dronePose['pitch'], self.dronePose['yaw']),
                rospy.Time.now(),
                "vicon/parrot/parrot",
                "map")
        
    def broadcastTruckPose(self):
        self.br.sendTransform((self.truckPose['x'], self.truckPose['y'], self.truckPose['z']),
                tf.transformations.quaternion_from_euler(self.truckPose['roll'], self.truckPose['pitch'], self.truckPose['yaw']),
                rospy.Time.now(),
                "vicon/truck/truck",
                "map")

def main(args):

    sphinx_bridge = SphinxBridge()
    sphinx_bridge.connect()

    while not rospy.is_shutdown():

        if sphinx_bridge.object_name == "drone":
            sphinx_bridge.getDronePose()
            sphinx_bridge.broadcastDronePose()

        elif sphinx_bridge.object_name == "truck":
            sphinx_bridge.getTruckPose()
            sphinx_bridge.broadcastTruckPose()

        else:
            rospy.logerr("Invalid object name!")
            
        sphinx_bridge.rate.sleep()


if __name__ == "__main__":
    main(sys.argv)
