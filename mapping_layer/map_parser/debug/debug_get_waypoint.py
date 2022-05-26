"""
Debug file for function get_waypoint() from MapObject
"""

import sys
import os

import rospy
from visualization_msgs.msg import Marker

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from map_object import MapObject

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from t4ac_mapping_layer.t4ac_map_monitor_ros.src.modules  import markers_module

map_path = '/workspace/team_code/catkin_ws/src/t4ac_mapping_layer/maps/xodr/'
map_object = MapObject('Town01', map_path)

waypoint = map_object.get_waypoint(87.96, -89.14, 0)

# Create ROS marker and config ROS parameters
rospy.init_node("debugger_node", anonymous=True)
rate = rospy.Rate(1)

waypoint_marker = markers_module.get_waypoint(
    waypoint, [1,1,0], -1, 1)

waypoint_pub = rospy.Publisher(
    "/t4ac/mapping/debug/central_waypoint", Marker, queue_size=1)

while not rospy.is_shutdown():
    waypoint_pub.publish(waypoint_marker)
    rate.sleep()