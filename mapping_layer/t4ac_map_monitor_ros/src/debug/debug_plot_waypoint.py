"""
Debug file for ploting a waypoint in RVIZ

The workflow to debug using this file is:
    - Launch a roscore
    - Launch RVIZ
    - Launch the map_visualizator
    - Edit this file to select the waypoint location
    - Run this file
    - Visualize results in RVIZ
"""

import sys
import os

import rospy
from visualization_msgs.msg import Marker

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from t4ac_mapping_layer.map_parser.builder_classes import T4ac_Location
from t4ac_mapping_layer.map_parser.builder_classes import T4ac_Waypoint
from modules import markers_module

# ================================ #
# Edit here xyz location:
location = T4ac_Location(88.76, -33.64, 0.0)
waypoint = T4ac_Waypoint(location)
# ================================ #

# Create ROS marker and config ROS parameters
rospy.init_node("debugger_node", anonymous=True)
rate = rospy.Rate(1)

waypoint_marker = markers_module.get_waypoint(
    waypoint, [1,0,0], -1, 1)

waypoint_pub = rospy.Publisher(
    "/t4ac/mapping/debug/waypoint", Marker, queue_size=1)

while not rospy.is_shutdown():
    waypoint_pub.publish(waypoint_marker)
    rate.sleep()