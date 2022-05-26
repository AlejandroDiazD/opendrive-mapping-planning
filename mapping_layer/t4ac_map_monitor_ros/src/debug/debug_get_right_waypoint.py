"""
Debug file for checking the waypoint method get_right_waypoint()

There is a subscriber for /initialpose so the central waypoint can be
selected in RVIZ using the 2D Pose Estimate. After selecting that central
waypoint, right and left are plotted if they exist.

The workflow to debug using this file is:
    - Launch a roscore
    - Launch RVIZ
    - Launch the map_visualizator
    - Run this file
    - Select a waypoint using 2D Pose Estimate button in RVIZ
    - Visualize results in RVIZ
"""

import sys
import os

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from t4ac_mapping_layer.map_parser.map_object import MapObject
from modules import markers_module

# Create ROS marker and config ROS parameters
rospy.init_node("debugger_node", anonymous=True)
rate = rospy.Rate(1)

# Init the MapObject
map_name = rospy.get_param('t4ac/mapping/map_name') 
map_path = rospy.get_param("t4ac/mapping/map_path")
map_object = MapObject(map_name, map_path)

def initialpose_callback(pose):
    """
    Receives pose and calculates right and left waypoints if exist
    """
    right_waypoint, right_waypoint_marker = None, None
    left_waypoint, left_waypoint_marker = None, None
    central_waypoint = map_object.get_waypoint(pose.pose.pose.position.x,
                                               pose.pose.pose.position.y,
                                               pose.pose.pose.position.z)
    right_waypoint = central_waypoint.get_closer_right_wp(map_object.map_waypoints)
    left_waypoint = central_waypoint.get_closer_left_wp(map_object.map_waypoints)

    central_waypoint_marker = markers_module.get_waypoint(
        central_waypoint, [0,1,0], -1, 1)
    if right_waypoint:
        right_waypoint_marker = markers_module.get_waypoint(
            right_waypoint, [1,0,0], -1, 1)
    if left_waypoint:
        left_waypoint_marker = markers_module.get_waypoint(
            left_waypoint, [0,0,1], -1, 1)

    central_waypoint_pub.publish(central_waypoint_marker)
    if right_waypoint_marker:
        right_waypoint_pub.publish(right_waypoint_marker)
    if left_waypoint_marker:
        left_waypoint_pub.publish(left_waypoint_marker)
    

initialpose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialpose_callback)

central_waypoint_pub = rospy.Publisher(
    "/t4ac/mapping/debug/central_waypoint", Marker, queue_size=1)
right_waypoint_pub = rospy.Publisher(
    "/t4ac/mapping/debug/right_waypoint", Marker, queue_size=1)
left_waypoint_pub = rospy.Publisher(
    "/t4ac/mapping/debug/left_waypoint", Marker, queue_size=1)


if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass