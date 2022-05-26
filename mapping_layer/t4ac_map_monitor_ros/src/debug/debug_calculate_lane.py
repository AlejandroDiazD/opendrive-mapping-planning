"""
Debug file that calculates a lane and its central way and visualizes 
their markers in RVIZ

The workflow to debug using this file is:
    - Launch a roscore
    - Launch RVIZ
    - Launch the map_visualizator
    - Edit this file to set the target road and lane id
    - Run this file
    - Visualize results in RVIZ
"""

import sys
import os

import rospy
from visualization_msgs.msg import Marker

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from t4ac_mapping_layer.map_parser.map_object import MapObject
from modules import markers_module
from modules import lanes_module
from modules import monitor_module

# Create ROS marker and config ROS parameters
rospy.init_node("debugger_node", anonymous=True)
rate = rospy.Rate(2)

lane_marker_pub = rospy.Publisher(
    "/t4ac/mapping/debug/lane", Marker, queue_size=2)
way_marker_pub = rospy.Publisher(
    "/t4ac/mapping/debug/way", Marker, queue_size=2)

# Init the MapObject
map_name = rospy.get_param('t4ac/mapping/map_name') 
map_path = rospy.get_param("t4ac/mapping/map_path")
map_object = MapObject(map_name, map_path)

# ================================ #
# Edit here the road_id and lane_id:
road = map_object.get_road_by_id(1354)
lane = map_object.get_lane_by_id(1354, 1, 0)
# ================================ #

central_way = map_object.generate_waypoints_in_lane(road, lane, 3)
central_way_marker = markers_module.get_way(central_way, [1, 1, 0], -1, 0.2, 4)
lane_monitor = monitor_module.calulate_lane(central_way)
lane_marker_right, lane_marker_left = markers_module.get_lane(lane_monitor, [1, 0, 0], "lane_marker", 4, 0.2, 0, -1)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        way_marker_pub.publish(central_way_marker)
        lane_marker_pub.publish(lane_marker_right)
        lane_marker_pub.publish(lane_marker_left)
        rate.sleep()