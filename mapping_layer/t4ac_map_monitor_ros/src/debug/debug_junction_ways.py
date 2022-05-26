"""
File to debug the junction ways function of the map_monitor plotting in RVIZ
the calculated junction ways of an specific junction selectec by its id.

The target functions to be debugged are:
    * calculate_junction_ways()

The workflow to debug using this file is:
    - Launch a roscore
    - Launch RVIZ
    - Launch the map_visualizator
    - Edit this file to select the junction id
    - Run this file
    - Visualize results in RVIZ
"""

import sys
import os

import rospy
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
from t4ac_mapping_layer.map_parser.map_object import MapObject

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from modules.junctions_module import calculate_junction_ways
from modules.markers_module import get_way

# Create ROS marker and config ROS parameters
rospy.init_node("debugger_node", anonymous=True)
rate = rospy.Rate(1)

ways_marker_pub = rospy.Publisher(
    "/t4ac/debug/intersection_ways", MarkerArray, queue_size=1)

map_name = rospy.get_param('t4ac/mapping/map_name') 
map_path = rospy.get_param("t4ac/mapping/map_path")
map_object = MapObject(map_name, map_path)

# ================================ #
# Edit here the road_id and lane_id:
junction = map_object.get_junction_by_id(1736)
# ================================ #

junction_ways = calculate_junction_ways(junction, map_object)

ways_marker = MarkerArray()
for i, way in enumerate(junction_ways):
    way_marker = get_way(way, [0,1,1], 0.2, 0.2, i)
    ways_marker.markers.append(way_marker)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        ways_marker_pub.publish(ways_marker)
        rate.sleep()





















intersections_pub = rospy.Publisher("debugging/intersections", Marker, queue_size=1)
rospy.init_node("intersections_debugger_node", anonymous=True)
rate = rospy.Rate(1)


print(">>> OK")

while not rospy.is_shutdown():
    route_pub.publish(route_marker)
    rate.sleep()