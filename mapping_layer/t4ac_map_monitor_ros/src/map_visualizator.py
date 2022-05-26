#!/usr/bin/env python3
"""
Map Visualizator without using PythonAPI

Last mod: Alejandro D. 4/5/22
"""

import sys 
import os

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import String

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from t4ac_mapping_layer.map_parser.map_object import MapObject

from modules import markers_module


map_visualizator_pub = rospy.Publisher(
    "/t4ac/mapping/map/lanes_marker", Marker, queue_size=1)


def main():
    rospy.init_node("map_visualizator_node", anonymous=True)
    rate = rospy.Rate(1)
    map_path = rospy.get_param("/t4ac/mapping/map_path")
    map_name = rospy.get_param("/t4ac/mapping/map_name")
    previous_map_name = None

    map_object = MapObject(map_name, map_path)
    waypoints = map_object.map_waypoints
    lane_markers = markers_module.get_topology(waypoints, rgb=[192,192,192])

    while not rospy.is_shutdown(): #() and not KeyboardInterrupt):
        map_name = rospy.get_param("/t4ac/mapping/map_name")
        if map_name != previous_map_name:

            map_object = MapObject(map_name, map_path)
            waypoints = map_object.map_waypoints

            lane_markers = markers_module.get_topology(waypoints, [192,192,192])
            map_visualizator_pub.publish(lane_markers)

            previous_map_name = map_name
        rate.sleep()


if __name__ == '__main__':
    print("Start Map visualizator node")
    main()


