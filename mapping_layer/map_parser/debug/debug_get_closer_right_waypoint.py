"""
Debug file for optimizing the T4ac_Waypoint.get_closer_right_waypoint
(and left) method.
"""

import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from map_object import MapObject

map_path = '/workspace/team_code/catkin_ws/src/t4ac_mapping_layer/maps/xodr/'
map_object = MapObject('Town03', map_path)

time_0 = time.time()
waypoint = map_object.get_waypoint(120, 3.42, 0)
time_1 = time.time()
print("Time calulating a waypoint = ", time_1-time_0)

right_waypoint = waypoint.get_closer_right_wp(
    map_object.map_waypoints, map_object.map_kdtree)
time_2 = time.time()
print("Time calculating right waypoint = ", time_2-time_1)

print("\n\n >>> ", type(map_object.map_kdtree))