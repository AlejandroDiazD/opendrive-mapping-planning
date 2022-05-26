"""
===================
Only for developers
===================
File to debug and test the LaneWaypointPlanner (LWP)

The route calculated is represented in RVIZ using ROS markers
"""

import sys
import os

import rospy
from visualization_msgs.msg import Marker

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from lane_waypoint_planner import LaneWaypointPlanner
from modules.markers_module import get_waypoints

LWP = LaneWaypointPlanner("Town10HD", 0)

# route = LWP.calculate_waypoint_route(3, (216.91, -59.18, 0), (116.30, -59.36, 0))
route = LWP.calculate_waypoint_route(3, (-25.79, -27.52, 0.0), (71.34, -24.07, 0.0))

route_pub = rospy.Publisher("debugging/route", Marker, queue_size=1)
rospy.init_node("route_debugger_node", anonymous=True)
rate = rospy.Rate(1)
route_marker = get_waypoints(route, [0, 1, 1], -1, 1)

print(">>> OK")

while not rospy.is_shutdown():
    route_pub.publish(route_marker)
    rate.sleep()

