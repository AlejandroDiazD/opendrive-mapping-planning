#!/usr/bin/env python3
"""
Visualize the route, init and goal point in RVIZ using markers
"""
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from t4ac_msgs.msg import Path

from modules.markers_module import get_point, get_waypoints


class RouteVisualizator():
    """
    Class for visualizing the planner elements (route, init and goal) as
    ROS markers in RVIZ
    """

    def __init__(self):
        self.odometry_sub = rospy.Subscriber(
                    "/t4ac/localization/pose", Odometry, self.odometry_callback)
        self.goal_sub = rospy.Subscriber(
                    "/t4ac/planning/goal", PoseStamped, self.goal_callback)
        self.route_sub = rospy.Subscriber(
                    "/t4ac/planning/route", Path, self.route_callback)


        self.odometry_pub = rospy.Publisher(
            "/t4ac/planning/visualization/odometry", Marker, queue_size=1)
        self.goal_pub = rospy.Publisher(
            "/t4ac/planning/visualization/goal", Marker, queue_size=1)
        self.route_pub = rospy.Publisher(
            "/t4ac/planning/visualization/route", Marker, queue_size=1)



    def odometry_callback(self, odometry):
        """
        Get the odometry of the ego-vehicle and publish it as a ROS marker
        """
        odometry_marker = get_point(odometry.pose.pose.position.x, 
                                    odometry.pose.pose.position.y, 
                                    odometry.pose.pose.position.z, 
                                    [1, 0, 1], 0.5, 1)
        self.odometry_pub.publish(odometry_marker)


    def goal_callback(self, goal):
        """
        Get the goal of the route and publish it as a ROS marker
        """
        goal_marker = get_point(goal.pose.position.x,
                                goal.pose.position.y,
                                goal.pose.position.z,
                                [1, 0, 0], -1, 1)
        self.goal_pub.publish(goal_marker)


    def route_callback(self, route):
        """
        Get the route calculated and publish it as a ROS marker
        """
        route_marker = get_waypoints(route.waypoints, [0, 1, 1], -1, 1)
        self.route_pub.publish(route_marker)


if __name__ == '__main__':
    try:
        rospy.init_node("route_visualizator_node", anonymous=True)
        route_visualizator = RouteVisualizator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass