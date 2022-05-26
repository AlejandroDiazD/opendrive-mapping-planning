"""
This file simulates the progress the vehicle through the route

The workflow to debug using this file is:
    - Launch a roscore
    - Launch RVIZ
    - Launch the map_visualizator
    - Launch global_planner from Planning Layer
    - Run this file
    - Select 2D Pose Estimate in RVIZ for initial_xyz
    - Select 2D Nav Goal in RVIZ for goal_xyz
    - Visualize results in RVIZ
"""
import sys
import os

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from lane_waypoint_planner import LaneWaypointPlanner


rospy.init_node("odometry_simulator_node", anonymous=True)

map_name = rospy.get_param("/t4ac/mapping/map_name")
map_path = rospy.get_param("/t4ac/mapping/map_path")
lwp = LaneWaypointPlanner(map_name, map_path)

def initial_pose_callback(initial_pose):
    global initial_xyz
    global route_flag
    route_flag = 1
    initial_xyz =(initial_pose.pose.pose.position.x, 
                  initial_pose.pose.pose.position.y, 
                  initial_pose.pose.pose.position.z)
    current_odometry = Odometry()
    current_odometry.pose.pose.position.x = initial_pose.pose.pose.position.x
    current_odometry.pose.pose.position.y = initial_pose.pose.pose.position.y
    current_odometry.pose.pose.position.z = initial_pose.pose.pose.position.z
    while(route_flag == 1):
        odometry_pub.publish(current_odometry)
        rospy.Rate(10).sleep()

def goal_callback(goal_pose):
    global initial_xyz
    global route_flag
    route_flag = 0
    goal_xyz = (goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z)
    route = lwp.calculate_waypoint_route(3, initial_xyz, goal_xyz)
    for waypoint in route:
        current_odometry = Odometry()
        current_odometry.pose.pose.position.x = waypoint.transform.location.x
        current_odometry.pose.pose.position.y = waypoint.transform.location.y
        current_odometry.pose.pose.position.z = waypoint.transform.location.z+1
        odometry_pub.publish(current_odometry)
        rospy.Rate(30).sleep()
    route_flag = 1

initial_pose_sub = rospy.Subscriber(
        "/initialpose", PoseWithCovarianceStamped, initial_pose_callback)
goal_subscriber = rospy.Subscriber(
        "/t4ac/planning/goal", PoseStamped, goal_callback)

odometry_pub = rospy.Publisher("/t4ac/localization/pose", Odometry, queue_size=1)

rospy.spin()

