#!/usr/bin/env python3
"""
Receive path planning route, ego_vehicle_position and xodr map.
With that info calculate monitorized elements.
"""

import sys 
import os

import rospy
import nav_msgs.msg

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from t4ac_mapping_layer.map_parser.map_object import MapObject

from modules import route_module
from modules import calculus_module
from modules import monitor_module
import t4ac_msgs.msg



class MapMonitor:
    """
    Class for the map monitor. 
    Here are two main callbacks: one when a new route is published an other 
    when localization of th evehicle is published (localization is supposed 
    to be constantly published) 
    """

    def __init__(self, map_data, map_path, map_flag=0):
        # Map
        self.map_object = MapObject(map_data, map_path, map_flag)
        # Ego vehicle localization 
        self.route_segment_centers = None
        self.segment_index = -1
        self.ego_vehicle_location = None
        self.ego_vehicle_waypoint = None
        # Route
        # self.location_route = []
        self.waypoint_route = []
        self.flag_goal_reached = 0
        # Monitor
        self.n_min = 15
        self.monitor_flag = 0

        # Monitor ROS Publishers
        self.lanes_monitor_pub = rospy.Publisher(
            "/t4ac/mapping/monitor/lanes", t4ac_msgs.msg.MonitorizedLanes,
            queue_size=1)#, latch=True)
        self.intersections_monitor_pub = rospy.Publisher(
            "/t4ac/mapping/monitor/intersections", 
            t4ac_msgs.msg.MonitorizedLanes, queue_size=1, latch=True)
        self.regElems_monitor_pub = rospy.Publisher(
            "/t4ac/mapping/monitor/regElems", 
            t4ac_msgs.msg.MonitorizedRegElem, queue_size=1, latch=True)

        # Monitor ROS Subscribers
        self.route_sub = rospy.Subscriber("/t4ac/planning/route",
        t4ac_msgs.msg.Path, self.route_callback)
        self.localization_sub = rospy.Subscriber("/t4ac/localization/pose", 
        nav_msgs.msg.Odometry, self.localization_callback)


    ### Route Callback ###
    def route_callback(self, path_route):
        """
        Callback function called when a route is published by path planner

        Args:
            path_route: Route of type nav_msgs/Path.msg

        Returns: 
            Set route_location and route_waypoint with new route published.
            Also check if vehicle is inside the route and in which segment.
        """
        # Generate route as a list of carla.Waypoint()
        self.waypoint_route = route_module.path_to_waypoint_route(
            path_route, self.map_object)
        # Check conditions for map monitor
        if self.ego_vehicle_waypoint is not None:
            # Get segment_index (if is outside of the route segment_index is -1)
            self.route_segment_centers = (
                route_module.calculate_route_segment_centers(self.waypoint_route))
            self.segment_index = route_module.get_route_segment_index(
                self.route_segment_centers, self.ego_vehicle_waypoint)
            self.monitor_flag = 1
        else:
            self.segment_index = -1
            # print("Warning! self.ego_vehicle_waypoint in map_monitor.py is None")
        


    ### Ego_vehicle_position Callback ###
    def localization_callback(self, ego_vehicle_odometry):
        """
        Callback function called when a ego_vehicle position is published

        Args:
            ego_vehicle_odometry: Current local UTM position of the 
                vehicle of type nav_msgs/Odometry.msg

        Returns: 
            Activates map monitor if possible
        """
        # Get ego-vehicle Odometry as T4ac_Waypoint
        self.ego_vehicle_waypoint = self.map_object.get_waypoint(
                ego_vehicle_odometry.pose.pose.position.x, 
                ego_vehicle_odometry.pose.pose.position.y, 
                ego_vehicle_odometry.pose.pose.position.z)
        
        # Calculate number of waypoints to monitorize depending on the velocity
        # 'n' is number of waypoints to monitorize in the frontside and 'n2' in
        # the backside
        n_max = calculus_module.braking_n_distance(ego_vehicle_odometry)
        if n_max < self.n_min: n_max = self.n_min
        n2_max = int(n_max/2)
        
        # Check conditions to monitorize
        if len(self.waypoint_route) > 0:
            if self.route_segment_centers is not None:
                self.segment_index = route_module.get_route_segment_index(
                    self.route_segment_centers, self.ego_vehicle_waypoint)
                # print(">>> self.segment_index = ", self.segment_index)
            else:
                pass
                # print("Warning! self.route_segment_centers == None in " \
                #       "map_monitor.py")
        else:
            pass
            # print("Warning! self.waypoint_route < 0 in map_monitor.py", 
                # self.segment_index)
            self.segment_index = -1

        # If conditions are ok, monitorize
        if self.segment_index >= 0:              
            # Check if current segment is last segment of the route
            if (self.segment_index == (len(self.waypoint_route)-1)):
                # or self.segment_index == (len(self.waypoint_route)-2)):
                if self.flag_goal_reached == 0:
                    print("Congratz, goal reached!")
                    self.flag_goal_reached = 1
            else:
                # Monitorize 
                if self.monitor_flag == 1:
                    print(">>> Monitoring...")
                    self.monitor_flag = 0
                # Set n front and n2 back waypoints to monitorize
                if (self.segment_index + n_max) < len(self.waypoint_route):
                    n1 = n_max
                else: 
                    n1 = len(self.waypoint_route) - self.segment_index
                if (self.segment_index - n2_max) < 0:
                    n2 = self.segment_index
                else:
                    n2 = n2_max
                # Calculate and publish monitorized lanes
                lanes = monitor_module.calculate_lanes(
                    self.map_object.map_waypoints, self.map_object.map_kdtree, 
                    self.segment_index, self.waypoint_route[:], n1, n2)
                if lanes is not None:
                    # print("Lanes is not None")
                    self.lanes_monitor_pub.publish(lanes)
                else:
                    print("Warning! lanes == None in map_monitor.py")

                # Calculate and publish monitorized intersections
                intersection_lanes = monitor_module.calculate_intersections(
                    self.waypoint_route[:], self.segment_index, n1, 
                    self.map_object, self.map_object.roads)
                if intersection_lanes:
                    self.intersections_monitor_pub.publish(intersection_lanes)

                # @TODO
                # # Calculate and publish regulatory elements
                # regElems = monitor_module.calculate_regElems(
                #     self.ego_vehicle_waypoint, self.waypoint_route[:], 
                #     self.segment_index, n1, 10, self.carla_map)
                # if regElems:
                #     self.regElems_monitor_pub.publish(regElems)


def map_monitor():

    rospy.init_node("map_monitor_node", anonymous=True)
    map_name = rospy.get_param('t4ac/mapping/map_name') 
    map_path = rospy.get_param("t4ac/mapping/map_path")
    map_monitor = MapMonitor(map_name, map_path)    
    rospy.spin()

if __name__ == '__main__':
    try:
        map_monitor()
    except rospy.ROSInterruptException:
        pass

