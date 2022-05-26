#!/usr/bin/env python3
"""
Module to implement callbacks for monitorized elements. Every time a
monitorized element is published, there is a callback to represent it
in RVIZ
"""

import rospy
import visualization_msgs.msg

import t4ac_msgs.msg
from modules import monitor_classes
from modules import markers_module

class MonitorVisualizator:
    """
    Class for the monitor visualizator.
    Represents all the monitorized elements in ROS topics as markers, to 
    be visualized in RVIZ.
    """

    def __init__(self):
        # ROS Subscribers
        self.lanes_monitor_sub = rospy.Subscriber(
            "/t4ac/mapping/monitor/lanes", 
            t4ac_msgs.msg.MonitorizedLanes, self.lanes_callback)
        self.intersections_monitor_sub = rospy.Subscriber(
            "/t4ac/mapping/monitor/intersections", 
            t4ac_msgs.msg.MonitorizedLanes, self.intersections_callback)
        self.regElems_monitor_sub = rospy.Subscriber(
            "/t4ac/mapping/monitor/regElems", 
            t4ac_msgs.msg.MonitorizedRegElem, self.regElems_callback)

        # ROS Publishers
        self.lanes_monitor_visualizator_pub = rospy.Publisher(
            "/t4ac/mapping/monitor/lanes_marker", 
            visualization_msgs.msg.MarkerArray, queue_size = 12)
        self.intersections_monitor_visualizator_pub = rospy.Publisher(
            "/t4ac/mapping/monitor/intersections_marker", 
            visualization_msgs.msg.MarkerArray, queue_size = 10)
        self.regElems_monitor_visualizator_pub = rospy.Publisher(
            "/t4ac/mapping/monitor/regElems_marker", 
            visualization_msgs.msg.Marker, queue_size = 10)

    def lanes_callback(self, lanes):
        """
        Callback function called when lanes are published by the map_monitor
        in /t4ac/mapping/monitor/lanes

        Args:
            lanes: (t4ac_msgs.msg.MonitorizedLanes)
                Monitorized lanes pusblished by the map_monitor

        Returns:
            Publish lane markers to be visualized in RVIZ
        """
        standard_lane_markers = visualization_msgs.msg.MarkerArray()
        for lane in lanes.lanes:
            if lane.role == "current_front" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                        lane, [1,0,0], "current_front_", 4, 0.2, 0.1, 0.5, 0)

            elif lane.role == "current_back" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                        lane, [0,0,1], "current_back_", 4, 0.2, 0.1, 0.5, 2)

            elif lane.role == "right_front" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                        lane, [1,1,0], "right_front_", 8, 0.4, 0.1, 0.5, 4)

            elif lane.role == "right_back" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                        lane, [1,1,0], "right_back_", 8, 0.4, 0.1, 0.5, 6, 0.5)

            elif lane.role == "left_front" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                        lane, [1,1,0], "left_front_", 8, 0.4, 0.1, 0.5, 8)

            elif lane.role == "left_back" and lane.left.way and lane.right.way:
                standard_lane_markers.markers += markers_module.get_lane(
                        lane, [1,1,0], "left_back_", 8, 0.4, 0.1, 0.5, 10, 0.5)

        self.lanes_monitor_visualizator_pub.publish(standard_lane_markers)


    def intersections_callback(self, intersection_lanes):
        """
        Callback function called when intersection_lanes are published by 
        the map_monitor in /t4ac/mapping/monitor/intersections

        Args:
            intersection_lanes: (t4ac_msgs.msg.MonitorizedLanes).
                Monitorized intersection_lanes pusblished by the map_monitor

        Returns:
            Publish lane_intersection markers to be visualized in RVIZ
        """
        intersection_lane_markers = visualization_msgs.msg.MarkerArray()
        i = 0
        for lane in intersection_lanes.lanes:
            if lane.role == "merge": # Colour yellow
                intersection_lane_markers.markers += ( 
                    markers_module.get_lane(
                        lane, [1,1,0], "merge_"+str(i), 4, 0.2, 0.05, 1, i))
                i += 1

            elif lane.role == "split": # Colour orange
                intersection_lane_markers.markers += (
                    markers_module.get_lane(
                        lane, [1,0.5,0], "split_"+str(i), 4, 0.2, 0.05, 1, i))
                i += 1

            elif lane.role == "cross": # Colour purple
                intersection_lane_markers.markers += (
                    markers_module.get_lane(
                        lane, [1,0,1], "cross_"+str(i), 4, 0.2, 0.05, 1, i))
                i += 1

        self.intersections_monitor_visualizator_pub.publish(intersection_lane_markers)

    def regElems_callback(self, regElems):
        """
        Callback function called when regElems are published by 
        the map_monitor in /t4ac/mapping/monitor/regElems

        Args:
            regElems: (t4ac_msgs.msg.MonitorizedRegElem).
                Monitorized Regulatory Elements pusblished by the map_monitor

        Returns:
            Publish Regulatory Element markers to be visualized in RVIZ
        """
        nodes = []

        if len(regElems.reg_elems) > 0:
            for regElem in regElems.reg_elems:
                node = monitor_classes.Node3D()
                node.x = regElem.location.x
                node.y = regElem.location.y
                node.z = regElem.location.z
                nodes.append(node)
                if len(regElem.landmarks) > 0:
                    for landmark in regElem.landmarks:
                        node = monitor_classes.Node3D()
                        node.x = landmark.location.x
                        node.y = landmark.location.y
                        node.z = landmark.location.z
                        nodes.append(node)

            landmarks_marker = markers_module.get_nodes(
                nodes, [1,0,0], "1", 8, 0.5, 1, 0.2)
            self.regElems_monitor_visualizator_pub.publish(landmarks_marker)


def monitor_visualizator():
    # Init node
    rospy.init_node("monitor_visualizator_node", anonymous=True)
    monitor_visualizator = MonitorVisualizator()
    rospy.spin()

if __name__ == '__main__':
    try:
        monitor_visualizator()
    except rospy.ROSInterruptException:
        pass
