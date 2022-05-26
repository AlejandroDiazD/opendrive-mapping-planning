#!/usr/bin/env python3
"""
Calculate monitorized elements in format of t4ac_msgs so it can be pusblished
in ROS topics.

Last Mod: Alejandro D. 5/5/22
--> Now the system works with the new T4ac_Waypoint methods
"""
import rospy

import t4ac_msgs.msg
from . import lanes_module
from . import junctions_module
# from . import regElem_module


def calulate_lane(central_way):
    """
    Calculate lane in t4ac_msgs.Lane format (for ROS)

    Args:
        central_way: (list) List of T4ac_Waypoint

    Returns:
        monitor_lane: (t4ac_msgs.Lane)
    """
    lane = lanes_module.calculate_lane(central_way)
    monitor_lane = t4ac_msgs.msg.Lane()
    monitor_lane.right.way = lane.right_way
    monitor_lane.left.way = lane.left_way
    monitor_lane.role = lane.role

    return monitor_lane
    
def calculate_lanes(map_waypoints, map_kdtree, segment_index, waypoint_route, n1, n2):
    """
    Calculate lanes in t4ac_msgs.msg/MonitorizedLanes format. This way this 
    info can be published in ROS topics.
    First calculate them using calculate_lanes() in monitor_classes.Lane format
    and make conversion to t4ac_msgs.msg/Lane format.

    Args:
        map_waypoints: (list)
        map_kdtree: (scipy.spatial.kdtree.KDTree) 
        segment_index: (int) Index to locate in which segment of the route 
            is the ego_vehicle
        waypoint_route: (list) Route as a list of T4ac_Waypoint
        n1: (int) Number of waypoints to monitorize in front (current lane)
        n2: (int) Number of waypoints to monitorize in back (back lane)

    Returns:
        monitor_lanes: (list) List of t4ac_msgs.msg/Lane objects
    """
    lanes = lanes_module.calculate_lanes(map_waypoints, map_kdtree, 
        segment_index, waypoint_route, n1, n2)

    monitor_lanes = t4ac_msgs.msg.MonitorizedLanes()
    monitor_lanes.header.frame_id = "MonitorizedLanes"
    monitor_lanes.header.stamp = rospy.Time.now()
    for lane in lanes:
        monitor_lane = t4ac_msgs.msg.Lane()
        monitor_lane.right.way = lane.right_way
        monitor_lane.left.way = lane.left_way
        monitor_lane.role = lane.role
        monitor_lanes.lanes.append(monitor_lane)
    return monitor_lanes

def calculate_intersections(waypoint_route,segment_index, 
    n1, map_object, map_roads, get_lanes_as_dict=False):
    """
    Calculate intersection lanes in t4ac_msgs.msg/MonitorizedLanes format. 
    This way this info can be published in ROS topics.

    Args: 
        waypoint_route: (list) Route as a list of carla.Waypoint
        segment_index: (int) Index to locate in which segment of the route is the
            ego_vehicle
        n1: (int) Number of waypoints to monitorize in front (current lane)
        map_object: (MapObject object) 
        map_roads: (list) All T4ac_Roads in the map obtained by the map_parser
        get_lanes_as_dict: (bool) return the lanes as a dictionary (instead of t4ac_msgs.msg.MonitorizedLanes)
                           where the keys are the roles and the values are list with two sublists (left nodes and
                           right nodes)

    Returns:
        monitor_intersections: (t4ac_msgs.msg.MonitorizedLanes) List of 
            t4ac_msgs.msg/Lane that represent intersection lanes with the
            current lane
    """

    intersection_lanes = junctions_module.calculate_intersections(
        waypoint_route, segment_index, n1, map_object, map_roads)

    if intersection_lanes:
        if get_lanes_as_dict:
            lanes_dict = dict({})

            cnt_merge = cnt_split = cnt_cross = 0

            for lane in intersection_lanes:
                left_nodes = lane.left_way
                right_nodes = lane.right_way
                nodes = [left_nodes, right_nodes]

                role = None
                if lane.role == "split":
                    role = lane.role + "_" + str(cnt_split)
                    cnt_split += 1
                elif lane.role == "merge":
                    role = lane.role + "_" + str(cnt_split)
                    cnt_merge += 1
                elif lane.role == "cross":
                    role = lane.role + "_" + str(cnt_split)
                    cnt_cross += 1

                lanes_dict[role] = nodes
            return lanes_dict
        else:
            monitor_intersections = t4ac_msgs.msg.MonitorizedLanes()
            monitor_intersections.header.frame_id = "MonitorizedIntersections"
            monitor_intersections.header.stamp = rospy.Time.now()
            for lane in intersection_lanes:
                monitor_intersection = t4ac_msgs.msg.Lane()
                monitor_intersection.right.way = lane.right_way
                monitor_intersection.left.way = lane.left_way
                monitor_intersection.role = lane.role
                monitor_intersections.lanes.append(monitor_intersection)
            return monitor_intersections
    else:
        return False

# @TODO
# def calculate_regElems(current_waypoint, waypoint_route, segment_index, n1, 
#                  distance, carla_map):
#     """
#     Calculate Regulatory Elements in t4ac_msgs.msg/MonitorizedRegElem format. 
#     This way this info can be published in ROS topics.

#     Args:
#         current_waypoint: (carla.Waypoint) Current position of ego_vehicle 
#         waypoint_route: (list) Route as a list of carla.Waypoint
#         segment_index: (int) Index to locate in which segment of the route 
#             is the ego_vehicle
#         n1: (int) Number of waypoints to monitorize in front (current lane)
#         distance: (int) Threshold distance to look for landmarks
#         carla_map: (carla.Map) Map to operate with the PythonAPI

#     Returns:
#         monitor_regElems: (t4ac_msgs.msg/MonitorizedRegElem) List of regulatory
#             elements affecting the current monitorized part of the route.
#     """
#     regElems = regElem_module.get_regElems(current_waypoint, 
#                                     waypoint_route[:], segment_index,
#                                     n1, distance, carla_map)
    

#     monitor_regElems = t4ac_msgs.msg.MonitorizedRegElem()
#     if len(regElems) > 0:
#         for regElem in regElems:
#             monitor_regElem = t4ac_msgs.msg.RegulatoryElement()
#             monitor_regElem.type = regElem.element_type
#             monitor_regElem.id = regElem.id
#             monitor_regElem.location.x = regElem.location.x
#             monitor_regElem.location.y = regElem.location.y
#             monitor_regElem.location.z = regElem.location.z
#             monitor_regElem.distance = regElem.distance
#             if len(regElem.landmarks) > 0:
#                 for landmark in regElem.landmarks:
#                     monitor_landmark = t4ac_msgs.msg.Landmark()
#                     monitor_landmark.location.x = landmark.location.x
#                     monitor_landmark.location.y = landmark.location.y
#                     monitor_landmark.location.z = landmark.location.z
#                     monitor_landmark.distance = landmark.distance
#                     monitor_landmark.affecting_road = str(landmark.affecting_road)
#                     monitor_regElem.landmarks.append(monitor_landmark)
#             if len(regElem.affecting_roads) > 0:
#                 for road in regElem.affecting_roads:
#                    monitor_regElem.affecting_roads.append(str(road)) 
#             monitor_regElems.reg_elems.append(monitor_regElem)
#     if len(monitor_regElems.reg_elems) > 0:
#         return monitor_regElems
#     else:
#         return False