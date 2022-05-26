"""
Module to implemente some route calculus
"""

import sys 
import os
import math 

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
from t4ac_mapping_layer.map_parser.builder_classes import T4ac_Location
from t4ac_mapping_layer.map_parser.builder_classes import T4ac_Waypoint


def path_to_waypoint_route(path_route, map_object):
    """
    Transform a route of nav_msgs/Path.msg format into a route of 
    T4ac_Waypoint() format

    Args:
        path_route: (t4ac_msgs.Path) 
        map_object: (class) MapObject generated from t4ac_mapping_layer

    Returns: 
        waypoint_route: (list) Route as a List of T4ac_Waypoint
    """
    waypoint_route = []
    for path_value in path_route.waypoints:
        waypoint = map_object.get_waypoint(
                                path_value.transform.location.x, 
                                path_value.transform.location.y, 
                                path_value.transform.location.z)
        waypoint_route.append(waypoint)
    return waypoint_route

def calculate_route_segment_centers(waypoint_route):
    """
    Calculates len(waypoint_route)-1 centers of each segment for then
    calculating the closer segment.
    
    Args:
        waypoint_route: (list) List of T4ac_Waypoint

    Returns:
        route_segment_centers: (list) List of T4ac_Waypoint
    """
    route_segment_centers = []
    for i in range(len(waypoint_route)-1):
        location = T4ac_Location(
            ((waypoint_route[i].transform.location.x + 
              waypoint_route[i+1].transform.location.x) / 2),
            ((waypoint_route[i].transform.location.y + 
              waypoint_route[i+1].transform.location.y) / 2), 
            ((waypoint_route[i].transform.location.z + 
              waypoint_route[i+1].transform.location.z) / 2))
        waypoint = T4ac_Waypoint(location)
        route_segment_centers.append(waypoint)
    return route_segment_centers
        
def get_route_segment_index(route_segment_centers, current_waypoint):
    """
    Get the closer route segment where is supposed to be the current 
    waypoint. A threshold of 5 meters is considered.

    Args:
        route_segment_centers: (list) List of T4ac_Location
        current_waypoint: (T4ac_Waypoint)
    
    Returns:
        segment_index: (int)
    """
    closer_distance = 10000 # High arbitrary value
    for i, wp in enumerate(route_segment_centers):
        distance = math.sqrt(
            (wp.transform.location.x-current_waypoint.transform.location.x)**2 + 
            (wp.transform.location.y-current_waypoint.transform.location.y)**2 +
            (wp.transform.location.z-current_waypoint.transform.location.z)**2)
        if distance < closer_distance:
            closer_distance = distance
            closer_wp = wp
            segment_index = i
    if closer_distance < 5:
        return segment_index
    else:
        return -1





