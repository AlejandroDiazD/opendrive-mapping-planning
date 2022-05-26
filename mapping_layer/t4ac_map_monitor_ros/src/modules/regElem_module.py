"""
================================================
In developement --> This module is not operative
Alejandro Diaz 12/5/22
================================================
Module to get affecting Regulatory Elements to the route
"""
import sys 
import os
import glob

try:
    sys.path.append(glob.glob('/home/robesafe/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

from . import monitor_classes
from . import lanes_module
from . import calculus_module

### debug ###
import rospy
import visualization_msgs
from . import markers_module

landmarks_monitor_pub = rospy.Publisher(
            "/mapping_planning/debug/landmarks", visualization_msgs.msg.Marker,
            queue_size=1, latch=True)
### debug ###

def get_regElems(current_waypoint, waypoint_route, segment_index, n1, 
                 distance, carla_map):
    """
    Get Regulatory Elements affecting the current lane of the route

    Args:
        current_waypoint: (carla.Waypoint) Current position of ego_vehicle 
        waypoint_route: (list) Route as a list of carla.Waypoint
        segment_index: (int) Index to locate in which segment of the route 
            is the ego_vehicle
        n1: (int) Number of waypoints to monitorize in front (current lane)
        distance: (int) Threshold distance to look for landmarks
        carla_map: (carla.Map) Map to operate with the PythonAPI

    Returns:
        regElems: (list) List of monitor_classes.RegulatoryElement() affecting
            any waypoint of the current lane in the route.
    """
    close_landmarks = get_close_landmarks(
        waypoint_route[segment_index:segment_index + n1], distance)

    affecting_landmarks = get_affecting_landmarks(
        waypoint_route[segment_index:segment_index + n1], close_landmarks)

    regElems = []
    for landmark in affecting_landmarks:
        regElem = get_regElem(landmark, carla_map, current_waypoint)
        regElems.append(regElem)

    return regElems

def get_regElem(landmark, carla_map, current_waypoint):
    """
    Get a Regulatory Element (regElem) from a landmark. Regulatory Element
    has the location of the physical element, type, distance from current 
    location and ladmarks associated to the regElemn affecting the current 
    route.

    Args:
        landmark: (carla.Landmark) Landmark affecting the Regulatory Element
        carla_map: (carla.Map) Carla map to use PythonAPI

    Returns:
        regElem: (monitor_classes.RegulatoryElement)
    """
    # Get the landmark object defined in monitor_classes from carla.Landmark
    monitor_landmark = monitor_classes.Landmark()
    monitor_landmark.location = landmark.waypoint.transform.location
    monitor_landmark.distance = current_waypoint.transform.location.distance(
        landmark.transform.location)
    monitor_landmark.affecting_road = landmark.road_id

    # Get the Regulatory Element
    regElem = monitor_classes.RegulatoryElement()
    regElem.element_type = landmark.name
    regElem.id = landmark.id 
    regElem.location = landmark.transform.location
    regElem.distance = current_waypoint.transform.location.distance(
        landmark.waypoint.transform.location)
    regElem.affecting_roads = [landmark.road_id]
    regElem.landmarks = [monitor_landmark]

    return regElem

def get_close_landmarks(route, distance):
    """
    Get close landmarks to the current lane, given a distance

    Args:
        route: (list) Part of the route to check (as a list of carla.Waypoint)
        distance: (int) Threshold distance to look for landmarks

    Returns:
        landmarks: (list) List of carla.landmark()
    """
    landmarks = []
    landmark_ids = []
    for waypoint in route:
        landmarks_aux = waypoint.get_landmarks(distance = 10)
        for landmark in landmarks_aux:
            if landmark.id not in landmark_ids:
                landmarks.append(landmark)
                landmark_ids.append(landmark.id)
    
    return landmarks

def get_affecting_landmarks(route, landmarks):
    """
    Filter a list of landmarks, returning only the ones that affect the current
    lane of the route

    Args: 
        route: (list) Part of the route to check (as a list of carla.Waypoint)
        landmarks: (list) List of carla.Landmark()

    Returns:
        affecting_landmarks: (list) List of carla.Landmark that affect the current
            lane of the route.
    """
    affecting_landmarks = []
    for landmark in landmarks:
        affecting_lanes = get_affecting_lanes(landmark)
        for waypoint in route:
            if (landmark.road_id == waypoint.road_id and 
                waypoint.lane_id in affecting_lanes):
                affecting_landmarks.append(landmark)

    return affecting_landmarks

    
def get_affecting_lanes(landmark):
    """
    Get which lanes is affecting a landmark

    Args:
        landmark: (carla.Landmark)

    Returns:
        affecting_lanes: (list) List of lanes affected by the landmark
    """
    lane_validities = landmark.get_lane_validities()
    affecting_lanes = []

    for lanes in lane_validities:
        for lane in range(abs(lanes[0]), abs(lanes[1]) + 1):
            affecting_lanes.append(lane)
    if lane_validities[0][0] < 0:
        affecting_lanes = [-lane for lane in affecting_lanes]


    return affecting_lanes



