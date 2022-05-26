"""
Module to operate with lane objects. 

Last Mod: Alejandro D. 5/5/22
--> Now the system works with the new T4ac_Waypoint methods
"""

import math

from . import monitor_classes

def calculate_lane(central_way):
    """
    ================================================
    Adaptation for Carla Challenge without PythonAPI
    ================================================
    Create a lane object using a central way of waypoints and calculating 
    waypoints at both sides.

    Args:
        central_way: (list) List of T4ac_Waypoint centered in the lane to 
            calculate

    Returns: 
        lane: (T4ac_Lane) Lane object defined by waypoints at both sides 
            of the lane
    """
    lane = monitor_classes.Lane()  
    lane.central_way = central_way
    for i in range(len(central_way)):
        central_wp = central_way[i]
        node_right = monitor_classes.Node3D()
        node_left = monitor_classes.Node3D()
        yaw = central_wp.transform.rotation.yaw
        alpha = yaw # dev for challenge
        alpha_radians = math.radians(alpha)
        distance = central_wp.lane_width/2

        k = -1
        if central_way[i].lane_id < 0:
            k = 1
            
        node_right.x = central_wp.transform.location.x + math.cos(alpha_radians)*distance*(-k)
        node_right.y = central_wp.transform.location.y - math.sin(alpha_radians)*distance*k
        node_right.z = central_wp.transform.location.z
        lane.right_way.append(node_right)

        node_left.x = central_wp.transform.location.x - math.cos(alpha_radians)*distance*(-k)
        node_left.y = central_wp.transform.location.y + math.sin(alpha_radians)*distance*k
        node_left.z = central_wp.transform.location.z
        lane.left_way.append(node_left)

    return lane
  
def calculate_lanes(map_waypoints, map_kdtree, segment_index, waypoint_route, n1, n2):
    """
    Calculate lanes to monitorize in current route from current position to
    'n' waypoint in front and 'n2' in back. Also monitorize right and left
    lanes if lane change is allowed.

    Args:
        map_waypoints: (list)
        map_kdtree: (scipy.spatial.kdtree.KDTree)
        segment_index: (int) Index to locate in which segment of the route 
            is the ego_vehicle
        waypoint_route: (list) Route as a list of T4ac_Waypoint
        n1: (int) Number of waypoints to monitorize in front (current lane)
        n2: (int) Number of waypoints to monitorize in back (back lane)

    Returns:
        lanes: (list)List of monitor_classes.Lane object for current, right and
               left front lanes, and current, right and left back lanes. 
    """
    lanes = []
    # Current front lane
    current_front_central_way = []
    for i in range(n1):
        current_central_waypoint = waypoint_route[segment_index + i]
        current_front_central_way.append(current_central_waypoint)
    current_front_lane = calculate_lane(current_front_central_way)
    current_front_lane.role = "current_front" 
    lanes.append(current_front_lane)

    # Current back lane
    current_back_central_way = []
    for i in range(n2):
        current_back_central_waypoint = waypoint_route[segment_index - n2 + i]
        current_back_central_way.append(current_back_central_waypoint)
    current_back_lane = calculate_lane(current_back_central_way)
    current_back_lane.role = "current_back"
    lanes.append(current_back_lane)

    # Right front lane
    right_front_central_way = []
    for waypoint in current_front_central_way:
        if (waypoint.lane_change == "right" or 
            waypoint.lane_change == "both"):
            right_front_central_waypoint = waypoint.get_closer_right_wp(map_waypoints, map_kdtree)
            if right_front_central_waypoint is not None:
               right_front_central_way.append(right_front_central_waypoint)
    right_front_lane = calculate_lane(right_front_central_way)
    right_front_lane.role = "right_front"
    lanes.append(right_front_lane)

    # Right back lane
    right_back_central_way = []
    for waypoint in current_back_central_way:
        if (waypoint.lane_change == "right" or 
            waypoint.lane_change == "both"):
            right_back_central_waypoint = waypoint.get_closer_right_wp(map_waypoints, map_kdtree)
            if right_back_central_waypoint is not None:
               right_back_central_way.append(right_back_central_waypoint)
    right_back_lane = calculate_lane(right_back_central_way)
    right_back_lane.role = "right_back"
    lanes.append(right_back_lane)

    # Left front lane
    left_front_central_way = []
    for waypoint in current_front_central_way:
        if (waypoint.lane_change == "left" or 
            waypoint.lane_change == "both"):
            left_front_central_waypoint = waypoint.get_closer_left_wp(map_waypoints, map_kdtree)
            if left_front_central_waypoint is not None:
                left_front_central_way.append(left_front_central_waypoint)
    left_front_lane = calculate_lane(left_front_central_way)
    left_front_lane.role = "left_front"
    lanes.append(left_front_lane)

    # Left back lane
    left_back_central_way = []
    for waypoint in current_back_central_way:
        if (waypoint.lane_change == "left" or 
            waypoint.lane_change == "both"):
            left_back_central_waypoint = waypoint.get_closer_left_wp(map_waypoints, map_kdtree)
            if left_back_central_waypoint is not None:
                left_back_central_way.append(left_back_central_waypoint)
    left_back_lane = calculate_lane(left_back_central_way)
    left_back_lane.role = "left_back"
    lanes.append(left_back_lane)

    return lanes

def calculate_contour(lane, start_index = 0, end_index = -1):
    """
    Receive a lane and get contour of the segment of that lane defined by an
    initial and a final index. If start or end index are not given, contour is
    calculated for complete lane. 

    Args:
        lane: monitor_classes.Lane object of the lane to be checked
        start_index: Initial index where the segment of the lane starts
        end_index: Final index where the segment of the lane ends

    Returns:
        contour: List of Node2D(x,y) defining the contour of the segment. This contour 
            can be used to check if a position is inside a lane with the function
            inside_polygon(position, polygon)
    """
    if end_index == -1: end_index = len(lane.right_way)
    contour = []
    for i in range (0,len(lane.left_way)):
        p_2D = monitor_classes.Node2D()
        p_2D.x = lane.left_way[i].x
        p_2D.y = lane.left_way[i].y         
        contour.append(p_2D)
    for i in range(0,len(lane.right_way)):
        p_2D = monitor_classes.Node2D()
        p_2D.x = lane.right_way[len(lane.right_way)-i-1].x
        p_2D.y = lane.right_way[len(lane.right_way)-i-1].y
        contour.append(p_2D)
    return contour