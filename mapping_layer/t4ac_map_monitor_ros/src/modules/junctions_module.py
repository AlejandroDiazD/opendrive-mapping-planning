"""
Module to get intersection lanes in map monitor. This is an adaptation from 
the previous junctions_module, but without any use of PythonAPI.

*Note: A 'way' concept is used in some functions, it means a list of 
waypoints centered at the lane. 
"""

from . import lanes_module
from . import calculus_module


def calculate_intersections(waypoint_route, segment_index, n1, map_object, map_roads):
    """
    Calculate intersection lanes affecting the current monitorized lane

    Args:
        waypoint_route: (list) Route as a list of T4ac_Waypoint
        segment_index: (int) Index to locate in which segment of the route is the
            ego_vehicle
        n1: (int) Number of waypoints to monitorize in front (current lane)
        map_junctions: (list) List of all T4ac_Junctions of the map obtained 
                        in the map_parser
        map_roads: (list) All T4ac_Roads in the map obtained by the map_parser
    """
    # Get the way of the current monitorized lane
    current_way = waypoint_route[segment_index : segment_index + n1]

    # Get the id of the junctions affecting the current way
    junctions = get_junctions(current_way, map_object)
    # print("junctions: ", junctions)
    if junctions:
        # Calculate the discretized intersection lanes affecting the current
        # way. Intersection lanes can be split, merge or cross.
        intersection_lanes = check_junctions(junctions, waypoint_route[:], map_object)
        return intersection_lanes
    else:
        return False

def get_junctions(current_way, map_object):
    # Necesita el objeto map_junctions
    """
    Return the junctions objects affecting any of the waypoint list 
    given as input.

    Args:
        current_way: (list) List of waypoints centered at the current lane
        map_junctions: (list) List of all T4ac_Junctions of the map obtained 
                        in the map_parser

    Returns: 
        junctions: (list) List of T4ac_Junction objects
    """
    junctions = []
    junctions_id = []
    for waypoint in current_way:
        # print("waypoint.junction: ", waypoint.junction)
        if waypoint.junction > -1:
            junction_id = waypoint.junction
            # print("junction_id: ", junction_id)
            # print("junctions_id: ", junctions_id)
            if junction_id not in junctions_id:
                junctions_id.append(junction_id)
                junction = map_object.get_junction(map_object.junctions, junction_id)
                # print("junction: ", junction)
                junctions.append(junction)
    return junctions

def check_junctions(junctions, waypoint_route, map_object):
    """
    Receive a list of junction a calculate for each junction its inteserction
    lanes with the route.
    Type of intersections are:
        - merge
        - split
        - cross

    Args:
        junctions: (list) List of T4ac_Junction
        waypoint_route: (list) Route as a list of T4ac_Waypoint
        map_roads: (list) All T4ac_Roads in the map obtained by the map_parser

    Returns:
        intersection_lanes: (list) List of T4ac_Lane that are an intersection of
            the current lane.
    """
    intersection_lanes = []
    for junction in junctions:
        junction_ways = calculate_junction_ways(junction, map_object)
        main_way, junction_ways = get_main_way(junction_ways[:], waypoint_route[:])
        main_lane = lanes_module.calculate_lane(waypoint_route[:])
        main_contour = lanes_module.calculate_contour(main_lane)

        for way in junction_ways:
            way_role = classify_way(way, main_contour)
            if way_role == "split" or way_role == "merge" or way_role == "cross":
                intersection_lane = lanes_module.calculate_lane(way)
                intersection_lane.role = way_role
                intersection_lanes.append(intersection_lane)
    return intersection_lanes

def get_main_way(junction_ways, waypoint_route):
    """
    Receive a list of ways and separate main way from rest of junction ways.

    Args:
        junction_ways: (list) List of ways inside monitorized junctions
        waypoint_route: (list) Route as a list of T4ac_Waypoint

    Returns: 
        main_way: (list) Way of the current way of the route in the junction.
            Add an extra waypoint at the begining and another at the end, to 
            make sure when check other waypoints inside this contour.
        ways: (list) Other ways of the junction that are not main way
    """
    ways = []
    main_way = None
    route_lane = lanes_module.calculate_lane(waypoint_route[:])
    route_contour = lanes_module.calculate_contour(route_lane)

    for way in junction_ways:
        if way:
            if (calculus_module.inside_polygon(
                    way[3].transform.location, route_contour) and 
                calculus_module.inside_polygon(
                    way[-4].transform.location, route_contour)):
                    main_way = way[:]
                    # waypoint_options_previous = main_way[0].previous(3)
                    # extra_waypoint = waypoint_options_previous[0]
                    # main_way.insert(0, extra_waypoint)
                    # waypoint_options_next = main_way[-1].next(3)
                    # extra_waypoint = waypoint_options_next[0]
                    # main_way.append(extra_waypoint)

            else:
                ways.append(way)
                # ways.append(way[2:-2])

    return main_way, ways


def classify_way(way, contour):
    """
    Classify the role of a way with the current lane. A way can be:
    - merge (first waypoint is outside, last waypoint is inside the lane)
    - split (first waypoint is inside, last waypoint is outside the lane)
    - cross (first and last waypoint are outside, but some others are inside)
    - none (the way is not any type of intersection with the current lane)
    with the current lane.

    Args:
        way: (list) Way to check
        contour: (list) Waypoints defining the contour of the 
            lane to check if the way is inside

    Retuns:
        way_role: (str) Intersection role of the way
    """
    way_role = "none"
    if (calculus_module.inside_polygon(
            way[0].transform.location, contour) and
        calculus_module.inside_polygon(
            way[-1].transform.location, contour)):
        way_role = "current"
    elif (calculus_module.inside_polygon(
            way[0].transform.location, contour) and not
        calculus_module.inside_polygon(
            way[-1].transform.location, contour)):
        way_role = "split"
    elif (calculus_module.inside_polygon(
            way[-1].transform.location, contour) and not
        calculus_module.inside_polygon(
            way[0].transform.location, contour)):
        way_role = "merge"
    elif ((calculus_module.inside_polygon(
            way[0].transform.location, contour) or
        calculus_module.inside_polygon(
            way[-1].transform.location, contour)) == 0):
        for i in range(1, len(way)-1):
            if calculus_module.inside_polygon(way[i].transform.location, 
                contour):
                way_role = "cross"
                break

    return way_role


def calculate_junction_ways(junction, map_object):
    # Necesita el objecto map_roads
    """
    Get junction ways (way is a list of waypoints centered in the lane)
    given a specific junction

    Args:
        junction: (T4ac_Junction)
        map_roads: (list) All T4ac_Roads in the map obtained by the map_parser

    Returns:
        junctions_ways: (list) List of ways in the junction (one way for each 
        possible lane inside the junction)
    """
    # Get road ids of junction ways
    road_lane_ids = []

    for connection in junction.connection:
        for laneLink in connection.laneLink:
            road_lane_id = (connection.connectingRoad, laneLink.toLane)
            if road_lane_id not in road_lane_ids:
                road_lane_ids.append(road_lane_id)

    # Generate a way for each road_lane_id
    junction_ways = []
    distance = 1 # Distance between waypoints in the way
    for road_lane_id in road_lane_ids:
        road = map_object.get_road(map_object.roads, road_lane_id[0])
        lane = map_object.get_lane(road, road_lane_id[1], 0)
        if lane and lane.type == "driving":
            way = map_object.generate_waypoints_in_lane(road, lane, distance)
            junction_ways.append(way)
    return junction_ways


def get_main_ways_in_route(waypoint_route):
    """
    Receives a route of waypoints and returns the main ways of the junctions
    in the route (one main way for each junction)

    Args:
        waypoint_route: (list)

    Returns :
        main_ways: (list)
    """

    main_ways = []
    previous_junction = None
    in_junction = None
    main_way = None

    for i in range(len(waypoint_route)):
        # If the waypoint is inside a junction
        if waypoint_route[i].junction > -1:
            # Check if is a new junction
            if  waypoint_route[i].junction != previous_junction:
                # Create a new main way that will be filled with waypoints
                in_junction = 1 # Flag to know that is in a junction
                main_way = []

                # Try to add an extra waypoint at the begining of the main_way
                if ((i - 2) >= 0):
                    main_way.append(waypoint_route[i-2])
                # And add the current waypoint
                main_way.append(waypoint_route[i])
                previous_junction = waypoint_route[i].junction

            # If not it means that is still in the same junction
            else:
                # Keep adding waypoints
                main_way.append(waypoint_route[i])

        # This case means a main way has been filled
        elif waypoint_route[i].junction == -1 and in_junction == 1:
            in_junction = -1
            # Try to add an extra waypoint at the end of the main_way
            if ((i + 2) < len(waypoint_route)):
                main_way.append(waypoint_route[i+2])
            # And add the main_way to the main_ways list to return
            main_way_tuple = (previous_junction, main_way)
            main_ways.append(main_way_tuple)

    return main_ways
