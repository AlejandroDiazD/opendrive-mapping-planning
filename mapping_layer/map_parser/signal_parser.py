"""
================================================
In developement --> This module is not operative
Alejandro Diaz 12/5/22
================================================
Provisional parser module for providing signals information in 
Carla Challenge 2021 using XODR information. 

Main function of this module is:
    * parse_signals(map_data, map_flag)

That returns a list of signal dicts
"""

import sys
import math

sys.path.insert(0, '/workspace/team_code/catkin_ws/src/t4ac_mapping_layer')
from map_parser import map_parser
from map_utils import map_utils

def get_map_string(map_name):
    """
    Receives the map name and returns the map data into a string. 
    It looks for the map in the standard map path of our project
    """
    path = "/workspace/team_code/catkin_ws/src/t4ac_mapping_layer/maps/xodr/carla_0910/"
    map_path = path + map_name + ".xodr"
    with open (map_path, "r") as myfile:
        map_string = myfile.read()
    return map_string


def parse_regElems(roads):
    """
    This function puts together the functionality of parse_signals
    and parse_stops into one function.

    Args:
        roads: (list) List of roads from the map object. The map object is 
            created when parsing the xodr file or when the LaneWaypointPlanner
            is instantiated. For example: LWP.map_object.roads (see MapParser)

    Returns: 
        parsed_signals: (list) A list of the parsed signals. Each parsed signal
            is a python dictionary with all the available information from 
            the xodr map.
        parsed_stops: (list) A list of the parsed stops. Each parsed stop
            is a python dictionary with all the available information from 
            the xodr map.
    """
    parsed_signals = []
    parsed_stops = []

    for road in roads:

        if len(road.signals.signal) > 0:
            for signal in road.signals.signal:
                parsed_signal = {}
                parsed_signal['name'] = signal.name
                parsed_signal['id'] = int(signal.id)
                parsed_signal['road'] = int(road.id)
                parsed_signal_xyz, heading = calculate_signal_xyz(
                    road, float(signal.s), float(signal.t), 
                    float(signal.hOffset), float(signal.height), 
                    signal.orientation)
                parsed_signal['x'] = parsed_signal_xyz.x
                parsed_signal['y'] = parsed_signal_xyz.y
                parsed_signal['z'] = parsed_signal_xyz.z
                parsed_signal['yaw'] = heading
                parsed_signals.append(parsed_signal)

        if len(road.objects) > 0:
            for object in road.objects:
                if object.name == "Stencil_STOP" or object.name == "StopLine":
                    parsed_stop = {}
                    parsed_stop['name'] = object.name
                    parsed_stop['id'] = int(object.id)
                    parsed_stop['hdg'] = float(object.hdg)
                    parsed_stop['zOffset'] = float(object.zOffset)
                    parsed_stop['length'] = float(object.length)
                    parsed_stop['width'] = float(object.width)
                    parsed_stop['s'] = float(object.s)
                    parsed_stop['t'] = float(object.t)
                    parsed_stop['orientation'] = object.orientation
                    parsed_stop_xyz, _ = calculate_signal_xyz(
                        road, float(object.s), float(object.t), 
                        float(object.zOffset), 0, 
                        object.orientation)
                    parsed_stop['x'] = parsed_stop_xyz.x
                    parsed_stop['y'] = parsed_stop_xyz.y
                    parsed_stop['z'] = parsed_stop_xyz.z
                    parsed_stops.append(parsed_stop)

    return parsed_signals, parsed_stops


def parse_signals(roads):
    """
    """
    signals = []

    for road in roads:
        if len(road.signals.signal) > 0:
            for signal in road.signals.signal:
                parsed_signal = {}
                parsed_signal['name'] = signal.name
                parsed_signal['id'] = int(signal.id)
                parsed_signal['road'] = int(road.id)
                parsed_signal_xyz, heading = calculate_signal_xyz(
                    road, float(signal.s), float(signal.t), 
                    float(signal.hOffset), float(signal.height), 
                    signal.orientation)
                parsed_signal['x'] = parsed_signal_xyz.x
                parsed_signal['y'] = parsed_signal_xyz.y
                parsed_signal['z'] = parsed_signal_xyz.z
                parsed_signal['yaw'] = heading
                signals.append(parsed_signal)
    return signals

def parse_stops(roads):
    """
    Receive the roads list from the map_object and return the stop
    signals parsed

    Args: 
        roads: (list) List of roads from the map object. The map object is 
            created when parsing the xodr file or when the LaneWaypointPlanner
            is instantiated. For example: LWP.map_object.roads (see MapParser)

    Returns:
        parsed_stops: (list) A list of the parsed stops. Each parsed stop
            is a python dictionary with all the available information from 
            the xodr map.
    """
    parsed_stops = []
    for road in roads:
        if len(road.objects) > 0:
            for object in road.objects:
                if object.name == "Stencil_STOP" or object.name == "StopLine":
                    parsed_stop = {}
                    parsed_stop['name'] = object.name
                    parsed_stop['id'] = int(object.id)
                    parsed_stop['hdg'] = float(object.hdg)
                    parsed_stop['zOffset'] = float(object.zOffset)
                    parsed_stop['length'] = float(object.length)
                    parsed_stop['width'] = float(object.width)
                    parsed_stop['s'] = float(object.s)
                    parsed_stop['t'] = float(object.t)
                    parsed_stop['orientation'] = object.orientation
                    parsed_stop_xyz, _ = calculate_signal_xyz(
                        road, float(object.s), float(object.t), 
                        float(object.zOffset), 0, 
                        object.orientation)
                    parsed_stop['x'] = parsed_stop_xyz.x
                    parsed_stop['y'] = parsed_stop_xyz.y
                    parsed_stop['z'] = parsed_stop_xyz.z
                    parsed_stops.append(parsed_stop)
    return parsed_stops




def calculate_signal_xyz(road, s, t, hOffset, height, orientation):
    """
    """
    # Locate in which geometry is s of the signal
    geometry_index = 0
    length = 0
    for i in range(0, len(road.planView)):
        # if (road.planView[i].length > length):
        #     geometry_index = i
        #     length = road.planView[i].length
        #     print
        

        if s > road.planView[i].s:
            geometry_index = i
        else:
            break

    i = geometry_index
    s_distance = s - road.planView[i].s
    heading = road.planView[i].hdg + hOffset

    # Apply 't' traslation
    # if orientation == '-':
    #     # pass
    #     heading = road.planView[i].hdg + hOffset
    # elif orientation == '+':
    #     # t = -t
    #     heading = road.planView[i].hdg - hOffset

    

    t_xyz = map_utils.get_point_in_line(
        road.planView[i].x, road.planView[i].y, 0, t, road.planView[i].hdg + math.pi/2)
    
    # Apply 's' traslation
    if (road.planView[i].type == "line"):
        signal_xyz = map_utils.get_point_in_line(
            t_xyz.x, t_xyz.y, height, s_distance, road.planView[i].hdg)

    elif (road.planView[i].type == "arc"):
        # heading += math.pi/2
        radius = 1 / road.planView[i].curvature
        new_radius = radius - t
        k_arc = s_distance / radius
        new_s = k_arc * new_radius
        
        new_curvature = 1 / new_radius
        heading += new_s*new_curvature
        signal_xyz = map_utils.get_point_in_arc(
            t_xyz.x, t_xyz.y, t_xyz.z, new_s, new_curvature, road.planView[i].hdg) # + hOffset)

    if heading < 0:
        heading += 2 * math.pi
    if heading > 2 * math.pi:
        heading -= 2 * math.pi

    return signal_xyz, heading



root_path = "/workspace/team_code/catkin_ws/src/t4ac_mapping_layer/maps/xodr/"
town_name = "carla_0910/Town01.xodr"
map_path = root_path + town_name
with open (map_path, "r") as myfile:
    map_string = myfile.read()

parsed_signals = parse_signals(map_string, 1)
print("Hola")