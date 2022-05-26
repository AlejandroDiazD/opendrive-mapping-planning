#!/usr/bin/env python3
"""
================
Map Object Class
================

This class inherits from the MapParser and provides methods to operate
with the map_object generated from the HD map  

"""
# General imports
import sys
import os
import math

import numpy as np
from scipy.spatial import KDTree

# T4AC imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))
from map_parser import MapParser
from builder_classes import T4ac_Waypoint
from builder_classes import T4ac_Location


class MapObject(MapParser):
    """
        Args
            map_data: (str) It can be given in two ways:
                            0) Name of the map without extension, i.e. 'Town01' \
                            1) All map data saved as a string 
            map_path: (str) Path of the map files
            map_flag: (bool) 0) map_data in case 0
                             1) map data in case 1
    """

    def __init__(self, map_data, map_path, map_flag = 0):
        MapParser.__init__(self, map_data, map_path, map_flag)
        
        self.distance_between_waypoints = 0.2 # Distance in meters between 
                                              # waypoints that are generated
        self.map_name = self._get_map_name()
        self.map_waypoints = self.generate_waypoints(
            self.roads, self.distance_between_waypoints)

        self.map_locations_array = ([np.array((
            wp.transform.location.x, 
            wp.transform.location.y, 
            wp.transform.location.z)) 
            for wp in self.map_waypoints])
        self.map_kdtree = KDTree(self.map_locations_array)

    def _get_map_name(self):
        """
        Analyze the xodr map data and returns the name of the map.
        This has been implemented for Carla Challenge 2021, where we
        have the map_data in a string but we don't know to which map_name
        corresponds.

        For doing that, it checks some parameters previously known from 
        the map.

        Returns:
            map_name: (str) name of the map
        """

        if int(self.roads[0].length) == 36:
            return("Town01")

        elif int(self.roads[0].length) == 95:
            return("Town02")

        elif int(self.roads[0].length) == 43:
            return("Town03")

        elif int(self.roads[0].length) == 34:
            return("Town04")

        elif int(self.roads[0].length) == 53:
            return("Town05")

        elif int(self.roads[0].length) == 3:
            return("Town06")

        elif int(self.roads[0].length) == 33:
            return("Town07")

        elif int(self.roads[0].length) == 67:
            return("Town10")

        elif int(self.roads[0].length) == 13:
            return("Town10HD")

        else:
            return"UnknownMap"

    def get_waypoint(self, x, y, z, project_to_road=True):
        """
        Returns the closest waypoint centered in a driving lane of a road

        Args:
            x, y, z: (flt, flt, flt) xyz of the point
            project_to_road: 
                1) True: Generates the waypoint centered in the closer
                driving lane location
                2) False: Generates the waypoint in the same location

        Returns:
            waypoint: (T4ac_Waypoint)
        """
        if project_to_road == True:
            location_array = np.array((x, y, z))
            closer_dist, closer_point = self.map_kdtree.query(
                location_array, 1)
            waypoint = self.map_waypoints[closer_point.numerator]
        else:
            location = T4ac_Location(x, y, z)
            waypoint = T4ac_Waypoint(location)
        return waypoint
        
    def get_point_in_line(self,x,y,z,s,heading):
        """
        Get a T4ac_Location into a line geometry given an 's' distance
        """
        
        location = T4ac_Location(x, y, z)

        location.x += (s * math.cos(heading)) #+ (t * math.sin(heading))
        location.y += (s * math.sin(heading)) #+ (t * math.cos(heading))  

        return location

    def get_point_in_arc(self,x,y,z,s,curvature,heading):
        """
        Get a T4ac_Location into an arc geometry given an 's' distance
        """
        location = T4ac_Location(x, y, z)
        radius = 1.0 / curvature
        pi_half = math.pi / 2

        location.x += radius * math.cos(heading + pi_half)     
        location.y += radius * math.sin(heading + pi_half)
        heading += s*curvature
        location.x -= radius * math.cos(heading + pi_half)    
        location.y -= radius * math.sin(heading + pi_half)

        return location

    def get_road(self, roads, road_id):
        """
        Get a road from a map.roads by its id
        Args:
            roads: (list) List of T4ac_Road()
            road_id: (int) 
        Returns:
            road: (T4ac_Road()) Road of the given id
        """
        for road in roads:
            if road.id == road_id:
                return road
        print("get_road Error: Road not found!")

    def get_lane(self, road, lane_id, laneSectionValue):
        """
        Get a lane from a road by its id
        Args:
            road: (T4ac_Road) road containing the lane
            lane_id: (int)
            laneSectionValue: (int) or (str) 'unknown'
        Returns: 
            lane: (T4ac_Lane)
        """
        if laneSectionValue == 'unknown':
            if lane_id < 0:
                laneSection = 0
            elif lane_id > 0:
                laneSection = -1
        else:
            laneSection = laneSectionValue

        for lane in (road.lanes.laneSections[laneSection].right + 
                    road.lanes.laneSections[laneSection].left):
            if lane.id == lane_id:
                return lane

    def get_lane_change(self, road, lane_id):
        """
        Returns if lane change is possible for this lane, depending on
        the roadmark and if the left/right lane exists. 
        Values for lane change are:
            1) none
            2) left
            3) right
            4) both
        Args:
            road    : (T4ac_Road)
            lane_id : (int)
        Returns:
            lane_change: (str)
        Summary of How LaneChange works in XODR standard:
        Whatever the travel direction of the current lane, the roadMark that must
        be checked is:
            - right lane change : current lane id      (current_lane)
            - left lane change  : current lane id - 1  (decrement_lane)
        """

        current_lane = self.get_lane(road, lane_id, 0)
        decrement_lane = self.get_lane(road, lane_id - 1, 0)
        increment_lane =  self.get_lane(road, lane_id + 1, 0)
        # Must check all the roadmark segments, for possible bug 
        # in xodr definition
        incrementLane_laneChange = "none"
        decrementLane_laneChange = "none"
        currentLane_laneChange = "none"
        if current_lane is not None and current_lane.type == "driving":
            for roadMark in current_lane.roadMark:          
                if roadMark.laneChange == "both":
                    currentLane_laneChange = "both"
                    break

                elif roadMark.laneChange == "increase":
                    currentLane_laneChange = "increase"
                    break

                elif roadMark.laneChange == "decrease":
                    currentLane_laneChange = "decrease"
                    break
        
        if decrement_lane is not None and decrement_lane.type == "driving":
            for roadMark in decrement_lane.roadMark:
                if roadMark.laneChange == "both":
                    decrementLane_laneChange = "both"
                    break

                elif roadMark.laneChange == "increase":
                    decrementLane_laneChange = "increase"
                    break

                elif roadMark.laneChange == "decrease":
                    decrementLane_laneChange = "decrease"
                    break

        if increment_lane is not None and increment_lane.type == "driving":
            for roadMark in increment_lane.roadMark:
                if roadMark.laneChange == "both":
                    incrementLane_laneChange = "both"
                    break

                elif roadMark.laneChange == "increase":
                    incrementLane_laneChange = "increase"
                    break

                elif roadMark.laneChange == "decrease":
                    incrementLane_laneChange = "decrease"
                    break

        # Get the lane change
        # For positive travel direction
        if lane_id < 0:
            if ((incrementLane_laneChange == "increase" or incrementLane_laneChange == "both") and
                (currentLane_laneChange == "increase" or currentLane_laneChange == "none")):
                return "left"

            elif ((incrementLane_laneChange == "decrease" or incrementLane_laneChange == "none") and
                (currentLane_laneChange == "decrease" or currentLane_laneChange == "both")):
                return "right"

            elif ((incrementLane_laneChange == "increase" or incrementLane_laneChange == "both") and
                (currentLane_laneChange == "decrease" or currentLane_laneChange == "both")):
                return "both"

            else:
                return "none"

        # For negative traver direction
        elif lane_id > 0:
            if ((currentLane_laneChange == "increase" or currentLane_laneChange == "both") and
                (decrementLane_laneChange == "increase" or decrementLane_laneChange == "none")):
                return "right"

            elif ((currentLane_laneChange == "decrease" or currentLane_laneChange == "none") and
                (decrementLane_laneChange == "decrease" or decrementLane_laneChange == "both")):
                return "left"

            elif ((currentLane_laneChange == "increase" or currentLane_laneChange == "both") and
                (decrementLane_laneChange == "decrease" or decrementLane_laneChange == "both")):
                return "both"

            else:
                return "none"

    def get_junction(self, junctions, junction_id):
        """
        Get a junction from a map.junctions by its id
        Args:
            junctions: (list) List of T4ac_Junction()
            junction_id: (int) 
        Returns:
            junction: (T4ac_Junction()) Junction of the given id
        """
        for junction in junctions:
            if junction.id == junction_id:
                return junction
        print("get_junction Error: Junction not found!")

    def get_direction(self, road_id):
        """
        Get driving direction of the road, positive or negative, 
        according to the predecessor --> successor sequence

        Args:
            road: (int) id of the road

        Returns:
            (str) '+' or '-' defining driving direction
        """
        road = self.get_road(road_id)
        if (len(road.lanes.laneSections[0].right) > 0):
            for lane in road.lanes.laneSections[0].right:
                if (lane.type == "driving" and lane.id < 0):
                    return "+"
        if (len(road.lanes.laneSections[0].left) > 0):
            for lane in road.lanes.laneSections[0].left:
                if (lane.type == "driving" and lane.id > 0):
                    return "-"

    def xyz_to_roadlane(self, x, y, z):
        """
        Receives a location an get its corresponding road and lane, because 
        global path planning need road/lane tuple as input. If the location 
        is not inside a lane/road, returns the closer one inside a lane/road.

        Args:
            x: (int)
            y: (int)
            z: (int)

        Returns:
            closer_roadlane: (tuple) Closer road/lane to the given location
        """
        closer_waypoint = self.get_waypoint(x, y, z, project_to_road=True)
        closer_roadlane = (int(closer_waypoint.road_id), 
                           int(closer_waypoint.lane_id))

        return closer_roadlane

    def get_initial_position(self, road, lane, side):
        """
        Returns initial position of the road centered at the lane

        Args:
            road: (T4ac_Road())
            lane: (T4ac_Lane())
            side: (str) left or right 

        Returns: 
            location: (T4ac_Location()) 
        """
        # First get offset from the center of the lane to the center of the road
        lane_width_offset = 0
        if (side == 'left'):
            for i in range(0, abs(lane.id)):
                # -1-i because the list must be checked inside out
                lane_width_offset += road.lanes.laneSections[0].left[-1-i].width[0].a
            # total lane offset for lane origin
            lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a
        elif (side == 'right'):
            for i in range(0, abs(lane.id)):
                lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
            # total lane offset for lane origin
            lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a

        location = self.get_point_in_line(road.planView[0].x,
                                    road.planView[0].y,
                                    0,
                                    lane_offset,
                                    road.planView[0].hdg + math.pi/2)
        location.x = round(location.x, 3)
        location.y = round(location.y, 3)
        location.z = round(location.z, 3)

        return location 

    def get_inverted_initial_position(self, road, lane, side):
        """
        Return last position of the road. It can be used to get the origin 
        position for inverted roads

        Args:
            road: (T4ac_Road())
            lane: (T4ac_Lane())
            side: (str) left or right 

        Returns: 
            location: (T4ac_Location()) 
        """
        # First get offset from the center of the lane to the center of the road
        lane_width_offset = 0
        if (side == 'left'):
            for i in range(0, abs(lane.id)):
                # -1-i because the list must be checked inside out
                lane_width_offset += road.lanes.laneSections[0].left[-1-i].width[0].a
            # total lane offset for lane origin
            lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a
        elif (side == 'right'):
            for i in range(0, abs(lane.id)):
                lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
            # total lane offset for lane origin
            lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a

        # The get location centered on the road
        if (road.planView[-1].type == "line"):
            location_aux = self.get_point_in_line(road.planView[-1].x,
                                        road.planView[-1].y,
                                        0,
                                        road.planView[-1].length,
                                        road.planView[-1].hdg,
                                        )
        elif (road.planView[-1].type == "arc"):
            location_aux = self.get_point_in_arc(road.planView[-1].x,
                                        road.planView[-1].y,
                                        0,
                                        road.planView[-1].length,
                                        road.planView[-1].curvature,
                                        road.planView[-1].hdg,
                                        )
        # Apply offset
        location = self.get_point_in_line(location_aux.x,
                                    location_aux.y,
                                    0,
                                    lane_offset,
                                    road.planView[-1].hdg + math.pi/2)

        # Round value 
        location.x = round(location.x, 3)
        location.y = round(location.y, 3)
        location.z = round(location.z, 3)

        return location

    def get_lane_action(self, current, next):
        """
        Gets type of action to do between current (road, lane) and next 
        (road, lane). Type of action can be:
            - lanefollow
            - changeright
            - changeleft

        Args:
            current: (tuple) Current (road, lane)
            next: (tuple) Next (road, lane)

        Returns:
            action: (str) Type of action to do 
        """
        # If current road == next road, but lanes are different, means lanechange

        # Case of lanechange and '-' road direction
        if (current[0] == next[0] and current[1] > 0):
            if (current[1] > next[1]):
                action = "changeleft"
            else:
                action = "changeright"

        # Case of lanechange and '+' road direction
        elif (current[0] == next[0] and current[1] < 0):
            if (current[1] > next[1]):
                action = "changeright"
            else:
                action = "changeleft"

        # Case of lanefollow
        else:
            action = "lanefollow"

        return action

    def get_topology_waypoints(self, roads):
        """
        *DEV: This function has not been properly tested*

        Return a list of pairs of wp [[wp0,wp1],[wp2,wp3],...,[wpn, wpn+1]] 
        with in and out wp for every lane centered on the lane
        """
        topology_tuples = []
        for road in roads:
            # Get left lanes info
            if (road.lanes.laneSections[0].left):
                for lane in road.lanes.laneSections[0].left:
                    if (lane.type == "driving"):
                        waypoint_tuple = []
                        # lane width offset for lane origin
                        lane_width_offset = 0
                        for i in range(0, abs(lane.id)):
                            lane_width_offset += road.lanes.laneSections[0].left[-1-i].width[0].a # -1-i because the list must be checked inside out
                        # total lane offset for lane origin
                        lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a
                        # lane origin center
                        waypoint_lane_origin_center = T4ac_Waypoint()
                        waypoint_lane_origin_center.transform.location = self.get_point_in_line(road.planView[0].x, road.planView[0].y, 0, lane_offset, road.planView[0].hdg+math.pi/2)
                        waypoint_lane_origin_center.transform.location.z = self.calculate_elevation(road.elevationProfile[0].a, road.elevationProfile[0].b, road.elevationProfile[0].c, road.elevationProfile[0].d, road.elevationProfile[0].s, 0)
                        waypoint_lane_origin_center.road_id = road.id
                        waypoint_lane_origin_center.lane_id = lane.id
                        waypoint_tuple.append(waypoint_lane_origin_center)
                        # lane end center   
                                                        
                        if (road.planView[-1].type == "line"):                             
                            waypoint_lane_end_center = T4ac_Waypoint()
                            aux_location = self.get_point_in_line(road.planView[-1].x, road.planView[-1].y, 0, lane_offset, road.planView[-1].hdg+math.pi/2)
                            waypoint_lane_end_center.transform.location = self.get_point_in_line(aux_location.x, aux_location.y, 0, road.planView[-1].length, road.planView[-1].hdg)
                            waypoint_lane_end_center.transform.location.z = self.calculate_elevation(road.elevationProfile[-1].a, road.elevationProfile[-1].b, road.elevationProfile[-1].c, road.elevationProfile[-1].d, road.elevationProfile[-1].s, road.planView[-1].length)
                            waypoint_lane_end_center.road_id = road.id
                            waypoint_lane_end_center.lane_id = lane.id
                            waypoint_tuple.insert(0, waypoint_lane_end_center)
                        
                        elif (road.planView[-1].type == "arc"):
                            radius = 1/road.planView[-1].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[-1].length/radius
                            new_length = k_arc*new_radius
                            
                            waypoint_lane_end_center = T4ac_Waypoint()
                            aux_location = self.get_point_in_line(road.planView[-1].x, road.planView[-1].y, 0, lane_offset, road.planView[-1].hdg+math.pi/2)
                            waypoint_lane_end_center.transform.location = self.get_point_in_arc(aux_location.x, aux_location.y, 0, new_length, new_curvature, road.planView[-1].hdg)
                            waypoint_lane_end_center.transform.location.z = self.calculate_elevation(road.elevationProfile[-1].a, road.elevationProfile[-1].b, road.elevationProfile[-1].c, road.elevationProfile[-1].d, road.elevationProfile[-1].s, road.planView[-1].length)
                            waypoint_lane_end_center.road_id = road.id
                            waypoint_lane_end_center.lane_id = lane.id
                            waypoint_tuple.insert(0, waypoint_lane_end_center)
                        topology_tuples.append(waypoint_tuple)
                        
            # Get right lanes info
            if (road.lanes.laneSections[0].right):
                for lane in road.lanes.laneSections[0].right:
                    if (lane.type == "driving"):
                        waypoint_tuple = []
                        # lane width offset for lane origin
                        lane_width_offset = 0
                        for i in range(0, abs(lane.id)):
                            lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
                        # total lane offset for lane origin
                        lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a
                        # lane origin center
                        waypoint_lane_origin_center = T4ac_Waypoint()
                        waypoint_lane_origin_center.transform.location = self.get_point_in_line(road.planView[0].x, road.planView[0].y, 0, lane_offset, road.planView[0].hdg+math.pi/2)
                        waypoint_lane_origin_center.transform.location.z = self.calculate_elevation(road.elevationProfile[0].a, road.elevationProfile[0].b, road.elevationProfile[0].c, road.elevationProfile[0].d, road.elevationProfile[0].s, 0)
                        waypoint_lane_origin_center.road_id = road.id
                        waypoint_lane_origin_center.lane_id = lane.id
                        waypoint_tuple.append(waypoint_lane_origin_center)
                        # lane end center  
                                                        
                        if (road.planView[-1].type == "line"):                             
                            waypoint_lane_end_center = T4ac_Waypoint()
                            aux_location = self.get_point_in_line(road.planView[-1].x, road.planView[-1].y, 0, lane_offset, road.planView[-1].hdg+math.pi/2)
                            waypoint_lane_end_center.transform.location = self.get_point_in_line(aux_location.x, aux_location.y, 0, road.planView[-1].length, road.planView[-1].hdg)
                            waypoint_lane_end_center.transform.location.z = self.calculate_elevation(road.elevationProfile[-1].a, road.elevationProfile[-1].b, road.elevationProfile[-1].c, road.elevationProfile[-1].d, road.elevationProfile[-1].s, road.planView[-1].length)
                            waypoint_lane_end_center.road_id = road.id
                            waypoint_lane_end_center.lane_id = lane.id
                            waypoint_tuple.append(waypoint_lane_end_center)
                        
                        elif (road.planView[-1].type == "arc"):
                            radius = 1/road.planView[-1].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[-1].length/radius
                            new_length = k_arc*new_radius
                            
                            waypoint_lane_end_center = T4ac_Waypoint()
                            aux_location = self.get_point_in_line(road.planView[-1].x, road.planView[-1].y, 0, lane_offset, road.planView[-1].hdg+math.pi/2)
                            waypoint_lane_end_center.transform.location = self.get_point_in_arc(aux_location.x, aux_location.y, 0, new_length, new_curvature, road.planView[-1].hdg)
                            waypoint_lane_end_center.transform.location.z = self.calculate_elevation(road.elevationProfile[-1].a, road.elevationProfile[-1].b, road.elevationProfile[-1].c, road.elevationProfile[-1].d, road.elevationProfile[-1].s, road.planView[-1].length)
                            waypoint_lane_end_center.road_id = road.id
                            waypoint_lane_end_center.lane_id = lane.id
                            waypoint_tuple.append(waypoint_lane_end_center)
                        topology_tuples.append(waypoint_tuple)
                        
        return topology_tuples

    def generate_waypoints(self, roads, distance):
        """
        Generate waypoints centered in every driving lane given a distance

        Args:
            roads   : (list) List of the T4ac_Road objects of the map
            distance: (int)  Distance in meters between waypoints that 
                             are generated
        """
        # @ADD 07/11/20 : Fix elevation (z inertial)
        # @ADD 06/04/22 : Adapt code to the refactoring (add self 
        # to the methods)

        waypoints = []
        for road in roads:
            # Get left lanes info
            if (road.lanes.laneSections[-1].left):
                for lane in road.lanes.laneSections[-1].left:
                    if (lane.type == "driving"):
                        # lane width offset for lane origin
                        lane_width_offset = 0
                        for i in range(0, abs(lane.id)):
                            lane_width_offset += road.lanes.laneSections[-1].left[-1-i].width[-1].a # -1-i because the list must be checked inside out
                        # total lane offset for lane origin
                        lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[-1].a
                        # For each lane go through every geometry generating waypoints
                        for geometry in road.planView:
                            # lane origin center
                            waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                            waypoint_geometry_origin_lane_center.s = geometry.s
                            waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(geometry.x, geometry.y, 0, lane_offset, geometry.hdg+math.pi/2)
                            waypoint_geometry_origin_lane_center.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint_geometry_origin_lane_center.s)
                            waypoint_geometry_origin_lane_center.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                            waypoint_geometry_origin_lane_center.lane_width = lane.width[0].a
                            waypoint_geometry_origin_lane_center.road_id = road.id
                            waypoint_geometry_origin_lane_center.lane_id = lane.id
                            waypoint_geometry_origin_lane_center.junction = road.junction
                            waypoint_geometry_origin_lane_center.vmax = road.type.speed.max
                            waypoint_geometry_origin_lane_center.vunit = road.type.speed.unit
                            (waypoint_geometry_origin_lane_center.nLanes, 
                            waypoint_geometry_origin_lane_center.lanePosition) = (
                                waypoint_geometry_origin_lane_center.get_lanePosition(lane.id, road))
                            waypoint_geometry_origin_lane_center.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint_geometry_origin_lane_center)
                            
                            if (geometry.type == "line"):   
                                n = int(geometry.length/distance)                           
                                for n_dist in range(0,n):
                                    waypoint = T4ac_Waypoint()
                                    waypoint.s = geometry.s + n_dist*distance
                                    waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, geometry.hdg)
                                    waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                    waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                                    waypoint.lane_width = lane.width[0].a
                                    waypoint.road_id = road.id
                                    waypoint.lane_id = lane.id
                                    waypoint.junction = road.junction
                                    waypoint.vmax = road.type.speed.max
                                    waypoint.vunit = road.type.speed.unit
                                    waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                    waypoint.lane_change = self.get_lane_change(road, lane.id)
                                    waypoints.append(waypoint)
                                waypoint = T4ac_Waypoint()
                                waypoint.s = geometry.s + geometry.length
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, geometry.length, geometry.hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                           
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                            elif (geometry.type == "arc"):
                                radius = 1/geometry.curvature
                                new_radius = radius - lane_offset
                                new_curvature = 1/new_radius
                                k_arc = geometry.length/radius
                                new_length = k_arc*new_radius
                                n = int(new_length/distance)  
                                for n_dist in range(0,n):
                                    waypoint = T4ac_Waypoint()
                                    waypoint.s = geometry.s + n_dist*distance
                                    waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, geometry.hdg)
                                    waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                    waypoint.transform.rotation.yaw = (geometry.hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                    waypoint.lane_width = lane.width[0].a
                                    waypoint.road_id = road.id
                                    waypoint.lane_id = lane.id                
                                    waypoint.junction = road.junction
                                    waypoint.vmax = road.type.speed.max
                                    waypoint.vunit = road.type.speed.unit
                                    waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                    waypoint.lane_change = self.get_lane_change(road, lane.id)
                                    waypoints.append(waypoint)
                                
                                waypoint = T4ac_Waypoint()
                                waypoint.s = geometry.s + new_length
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, geometry.hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (geometry.hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                          
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
            # Get right lanes info
            if (road.lanes.laneSections[0].right):
                for lane in road.lanes.laneSections[0].right:
                    if (lane.type == "driving"):
                        # lane width offset for lane origin
                        lane_width_offset = 0
                        for i in range(0, abs(lane.id)):
                            lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
                        # total lane offset for lane origin
                        lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a
                        # For each lane go through every geometry generating waypoints
                        for geometry in road.planView:
                            n = int(geometry.length/distance)
                            # lane origin center
                            waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                            waypoint_geometry_origin_lane_center.s = geometry.s
                            waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(geometry.x, geometry.y, 0, lane_offset, geometry.hdg+math.pi/2)
                            waypoint_geometry_origin_lane_center.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint_geometry_origin_lane_center.s)
                            waypoint_geometry_origin_lane_center.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                            waypoint_geometry_origin_lane_center.lane_width = lane.width[0].a
                            waypoint_geometry_origin_lane_center.road_id = road.id
                            waypoint_geometry_origin_lane_center.lane_id = lane.id                       
                            waypoint_geometry_origin_lane_center.junction = road.junction
                            waypoint_geometry_origin_lane_center.vmax = road.type.speed.max
                            waypoint_geometry_origin_lane_center.vunit = road.type.speed.unit
                            (waypoint_geometry_origin_lane_center.nLanes, 
                            waypoint_geometry_origin_lane_center.lanePosition) = (
                                waypoint_geometry_origin_lane_center.get_lanePosition(lane.id, road))
                            waypoint_geometry_origin_lane_center.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint_geometry_origin_lane_center)
                            
                            if (geometry.type == "line"): 
                                n = int(geometry.length/distance)                             
                                for n_dist in range(0,n):
                                    waypoint = T4ac_Waypoint()
                                    waypoint.s = geometry.s + n_dist*distance
                                    waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, geometry.hdg)
                                    waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                    waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                                    waypoint.lane_width = lane.width[0].a
                                    waypoint.road_id = road.id
                                    waypoint.lane_id = lane.id                                
                                    waypoint.junction = road.junction
                                    waypoint.vmax = road.type.speed.max
                                    waypoint.vunit = road.type.speed.unit
                                    waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                    waypoint.lane_change = self.get_lane_change(road, lane.id)
                                    waypoints.append(waypoint)
                                waypoint = T4ac_Waypoint()
                                waypoint.s = geometry.s + geometry.length
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, geometry.length, geometry.hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                           
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                            
                            elif (geometry.type == "arc"):
                                radius = 1/geometry.curvature
                                new_radius = radius - lane_offset
                                new_curvature = 1/new_radius
                                k_arc = geometry.length/radius
                                new_length = k_arc*new_radius
                                n = int(new_length/distance)
                                for n_dist in range(0,n):
                                    waypoint = T4ac_Waypoint()
                                    waypoint.s = geometry.s + n_dist*distance
                                    waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, geometry.hdg)
                                    waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                    waypoint.transform.rotation.yaw = (geometry.hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                    waypoint.lane_width = lane.width[0].a
                                    waypoint.road_id = road.id
                                    waypoint.lane_id = lane.id                               
                                    waypoint.junction = road.junction
                                    waypoint.vmax = road.type.speed.max
                                    waypoint.vunit = road.type.speed.unit
                                    waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                    waypoint.lane_change = self.get_lane_change(road, lane.id)
                                    waypoints.append(waypoint)
                                
                                waypoint = T4ac_Waypoint()
                                waypoint.s = geometry.s + new_length
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, geometry.hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (geometry.hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
        return waypoints

    def generate_waypoints_in_lane(self, road, lane, distance):
        """
        Generate waypoints centered in a driving lane given a distance
        """
        waypoints = []
        if lane.id > 0: #left side --> road s reference line is inverted
            if (lane.type == "driving"):
                # lane width offset for lane origin
                lane_width_offset = 0
                for i in range(0, abs(lane.id)):
                    lane_width_offset += road.lanes.laneSections[-1].left[-1-i].width[0].a # -1-i because the list must be checked inside out
                # total lane offset for lane origin
                if len(road.lanes.laneSections) > 1:
                    lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[-1].a
                elif len(road.lanes.laneSections) == 1:
                    lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a
                # For each lane go through every geometry generating waypoints
                for geometry in reversed(road.planView):
                    # lane origin center
                    waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(geometry.x, geometry.y, 0, lane_offset, geometry.hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = geometry.s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    #waypoints.append(waypoint_geometry_origin_lane_center)
                    
                    if (geometry.type == "line"):  
                        waypoint = T4ac_Waypoint()
                        waypoint.s = geometry.s + geometry.length
                        waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, geometry.length, geometry.hdg)
                        waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                        waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                        waypoint.lane_change = self.get_lane_change(road, lane.id)
                        # waypoints.append(waypoint) 
                        n = int(geometry.length/distance)                           
                        for n_dist in range(n,-1,-1):
                            waypoint = T4ac_Waypoint()
                            waypoint.s = geometry.s + n_dist*distance
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, geometry.hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                    
                    elif (geometry.type == "arc"):
                        radius = 1/geometry.curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = geometry.length/radius
                        new_length = k_arc*new_radius
                        waypoint = T4ac_Waypoint()
                        waypoint.s = geometry.s + new_length
                        waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, geometry.hdg)
                        waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                        waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id                       
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                        waypoint.lane_change = self.get_lane_change(road, lane.id)
                        # waypoints.append(waypoint)
                        n = int(new_length/distance)  
                        for n_dist in range(n,-1,-1):
                            waypoint = T4ac_Waypoint()
                            waypoint.s = geometry.s + n_dist*distance
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, geometry.hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (geometry.hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                return waypoints

        elif lane.id < 0: #right side
            if (lane.type == "driving"):
                # lane width offset for lane origin
                lane_width_offset = 0
                for i in range(0, abs(lane.id)):
                    lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
                # total lane offset for lane origin
                lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a
                # For each lane go through every geometry generating waypoints
                for geometry in road.planView:
                    n = int(geometry.length/distance)
                    # lane origin center
                    waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(geometry.x, geometry.y, 0, lane_offset, geometry.hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = geometry.s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    #waypoints.append(waypoint_geometry_origin_lane_center)
                    
                    if (geometry.type == "line"): 
                        n = int(geometry.length/distance)                             
                        for n_dist in range(0,n):
                            waypoint = T4ac_Waypoint()
                            waypoint.s = geometry.s + n_dist*distance
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, geometry.hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        waypoint = T4ac_Waypoint()
                        waypoint.s = geometry.s + geometry.length
                        waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, geometry.length, geometry.hdg)
                        waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                        waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                        waypoint.lane_change = self.get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                    
                    
                    elif (geometry.type == "arc"):
                        radius = 1/geometry.curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = geometry.length/radius
                        new_length = k_arc*new_radius
                        n = int(new_length/distance)
                        for n_dist in range(0,n):
                            waypoint = T4ac_Waypoint()
                            waypoint.s = geometry.s + n_dist*distance
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, geometry.hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (geometry.hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        waypoint = T4ac_Waypoint()
                        waypoint.s = geometry.s + new_length
                        waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, geometry.hdg)
                        waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                        waypoint.transform.rotation.yaw = (geometry.hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                        waypoint.lane_change = self.get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                return waypoints

    def generate_next_waypoints_until_lane_end(self, road, lane, distance, s_0):
        """
        Generate waypoints centered in a driving lane given a distance, from s_0 to end
        """
        waypoints = []
        # If left side, next wps have lower s (s is inverted)
        if lane.id > 0: #left side
            if (lane.type == "driving"):
                # lane width offset for lane origin
                lane_width_offset = 0
                for i in range(0, abs(lane.id)):
                    lane_width_offset += road.lanes.laneSections[-1].left[-1-i].width[0].a # -1-i because the list must be checked inside out
                # total lane offset for lane origin
                if len(road.lanes.laneSections) > 1:
                    lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[-1].a
                elif len(road.lanes.laneSections) == 1:
                    lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a
                
                # First locate in which geometry is s_0
                geometry_index = 0
                for i in range(0, len(road.planView)):
                    if s_0 > road.planView[i].s:
                        geometry_index = i
                    else:
                        break
                # Then generate every wp from this s_0 in this geometry_index to origin (because in left side s is inverted)
                for i in range(geometry_index, -1,-1):
                    # For first geometry get only wps from s_0
                    if i == geometry_index:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"):   
                            waypoint = T4ac_Waypoint()
                            waypoint.s = s_0 - road.planView[i].s
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0-road.planView[i].s, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                            n = int((s_0-road.planView[i].s)/distance)                           
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            waypoint = T4ac_Waypoint()
                            waypoint.s = s_0 - road.planView[i].s
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0-road.planView[i].s, new_curvature, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                          
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                            n = int((s_0-road.planView[i].s)/distance)
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                                
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                            

                    # For rest of geometries get every wp
                    else:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"):   
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + road.planView[i].length
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                      
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                            n = int(road.planView[i].length/distance)                           
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                                
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + new_length
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                           
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                            n = int(new_length/distance)  
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                 
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                return waypoints

        # If right side, next wps have higher s 
        elif lane.id < 0: #right side
            if (lane.type == "driving"):
                # lane width offset for lane origin
                lane_width_offset = 0
                for i in range(0, abs(lane.id)):
                    lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
                # total lane offset for lane origin
                lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a

                # First locate in which geometry is s_0
                geometry_index = 0
                for i in range(0, len(road.planView)):
                    if s_0 > road.planView[i].s:
                        geometry_index = i
                    else:
                        break

                # Then generate every wp from this s_0 in this geometry_index to end 
                for i in range(geometry_index, len(road.planView)):

                    if i == geometry_index:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"): 

                            n = int((road.planView[i].length+road.planView[i].s-s_0)/distance)                             
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = s_0 + (n_dist*distance)
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0-waypoint_geometry_origin_lane_center.s+(n_dist*distance), road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                              
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + road.planView[i].length
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                           
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            n = int((new_length+road.planView[i].s-s_0)/distance)
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = s_0 + n_dist*distance
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0-waypoint_geometry_origin_lane_center.s+(n_dist*distance), new_curvature, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                               
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + new_length
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                          
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)

                    
                    else:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"): 
                            n = int(road.planView[i].length/distance)                             
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                               
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + road.planView[i].length
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                            #waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                            
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            n = int(new_length/distance)
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                                
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + new_length
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                           
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                return waypoints

    def generate_previous_waypoints_until_lane_start(self, road, lane, distance, s):
        """
        Generate waypoints centered in a driving lane given a distance, from start to s
        """
        waypoints = []
        # If left side, next wps have lower s (s is inverted)
        if lane.id > 0: #left side
            if (lane.type == "driving"):
                # lane width offset for lane origin
                lane_width_offset = 0
                for i in range(0, abs(lane.id)):
                    lane_width_offset += road.lanes.laneSections[-1].left[-1-i].width[0].a # -1-i because the list must be checked inside out
                # total lane offset for lane origin
                if len(road.lanes.laneSections) > 1:
                    lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[-1].a
                elif len(road.lanes.laneSections) == 1:
                    lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a

                # First locate in which geometry is s
                geometry_index = 0
                for i in range(0, len(road.planView)):
                    if s > road.planView[i].s:
                        geometry_index = i
                    else:
                        break
                # Then generate every wp from this s in this geometry_index to end (because in left side s is inverted)
                for i in range(len(road.planView)-1, geometry_index-1, -1):
                    # For first geometry get only wps from s
                    if i == geometry_index:
                            # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"):   
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + road.planView[i].length
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                            
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                            n = int((road.planView[i].s+road.planView[i].length-s)/distance)                           
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, (s-waypoint_geometry_origin_lane_center.s)+(n_dist*distance), road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                               
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + road.planView[i].length
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, new_curvature, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                            
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                            n = int((road.planView[i].s+road.planView[i].length-s)/distance)  
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, (s-waypoint_geometry_origin_lane_center.s)+(n_dist*distance), new_curvature, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                                
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                            

                    # For rest of geometries get every wp
                    else:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"):   
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + road.planView[i].length
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                            
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                            n = int(road.planView[i].length/distance)                           
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                                
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + new_length
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                            
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                            n = int(new_length/distance)  
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                                
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                return waypoints

        # If right side, next wps have higher s 
        elif lane.id < 0: #right side
            if (lane.type == "driving"):
                # lane width offset for lane origin
                lane_width_offset = 0
                for i in range(0, abs(lane.id)):
                    lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
                # total lane offset for lane origin
                lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a

                # First locate in which geometry is s
                geometry_index = 0
                for i in range(0, len(road.planView)):
                    if s > road.planView[i].s:
                        geometry_index = i
                    else:
                        break

                # Then generate every wp from origin to this s in geometry_index 
                for i in range(0, geometry_index+1):

                    if i == geometry_index:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"): 

                            n = int((s-road.planView[i].s)/distance)                             
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + (n_dist*distance)
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                                
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            waypoint = T4ac_Waypoint()
                            waypoint.s = s
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s-road.planView[i].s, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                            
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            n = int((s-road.planView[i].s)/distance)
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + (n_dist*distance)
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                                
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                            waypoint = T4ac_Waypoint()
                            waypoint.s = s
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s-road.planView[i].s, new_curvature, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                           
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)

                    
                    else:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"): 
                            n = int(road.planView[i].length/distance)                             
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                               
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + road.planView[i].length
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                            #waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                            
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            n = int(new_length/distance)
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                                waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                                
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                                waypoint.lane_change = self.get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                            waypoint = T4ac_Waypoint()
                            waypoint.s = road.planView[i].s + new_length
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                            waypoint.transform.location.z = self.calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                            
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.nLanes, waypoint.lanePosition = waypoint.get_lanePosition(lane.id, road)
                            waypoint.lane_change = self.get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                return waypoints

    # @TODO
    def generate_waypoints_from_to(self, road, lane, distance, s_0, s):
        """
        Generate waypoints centered in a driving lane given a distance, from s_0 to s
        """
        waypoints = []
        # If left side, next wps have lower s (s is inverted)
        if lane.id > 0: #left side
            if (lane.type == "driving"):
                # lane width offset for lane origin
                lane_width_offset = 0
                for i in range(0, abs(lane.id)):
                    lane_width_offset += road.lanes.laneSections[0].left[-1-i].width[0].a # -1-i because the list must be checked inside out
                # total lane offset for lane origin
                lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a

                # Locate in which geometry is s_0
                geometry_index_s_0 = 0
                for i in range(0, len(road.planView)):
                    if s_0 > road.planView[i].s:
                        geometry_index_s_0 = i
                    else:
                        break

                # Locate in which geometry is s
                geometry_index_s = 0
                for i in range(0, len(road.planView)):
                    if s_0 > road.planView[i].s:
                        geometry_index_s = i
                    else:
                        break

                # Then generate every wp from this s_0 in this geometry_index to s (because in left side s is inverted)
                for i in range(geometry_index_s_0, geometry_index_s,-1):
                    # Case of first geometry get only wps from s_0
                    if i == geometry_index_s_0:
                            # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"):   
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = s_0
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                            n = int(s_0/distance)                           
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)
                            
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = s_0
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                            n = int(s_0/distance)  
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)
                            
                            
                    # Case of last geometry get until s
                    elif i == geometry_index_s:    
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"):   
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = s_0
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                            n = int(s_0/distance)                           
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)
                            
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = s_0
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                            n = int(s_0/distance)  
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)

                    # case of rest of geometries get every wp
                    else:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"):   
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + road.planView[i].length
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                            n = int(road.planView[i].length/distance)                           
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)
                            
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + new_length
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                            n = int(new_length/distance)  
                            for n_dist in range(n,-1,-1):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)
                            
                return waypoints

        # If right side, next wps have higher s 
        elif lane.id < 0: #right side
            if (lane.type == "driving"):
                # lane width offset for lane origin
                lane_width_offset = 0
                for i in range(0, abs(lane.id)):
                    lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
                # total lane offset for lane origin
                lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a

                # Locate in which geometry is s_0
                for i in range(0, len(road.planView)):
                    if s_0 > road.planView[i].s:
                        geometry_index_s_0 = i
                    else:
                        break

                # Locate in which geometry is s
                for i in range(0, len(road.planView)):
                    if s_0 > road.planView[i].s:
                        geometry_index_s = i
                    else:
                        break

                # Then generate every wp from this s_0 in this geometry_index to s 
                for i in range(geometry_index_s_0, geometry_index_s):

                    # Case of first geometry 
                    if i == geometry_index_s_0:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"): 

                            n = int((road.planView[i].length-s_0)/distance)                             
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0+(n_dist*distance), road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = s_0 + (n_dist*distance)
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + road.planView[i].length
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            n = int((new_length-s_0)/distance)
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0+(n_dist*distance), new_curvature, road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = s_0 + n_dist*distance
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)
                            
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + new_length
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)

                    # Case of last geometry 
                    elif i == geometry_index_s:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"): 

                            n = int(s/distance)                             
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = road.planView[i].s + (n_dist*distance)
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + s
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            n = int(s/distance)
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = road.planView[i].s + (n_dist*distance)
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)
                            
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + s
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                    
                    # Case of any other geometry 
                    else:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = T4ac_Waypoint()
                        waypoint_geometry_origin_lane_center.transform.location = self.get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                        #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.s = road.planView[i].s
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        
                        if (road.planView[i].type == "line"): 
                            n = int(road.planView[i].length/distance)                             
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + road.planView[i].length
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        
                        
                        elif (road.planView[i].type == "arc"):
                            radius = 1/road.planView[i].curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = road.planView[i].length/radius
                            new_length = k_arc*new_radius
                            n = int(new_length/distance)
                            for n_dist in range(0,n):
                                waypoint = T4ac_Waypoint()
                                waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                                #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.s = road.planView[i].s + n_dist*distance
                                waypoint.junction = road.junction
                                waypoints.append(waypoint)
                            
                            waypoint = T4ac_Waypoint()
                            waypoint.transform.location = self.get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + new_length
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                return waypoints

    def calculate_elevation(self, a, b, c, d, s_0, s):
        """
        Return elevation (inertial z) at a given position (knowing the elevationProfile)
        """
        ds = s - s_0
        elevation = a + b*ds + c*ds**2 + d*ds**3
        return elevation

    def calculate_elevationProfile(self, elevationProfile, s):
        """
        Return elevation (inertial z) at a given position (not knowing which elevationProfile is)
        """
        if elevationProfile:
            for i in range(0,len(elevationProfile)):
                if elevationProfile[i].s > s: break
            if (i == len(elevationProfile)):
                profile = elevationProfile[i]
            else:
                profile = elevationProfile[i-1]

            elevation = self.calculate_elevation(profile.a, profile.b, profile.c, profile.d, profile.s, s)
            return elevation

    def get_road_by_id(self, road_id):
        """
        Get a specific road from a self.roads by its id

        Args:
            road_id: (int) 

        Returns:
            road: (T4ac_Road()) Road of the given id
        """
        for road in self.roads:
            if road.id == road_id:
                return road
        print("get_road Error: Road not found!")

    def get_lane_by_id(self, road_id, lane_id, laneSectionValue):
        """
        Get a specific lane from a road by its id

        Args:
            road_id: (int) road_id of the road containing the lane
            lane_id: (int)
            laneSectionValue: (int) or (str) 'unknown'

        Returns: 
            lane: (T4ac_Lane)
        """
        road = self.get_road_by_id(road_id)
        if laneSectionValue == 'unknown':
            if lane_id < 0:
                laneSection = 0
            elif lane_id > 0:
                laneSection = -1
        else:
            laneSection = laneSectionValue

        for lane in (road.lanes.laneSections[laneSection].right + 
                    road.lanes.laneSections[laneSection].left):
            if lane.id == lane_id:
                return lane

    def get_junction_by_id(self, junction_id):
        """
        Get a specific junction from a self.junctions by its id

        Args:
            junction_id: (int) 

        Returns:
            junction: (T4ac_Junction()) Junction of the given id
        """
        for junction in self.junctions:
            if junction.id == junction_id:
                return junction
        print("get_junction Error: Junction not found!")

    def get_lane_change_by_id(self, road_id, lane_id):
        """
        Returns if lane change is possible for this lane, depending on
        the roadmark and if the left/right lane exists. 

        Values for lane change are:
            1) None
            2) Left
            3) Right
            4) Both

        Args:
            road    : (T4ac_Road)
            lane_id : (int)

        Returns:
            lane_change: (str)

        Summary of How LaneChange works in XODR standard:
        Whatever the travel direction of the current lane, the roadMark that 
        must be checked is:
            - right lane change : current lane id      (current_lane)
            - left lane change  : current lane id - 1  (decrement_lane)
        """

        current_lane = self.get_lane_by_id(road_id, lane_id, 0)
        decrement_lane = self.get_lane_by_id(road_id, lane_id - 1, 0)
        increment_lane =  self.get_lane_by_id(road_id, lane_id + 1, 0)
        # Must check all the roadmark segments, for possible bug 
        # in xodr definition
        incrementLane_laneChange = "none"
        decrementLane_laneChange = "none"
        currentLane_laneChange = "none"
        if current_lane is not None and current_lane.type == "driving":
            for roadMark in current_lane.roadMark:          
                if roadMark.laneChange == "both":
                    currentLane_laneChange = "both"
                    break

                elif roadMark.laneChange == "increase":
                    currentLane_laneChange = "increase"
                    break

                elif roadMark.laneChange == "decrease":
                    currentLane_laneChange = "decrease"
                    break
        
        if decrement_lane is not None and decrement_lane.type == "driving":
            for roadMark in decrement_lane.roadMark:
                if roadMark.laneChange == "both":
                    decrementLane_laneChange = "both"
                    break

                elif roadMark.laneChange == "increase":
                    decrementLane_laneChange = "increase"
                    break

                elif roadMark.laneChange == "decrease":
                    decrementLane_laneChange = "decrease"
                    break

        if increment_lane is not None and increment_lane.type == "driving":
            for roadMark in increment_lane.roadMark:
                if roadMark.laneChange == "both":
                    incrementLane_laneChange = "both"
                    break

                elif roadMark.laneChange == "increase":
                    incrementLane_laneChange = "increase"
                    break

                elif roadMark.laneChange == "decrease":
                    incrementLane_laneChange = "decrease"
                    break

        # Get the lane change
        # For positive travel direction
        if lane_id < 0:
            if ((incrementLane_laneChange == "increase" or 
                 incrementLane_laneChange == "both") and
                (currentLane_laneChange   == "increase" or 
                 currentLane_laneChange   == "none")):
                return "left"

            elif ((incrementLane_laneChange   == "decrease" or 
                   incrementLane_laneChange   == "none") and
                  (currentLane_laneChange     == "decrease" or 
                   currentLane_laneChange     == "both")):
                return "right"

            elif ((incrementLane_laneChange == "increase" or 
                   incrementLane_laneChange == "both") and
                  (currentLane_laneChange   == "decrease" or 
                   currentLane_laneChange   == "both")):
                return "both"

            else:
                return "none"

        # For negative traver direction
        elif lane_id > 0:
            if ((currentLane_laneChange   == "increase" or 
                 currentLane_laneChange   == "both") and
                (decrementLane_laneChange == "increase" or 
                 decrementLane_laneChange == "none")):
                return "right"

            elif ((currentLane_laneChange   == "decrease" or 
                   currentLane_laneChange   == "none") and
                  (decrementLane_laneChange == "decrease" or 
                   decrementLane_laneChange == "both")):
                return "left"

            elif ((currentLane_laneChange   == "increase" or 
                   currentLane_laneChange   == "both") and
                  (decrementLane_laneChange == "decrease" or 
                   decrementLane_laneChange == "both")):
                return "both"

            else:
                return "none"


