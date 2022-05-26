#!/usr/bin/env python3
"""
Last mod: Alejandro D. 25/04/2022
--> Refactored to the new MapObject class

Generates a lane based graph using networkx, using an xodr map as an input.
Map parser is used to obtain the information from the xodr file.

Each edge of the graph is composed by its road id and its lane id. 

This script has been structured in a class to be imported in other scripts.
"""
import sys
import os
import math

import networkx as nx
# import matplotlib.pyplot as plt 

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
from t4ac_mapping_layer.map_parser.map_object import MapObject


class LaneGraphPlanner():
    """
    Args
        map_data: (str) It can be given in two ways:
                        0) Name of the map without extension, i.e. 'Town01' \
                        1) All map data saved as a string 
        map_path: (str) Path of the map files
        map_flag: (bool) 0) map_data in case 0
                         1) map data in case 1
    """

    def __init__(self, map_data, map_path, map_flag=0):

        self.map_object = MapObject(map_data, map_path, map_flag)
        
        # Constant to define the weight of lanechange
        self.LANECHANGECOST = 10

        ### Dev ###
        self.road_weights = {} # Dictionary for consulting faster the weights
        self.default_vmax = 25
        ### Dev ###

        # Parameters to build the graph using edges. edge_labels and pos is only for
        # the graph representation
        self.edges, self.edge_labels, self.pos = (
            self.calculate_lane_graph_connections(self.LANECHANGECOST))

        # Networkx DiGraph to use for global path planning
        self.G = self.create_lane_graph(self.edges, self.edge_labels, self.pos)

        # Waypoints centered in every lane of the map separated by a given distance
        self.roadNetwork_waypoints = self.map_object.generate_waypoints(
            self.map_object.roads, 2)

        # Map name 
        self.map_name = self.map_object.map_name

        

    def calculate_lane_link(self, road, incomingLane):
        """
        Calculates the lane link when there is more than one lane section. It
        depends if the lane is forward or backward.

        Args:
            road
            incomingLane
            direction: forward or backward

        Returns:
            outgoingLane_id: (int)
        """
        # 1) Get incomingEntryLane
        # 1.1) incomingLane direction is forward
        if incomingLane.id < 0:
            # Get entryLane
            incomingEntryLane = incomingLane
        # 1.2) incomingLane direction is backward
        elif incomingLane.id > 0:
            # Get entryLane
            # 1.2.1) Case of 1 single laneSection
            if len(road.lanes.laneSections) == 1:
                incomingEntryLane = incomingLane
            # 1.2.2) Case of multiple laneSections
            elif len(road.lanes.laneSections) > 1:
                incomingEntryLane = incomingLane
                for i in range(len(road.lanes.laneSections)-1):
                    next_lane_id = incomingEntryLane.link.successor.id
                    next_lane = self.map_object.get_lane(
                        road, next_lane_id, i+1)
                    incomingEntryLane = next_lane
            
        # 2) Get incomingExitLane 
        # 2.1) incomingLane direction is forward
        if incomingLane.id < 0:
            # 2.1.1) Case of 1 single laneSection
            if len(road.lanes.laneSections) == 1:
                incomingExitLane = incomingLane
            # 2.1.2) Case of multiple laneSections
            elif len(road.lanes.laneSections) > 1:
                incomingExitLane = incomingLane
                for i in range(len(road.lanes.laneSections)-1):
                    next_lane_id = incomingExitLane.link.successor.id
                    next_lane = self.map_object.get_lane(
                        road, next_lane_id, i+1)
                    incomingExitLane = next_lane
        # 2.2) incomingLane direction is backward
        elif incomingLane.id > 0:
            incomingExitLane = incomingLane

        # 3) Get outgoingEntryLane
        # 3.1) incomingLane direction is forward
        if incomingLane.id < 0:
            outgoingEntryLane_id = incomingExitLane.link.successor.id
        # 3.2) incomingLane direction is backward
        elif incomingLane.id > 0:
            outgoingEntryLane_id = incomingExitLane.link.predecessor.id

        return incomingEntryLane.id, outgoingEntryLane_id


    def next_lane_connection(self, road, lane, direction, laneSection):
        """
        Returns next lane of a lane section

        Args:
            road
            lane
            direction: forward or backward
            laneSection: (int)
        Returns:
            next_lane
        """

        if direction == "forward":
            next_lane_id = lane.link.successor.id
            next_lane = self.map_object.get_lane(road, next_lane_id, laneSection+1)
            return next_lane

        elif direction == "backward":
            next_lane_id = lane.link.successor.id
            next_lane = self.map_object.get_lane(road, next_lane_id, laneSection+1)
            return next_lane

        else:
            print("Warning! Direction in function 'next_lane_connection' \
                   when there is more than 1 laneSection is not valid")

    def calculate_lane_graph_connections(self, LANECHANGECOST):
        """
        Evaluate connections for every road/lane in the map to generate the graph.
        Graph is generated from edges, and an edge in generated from two nodes.
        A node is a tuple of two parameters : (road_id, lane_id).

        There is a first loop that evaluates every road in the map, and there is a 
        second nested loop to evaluate every lane of each road.

        Lane change is also considered.

        Args:
            LANECHANGECOST: weight for the lane change 

        Returns:
            edges, edge_labels, pos: parameters to build the graph using networkx
        """
        edges = []
        edge_labels = dict()
        pos = dict()
        for road in self.map_object.roads:
            # Get every lane in the road
            lanes = road.lanes.laneSections[0].right + road.lanes.laneSections[0].left
            flag_laneSections = 0

            # Check if there is more than 1 lane section
            if len(road.lanes.laneSections) > 1: 
                flag_laneSections = 1

            for lane in lanes:
                ### Dev ###

                if road.type.speed.max == 0: v_max = self.default_vmax
                else: v_max = road.type.speed.max

                # Check velocity units (convert from mph to kmh)
                if road.type.speed.unit == "mph": v_max = v_max*1.609

                # Create a dictionary for consulting faster the weights
                road_weight = {
                            'length' : round(road.length, 4),
                            'time'   : round(road.length/v_max, 4),
                            'vmax'   : round(v_max, 4)
                            }

                self.road_weights[road.id] = road_weight
                

                # Take successor as next connection for negative lane ids
                if (lane.id < 0 and lane.type == 'driving'):
                    # When successor is road get successor connection
                    if (road.link.successor.elementType == 'road'): 

                        incomingEntryLaneId, outgoingEntryLaneId = (
                            self.calculate_lane_link(road, lane))

                        # Create edge from current to successor
                        edge = ([(road.id, incomingEntryLaneId),
                            (road.link.successor.elementId, outgoingEntryLaneId),
                            {'length': road_weight['length'], 
                             'time'  : road_weight['time']}])

                        edges.append(edge)
                        # This part is only for visual purpose:
                        # Add new edge label
                        edge_labels[((edge[0]), (edge[1]))] = edge[2]['length']
                        # Add initial position of the road to pos dict
                        location = self.map_object.get_initial_position(road, lane, 'right')
                        pos[road.id, lane.id] = (location.x, location.y)

                    # When successor is junction get all possible connections
                    elif (road.link.successor.elementType == 'junction'):

                        junction = self.map_object.get_junction(
                            self.map_object.junctions, road.link.successor.elementId)
                        # Create edge from current to all possible connections
                        for connection in junction.connection:
                            if (connection.incomingRoad == road.id):
                                for laneLink in connection.laneLink:
                                    if laneLink.fromLane == lane.id:
                                        connectingLane = laneLink.toLane
                                        edge = ([(road.id, lane.id),
                                                (connection.connectingRoad, connectingLane),
                                                {'length': road_weight['length'], 
                                                 'time'  : road_weight['time']}])
                                        edges.append(edge)
                                        edge_labels[((edge[0]), (edge[1]))] = edge[2]['length']
                                        location = self.map_object.get_initial_position(road, lane, 'right')
                                        pos[road.id, lane.id] = (location.x, location.y)  

                    # Check lane changes
                    ### (Must check all the roadmark segments, for possible bug in xodr definition) ###
                    for roadMark in lane.roadMark:
                        laneChange_flag = "none"
                        if roadMark.laneChange == "both":
                            laneChange_flag = "both"
                            break
                    ### *** ###
                    if (laneChange_flag == "both"):
                        # Get left lane
                        left_lane = self.map_object.get_lane(road, lane.id - 1, 0)
                        if (left_lane is not None and left_lane.type == "driving"):
                            # Create edge from current to left lane
                            edge = ([(road.id, lane.id),
                                (road.id, left_lane.id),
                                {'length': LANECHANGECOST,
                                 'time'  : LANECHANGECOST}])                
                            edges.append(edge)
                            # Create edge from left to current lane
                            edge = ([(road.id, left_lane.id),
                                (road.id, lane.id),
                                {'length': LANECHANGECOST,
                                 'time'  : LANECHANGECOST}])                
                            edges.append(edge)


                # Take predecessor as next connection for positive lane ids
                elif (lane.id > 0 and lane.type == 'driving'):
                    # When predecessor is road get predecessor connection
                    if (road.link.predecessor.elementType == 'road'): 

                        incomingEntryLaneId, outgoingEntryLaneId = (
                            self.calculate_lane_link(road, lane))

                        # Create edge from current to predecessor
                        edge = ([(road.id, incomingEntryLaneId),
                            (road.link.predecessor.elementId, outgoingEntryLaneId),
                            {'length': road_weight['length'], 
                             'time'  : road_weight['time']}])

                        edges.append(edge)
                        # Add new edge label
                        edge_labels[((edge[0]), (edge[1]))] = edge[2]['length']
                        # Add initial position of the road to pos dict
                        location = self.map_object.get_inverted_initial_position(road, lane, 'left')
                        pos[road.id, lane.id] = (location.x, location.y)

                    # When predecessor is junction get all possible connections
                    elif (road.link.predecessor.elementType == 'junction'):

                        junction = self.map_object.get_junction(
                            self.map_object.junctions, road.link.predecessor.elementId)
                        # Create edge from current to all possible connections
                        for connection in junction.connection:
                            if (connection.incomingRoad == road.id):
                                for laneLink in connection.laneLink:
                                    if laneLink.fromLane == lane.id:
                                        connectingLane = laneLink.toLane
                                        edge = ([(road.id, lane.id),
                                                (connection.connectingRoad, connectingLane),
                                                {'length': road_weight['length'], 
                                                 'time'  : road_weight['time']}])
                                        edges.append(edge)
                                        edge_labels[((edge[0]), (edge[1]))] = edge[2]['length']
                                        location = self.map_object.get_inverted_initial_position(road, lane, 'left')
                                        pos[road.id, lane.id] = (location.x, location.y)                   

                    # Check lane changes
                    ### (Must check all the roadmark segments, for possible bug in xodr definition) ###
                    for roadMark in lane.roadMark:
                        laneChange_flag = "none"
                        if roadMark.laneChange == "both":
                            laneChange_flag = "both"
                            break
                    ### *** ###
                    if (laneChange_flag == "both"):
                        # Get right lane
                        right_lane = self.map_object.get_lane(road, lane.id + 1, 0)
                        if (right_lane is not None and right_lane.type == "driving"):
                            # Create edge from current to right lane
                            edge = ([(road.id, lane.id),
                                (road.id, right_lane.id),
                                {'length': LANECHANGECOST,
                                 'time'  : LANECHANGECOST}])                
                            edges.append(edge)
                            # Create edge from right to current lane
                            edge = ([(road.id, right_lane.id),
                                (road.id, lane.id),
                                {'length': LANECHANGECOST,
                                 'time'  : LANECHANGECOST}])                
                            edges.append(edge)  
                            
        return edges, edge_labels, pos

    def create_lane_graph(self, edges, edge_labels, pos):
        """
        Create lane graph using networkx lib

        Args: 
            edges: (list) Edges from node to node to create the graph
            edge_labels: (dict) Labels to represent the weight of the edges
            pos: (dict) Positions where to represent the edges in the graph

        Returns:
            G: (networkx.DiGraph) 
        """
        G = nx.DiGraph()
        G.add_edges_from(edges)
        # pos2 = nx.spring_layout(G)
        # plt.figure()
        
        # @TODO: This draw functions are only for visualization of the graph purpose. 
        # In Town03 are giving some error due to the nodes generation, thats why 
        # have been commented

        # nx.draw(G, pos, labels={node:node for node in G.nodes()})
        # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
        # plt.show()

        return G

    def calculate_global_route(self, init_xyz, goal_xyz, weight="length"):
        """
        Calculates optimal route using A* algorithm, as a list of
        tuples (road, lane).

        Args:
            init_xyz: (tuple) xyz coordinates where the route starts
            goal_xyz: (tuple) xyz coordinates where the route ends
            weight  : (str)   Weight to consider the cost in A* algorithm. 
                              Options are:
                                    - length : distance in meters
                                    . time   : length / vmax of the road

        Returns: 
            route: (list) Route as a list of tuples (road, lane)
        """     

        # Get initial and goal (road, lane) from the xyz coordinates
        init_roadlane = self.map_object.xyz_to_roadlane(init_xyz[0], init_xyz[1], init_xyz[2])
        goal_roadlane = self.map_object.xyz_to_roadlane(goal_xyz[0], goal_xyz[1], goal_xyz[2])

        # Calculate route as a list of (road, lane) using A* algorithm
        route_aux = nx.astar_path(self.G, init_roadlane, goal_roadlane, weight=weight) 

        route = []
        # Add a 3rd parameter with the type of action to do: 
        # LANEFOLLOW, CHANGERIGHT, CHANGELEFT   
        for i in range(len(route_aux) - 1):
            # If current road is same that current road but lane is different, 
            # type of action is lanechange
            action = self.map_object.get_lane_action(route_aux[i], route_aux[i+1])
            route.append((route_aux[i][0], route_aux[i][1], action))
        
        # Add last step of the route
        action = "lanefollow"
        route.append((route_aux[-1][0], route_aux[-1][1], action))

        return route

    def calculate_roadlane_route(self, init_roadlane, goal_roadlane):
        """
        Calculates optimal route using A* algorithm, as a list of
        tuples (road, lane).
        (Usually init and goal position are given as xyz, this function
        is more about debugging purpose)

        Args:
            init_roadlane: (tuple) road/lane where the route starts
            goal_roadlane: (tuple) road/lane where the route ends

        Returns: 
            route: (list) Route as a list of tuples (road, lane)
        """   
        # Calculate route as a list of (road, lane) using A* algorithm
        route = nx.astar_path(self.G, init_roadlane, goal_roadlane) 
        print(nx.astar_path_length(self.G, init_roadlane, goal_roadlane))
        return route

    def euclidean_distance(self, wp1, wp2):
        """
        Computes euclidean distance from one waypoint to another
        Args:
            wp1: (t4ac.Waypoint)
            wp2: (t4ac.Waypoint)
        Returns: 
            distance: (float) Euclidean distance
        """

        x1 = wp1.transform.location.x
        y1 = wp1.transform.location.y
        x2 = wp2.transform.location.x
        y2 = wp2.transform.location.y

        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) **2)

        return distance

