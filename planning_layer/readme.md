# Planning-Layer

## Overview
This layer implements a global planner to calculate a global route of waypoints from the ego-vehicle position to a goal position, using the information of an HD map based on OpenDRIVE format. 
This layer inits one ROS node:
* __Global Planner Node__: This node is subscribed to a goal topic. Every time a new goal is published, the route is calculated and published in another route topic as a list of waypoints using a custom ROS message.

## Authors
Alejandro Diaz-Diaz (alejandro.diazd@uah.es)

## Requierements
* ROS noetic
* t4ac_mapping_layer
* t4ac_msgs
* networkx

## Inputs and Outputs
![esquema_planning](https://user-images.githubusercontent.com/61188820/165753791-8b0dec8e-11dd-4ad2-9094-30e86c1567aa.png)

### Global Planner Node
| Topics | Type | Message Format | Description |
| ------------- | ------------- | ----------------- | --------------|
| _/t4ac/localization/pose_  | Subscriber | nav_msgs/Odometry | Ego-vehicle odometry
| _/t4ac/planning/goal_  | Subscriber | geometry_msgs/PoseStamped | Destination of the route
| _/t4ac/decision_making/lane_change_order_left_ | Subscriber | std_msgs/Bool | Left lane change order
| _/t4ac/decision_making/lane_change_order_right_ | Subscriber | std_msgs/Bool | Right lane change order
| _/t4ac/planning/route_  | Publisher | t4ac_msgs/Path | Route calculated
| _/t4ac/planning/lane_change_executed_  | Publisher | std_msgs/Bool | True when lane change has been completed


## Launch Options
### planning.launch (main)
Main launch file of the Planning layer that starts the global planner.

| Argument | Data type | Description |
| ------------- | ------------- | --------------|
| _map_name_  | string | Name of the map
| _distance_between_waypoints_  | double | Distance between waypoints in the route
| _visualization_ | bool | Activates visualization markers for odometry, goal and route

| Parameter | Data type | Description |
| ------------- | ------------- | --------------|
| _map_name_  | string | Name of the map
| _map_path_  | string | Path of the map files
| _distance_between_waypoints_  | double | Distance between waypoints

## Description
The global planner is structured in the directory /t4ac_global_planner_ros as ROS package. 

There is main launch file (__planning.launch__) in the /launch directory that inits the global_planner.

The code of the global planner is in the /src directory. 

The global planner is structured in two files: lane_graph_planner.py (LGP) and lane_waypoint_planner.py (LWP). 

The LGP inherits from the MapObject (that inherits from the MapParser) of the Mapping Layer, and generates a graph of roads and lanes of the map using Networkx. The input to genrate the graph is an HD map in OpenDRIVE format (xodr file). Applying a Dijkstra algorithm to the graph, it can get the shortest path (as a list of tuples (road, lane)) from an init to a goal road-lane.

The LWP inherits from the LGP and receives the route calculated by the LGP to calculate waypoints centered at the lane separated by a given distance for each (road, lane) of the route.

There is a visualization option that can be activated in the launch file to visualize the odometry of the ego-vehicle, the goal position and the route calculated.


## Results
https://user-images.githubusercontent.com/61188820/168269086-3f0398cd-ef04-41cf-9c56-415bfa2d0bfd.mp4

Refer to bibliography.

## Future Works
### General 
* LaneWaypointPlanner (LWP) inherits from LaneGraphPlanner (LGP) that inherits from MapObject.This makes the LWP dependent from the Mapping layer. The ideal would be to make every layer independant from any other.
* Improve test/debugging process

### TODOs
* Fix minor bugs when calculating the route (some times calculates extra waypoints at the begining or at the end of the route)

## Bibliography
* Related paper will be published soon
