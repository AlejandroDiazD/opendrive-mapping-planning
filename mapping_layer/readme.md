# Mapping-Layer

## Overview
The Mapping-Layer parses the information from an HD map in OpenDRIVE format to visualize the map and generate a map monitor.

This layer inits 3 ROS nodes:
* __Map Visualizator Node__: Visualizes every driving lane of the map in RVIZ
* __Map Monitor Node__: Monitors the relevant lanes and regulatory elements around the ego-vehicle to support other modules making use of the map data.
* __Monitor Visualizator Node__: Receives the monitor elements and publishes them as ROS markers for visualization in RVIZ

## Authors
Alejandro Diaz-Diaz (alejandro.diazd@uah.es)

## Requierements
* ROS noetic
* t4ac_msgs

## Inputs and Outputs
![esquema_mapping](https://user-images.githubusercontent.com/61188820/165801771-7ff9ec1a-2783-43e7-bed4-1bcd34d2bfef.png)

### Map Visualizator Node
| Topics | Type | Message Format | Description |
| ------------- | ------------- | ----------------- | --------------|
| _/t4ac/mapping/map/lanes_marker_ | Publisher | visualization_msgs/Marker | Markers of the map lanes for RVIZ visualization

### Map Monitor Node
| Topics | Type | Message Format | Description |
| ------------- | ------------- | ----------------- | --------------|
| _/t4ac/localization/pose_  | Subscriber | nav_msgs/Odometry | Ego-vehicle odometry
| _/t4ac/planning/route_  | Subscriber | t4ac_msgs/Path | Route calculated
| _/t4ac/mapping/monitor/lanes_  | Publisher | t4ac_msgs/MonitorizedLanes | Monitored lanes around the ego-vehicle
| _/t4ac/mapping/intersections_  | Publisher | t4ac_msgs/MonitorizedLanes | Monitored lanes of next intersection affecting the route
| _/t4ac/mapping/monitor/regElems_  | Publisher | t4ac_msgs/MonitorizedRegElem | Monitored regulatory elements around the ego-vehicle affecting the route

### Monitor Visualizator Node
| Topics | Type | Message Format | Description |
| ------------- | ------------- | ----------------- | --------------|
| _/t4ac/mapping/monitor/lanes_  | Subscriber | t4ac_msgs/MonitorizedLanes | Monitored standard lanes around the ego-vehicle
| _/t4ac/mapping/monitor/intersections_  | Subscriber | t4ac_msgs/MonitorizedLanes | Monitored lanes of next intersection affecting the route
| _/t4ac/mapping/monitor/regElems_  | Subscriber | t4ac_msgs/MonitorizedRegElem | Monitored regulatory elements around the ego-vehicle affecting the route
| _/t4ac/mapping/monitor/lanes_marker_  | Publisher | visualization_msgs/Marker | Markers of the standard lanes for RVIZ visualization
| _/t4ac/mapping/monitor/intersections_marker_  | Publisher | visualization_msgs/Marker | Markers of the intersection lanes for RVIZ visualization
| _/t4ac/mapping/monitor/regElems_marker_  | Publisher | visualization_msgs/Marker | Markers of the regulatory elements for RVIZ visualization



## Launch Options
Arguments and parameters are the same for every launch option.
### mapping.launch (main)
Main launch file of the Mapping layer. It inits tha map visualizator, map monitor and monitor visualizator.

| Argument | Data type | Description |
| ------------- | ------------- | --------------|
| _map_name_  | string | Name of the map

| Parameter | Data type | Description |
| ------------- | ------------- | --------------|
| _map_name_  | string | Name of the map
| _map_path_  | string | Path of the map files

### map_visualizator.launch
Launch to visualize the map in RVIZ. This method has some bugs when calculating the Z coordinate.

### map_monitor.launch
Launch to start and visualize the map monitor.


## Description 

The Mapping Layer has two main parts: the map parser and the map monitor. The map parser is structured into a class MapParser that is managed as a module and used through the MapObject class. The map parser files are in the /map_parser directory. The map monitor is structured in the /t4ac_map_monitor_ros as a ROS package.

There is a main launch file (__mapping.launch__) to start the layer and there are also other launch files to start the different parts independently (map_visualizator and map_monitor).

The maps used, mainly by the map parser, are in /maps directory.

### Map Parser
The map parser code is in /map_parser directory. The main files are:

    /map_parser
        |_ map_classes.py
        |_ builder_classes.py
        |_ map_parser.py
        |_ map_object.py
        |_ signal_parser.py

The map_parser.py defines a MapParser class that receives an OpenDRIVE HD map as input and parses its information into the classes defined in map_classes.py following the OpenDRIVE structure (road, lane, junction, ...). 

In builder_classes.py is defined the T4ac_Waypoint class and its associated classes (transform, location and rotation). A waypoint is a structured object that represents a 3D point with location (x, y, z), rotation (pitch, yaw, roll) and some other topological information obtained from the HD map: road and lane ID, lane width, lane marking and speed limit of the road. The T4ac_Waypoint class also provides some util methods.

The map_object.py defines a MapObject class, that inherits from the MapParser so it parses the map data but also provides util methods to manage that data as for example to generate discrete waypoints in a specific road-lane. 

The signal_parser.py aims to be a specific parser for the signals, but is still in developement. 

### Map Monitor
The map monitor code is structured as a ROS package in /t4ac_map_monitor_ros and makes use of the map parser code importing the MapObject class.

There are the different launch files in /launch directory and the code in /src directory. 
The main files are:

    /t4ac_map_monitor_ros
    |_ /src
       |_ map_visualizator.py
       |_ map_monitor.py
       |_ monitor_visualizator.py

The map visualizator generates waypoints at both sides of every driving lane to visualize the road map using ROS markers in RVIZ.

The map monitor uses the parsed information from the HD map and the route calculated by the global planner to monitor the relevant elements close to the ego-vehicle that are affecting the route:
    
* Standard lanes: current front, right front, left front, current back, right back and left back lanes.
* Intersection lanes: split, merge and cross lanes.
* Regulatory Elements: This is a work in progress, but the elements are stops, giveaways, traffic lights, speed limits and crosswalks.

The distance monitored in front is calculated based on the current velocity of the ego-vehicle. The distance for the monitors behind is calculated based on the front distance. 

The monitors are calculated in a callback that is updated every time the odometry of the ego-vehicle is published. 

### Maps
Maps are in the /maps directory, separated by format. The maps used in this project are in OpenDRIVE format (xodr files). Inside the /xodr directory, some of the maps are classfied by their CARLA version. 

## Results
https://user-images.githubusercontent.com/61188820/168630376-acda7c02-b184-4e31-8011-14d543cb4927.mp4

Refer to bibliography.

## Future Works
### General
* Complete the regulatory elements part of the map monitor.
* Improve test/debugging process.

### TODOs
* @TODO: Solve bug when calculating Z in map visualizator
* @TODO: Adapt T4ac_Waypoint.get_closer_waypoint() to kdtrees
* @TODO: Solve bug when obtaining intersection lanes and monitors a lane in the opposite direction because it considers the complete route

## Bibliography
* Paper will be published soon
