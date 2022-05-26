# HD Maps: Exploiting OpenDRIVE potential for Path Planning and Map Monitoring

This code contains the layers for mapping and planning developed in this work. 

Each of the layers contains its own documentation.

## Publications

If you use this software in your research, please cite our publication:

__"HD Maps: Exploiting OpenDRIVE potential for Path Planning and Map Monitoring"__ , ..., __[Accepted paper, to be published in Jun 2022]__

## Modules
* Mapping: Parses the information from an HD map in OpenDRIVE format to visualize the map and generate a map monitor.
* Planning: Global planner to calculate a route of waypoints from ego-vehicle location to a goal location.

## Requirements
* ROS Noetic
* t4ac_msgs
* networkx
* Both modules, mapping and planning, require the other