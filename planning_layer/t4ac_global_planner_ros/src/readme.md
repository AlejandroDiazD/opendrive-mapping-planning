## lane_graph_planner.py
The class LaneGraphPlanner (LGP) inherits from the MapObject class (that inherits from the MapParser) from the Mapping Layer. 
Generates a roads-lanes graph of the map using Networkx. Then, the LGP provides methods to obtain a route of roads-lanes from an initial position to a goal position applying Dijkstra algorithm to the graph.

The LGP considers lane change when is possible and reveives 3 input parameters:
1. __map_data__:

    The map_data parameters can be given in two different ways, depending on the value selected in map_flag parameter:

    * map_flag == 0: The value of map_data must be a string with the name of the map selected (i.e. "Town01"). The map file will be searched in the map_path indicated, so it must exist in the directory.
    * map_flag == 1: The value of the map_data must be a string containing all the data of the map file. This option was enabled for the format of the CARLA CHALLENGE, where the map_data is given in this way.

2. __map_path__:

    The map path is the absolute path where the map files are located. In case of map_flag == 1 the map path is ignored. 

3. __map_flag__:

    Map flag can be 0 or 1 depending on the format of the value provided in map_data


## lane_waypoint_planner.py
The class LaneWaypointPlanner (LWP) inherits from the LaneGraphPlanner, and therefore inherits from the MapObject and the MapParser too. 
It uses LaneGraphPlanner methods to obtain a roads-lanes route from an initial xyz to a goal xyz locations. Then, it uses MapObject methods to generate waypoints centered in every lane of the route and separated by a given distance. 

The LWP is mainly structured in 2 ROS callbacks:
1. Odometry callback: This function is called every time the odometry of the ego-vehicle is pusblished to update the value of the xyz position of the ego-vehicle.
2. Goal callback: This function is called whenever a new goal is published. Then, it calculates the waypoint route and publishes it in the corresponding ROS topic.

There are other 2 additional callbacks for lane change order:

3. Change left callback: This function is called when the decision making module gives the order to try a left lane change. The LWP checks if the lane change is possible depending on the lane marking of the lane, and if is possible it recalculates a new route starting on the left lane and using the same goal of the previous route. This way a lane change is done.
4. Change right callback: Same for right lane change.


## route_visualizator.py
Optional node that can be activated in the main launch file for debugging purposes. This node gets subscribed to odometry, goal and route topics, and publish the equivalent messages as ROS markers so they can be visualized using RVIZ.