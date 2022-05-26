## map_visualizator.py
Gets map_waypoints (waypoints centered in every driving lane of the map at a given distance) using the MapObject and generates points at both sides to visualize every driving lane of the map in RVIZ using RVIz markers. 

The markers are only published once when the node is initialized, but whenever the ROS parameter /t4ac/mapping/map_name is changed the map_visualizator updates the map too.

@TODO: There is an error when calculating the Z of some waypoints with elevation.

## map_monitor.py
The map monitor is structured in a MapMonitor class that is initialized and subscribed to route and odometry topics to operate when their callbacks are activated. The map monitor goal is to monitor the relevant lanes and regulatory elements around the ego-vehicle that are affecting the route. 

The monitored elements are:
* __Standard lanes__: Current, back and the corresponding left and right lanes. Current lane is monitored from current position to a dynamic distance depending on the velocity of the ego-vehicle. Back lane is monitored from current position to back a proportional distance of the dynamic current lane obtained distance. Left and right lanes are monitored the same distance that current and back only if the lane marking from the HD map data allows the lane change.
* __Intersection lanes__: Other lanes that intersect the current monitored lane are checked. Intersection lanes can have different roles: split (1 lane splits into 2 or more), merge (2 or more lanes merge into 1) and cross (a lane crosses a part of the current lane). To calculate the intersection lanes, each lane of every junction (junctions are areas where more than 2 roads meet) in the current lane is evaluated. The polygon of each lane is calculated and evaluated if is inside the polygon of the current lane. Roundabouts are considered as a set of multiple junctions.
* __Regulatory elements__: The monitored elements are
stops, giveaways, traffic lights, speed limits and crosswalks. The regulatory elements are only monitored for the next intersection affecting the route.

The MapMonitor is structured in two callback functions that are activated whenever a message is published in any of the corresponding topics:
* Route callback: Recalculates the T4ac_Path route as a list of T4ac_Waypoint. Then, divides the route in segments every each 2 waypoints to locate the ego-vehicle in the corresponding segment and activates the map monitor.
* Localization callback: Calculates front and back distances to monitor depending on the velocity of the ego-vehicle and starts monitoring the elements. 

## monitor_visualizator.py
Activates a node that gets subscribed to the monitor topics and calculates the equivalent monitors as ROS markers to visualize them in RVIZ.