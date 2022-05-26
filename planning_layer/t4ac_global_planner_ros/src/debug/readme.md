Here are some debug files to check the functionality of different parts of the code. 

* debug_lgp.py: Inits the LaneGraphPlanner and calculates a route as a list of road-lane from an init xyz to a goal xyz.

* debug_lwp: Inits the LaneWaypointPlanner and calculates a route as a list of T4ac_Waypoint from an init xyz to a goal xyz. Then, plots the route in RVIZ using ROS markers.

* debug_odometry_simulator.py: This a really useful script to debug the functionality of the global planner and the map_monitor from the Mapping Layer. It simulates the odometry of the vehicle moving trough the route calculated by the planner.

* odometry_poses_debugging.txt: Structure for publishing an odometry msg using the terminal.