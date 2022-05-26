## map_classes.py
Defines some classes to reproduce the OpenDRIVE format and structure the parsed data from the HD map.

## builder_classes.py
In builder_classes.py is defined the T4ac_Waypoint class and its associated classes (transform, location and rotation). A waypoint is a structured object that represents a 3D point with location (x, y, z), rotation (pitch, yaw, roll) and some other topological information obtained from the HD map: road and lane ID, lane width, lane marking and speed limit of the road. The T4ac_Waypoint class also provides some util methods.

## map_parser.py
Defines the MapParser class the receives an HD map in OpenDRIVE format and structures the information in the classes defines in map_classes.py. 

The MapParser receives 3 input parameters:

1. __map_data__:

    The map_data parameters can be given in two different ways, depending on the value selected in map_flag parameter:

    * map_flag == 0: The value of map_data must be a string with the name of the map selected (i.e. "Town01"). The map file will be searched in the map_path indicated, so it must exist in the directory.
    * map_flag == 1: The value of the map_data must be a string containing all the data of the map file. This option was enabled for the format of the CARLA CHALLENGE, where the map_data is given in this way.

2. __map_path__:

    The map path is the absolute path where the map files are located. In case of map_flag == 1 the map path is ignored. 

3. __map_flag__:

    Map flag can be 0 or 1 depending on the format of the value provided in map_data

## map_object.py
The map_object.py defines a MapObject class, that inherits from the MapParser so it parses the map data but also provides util methods to manage that data as for example to generate discrete waypoints in a specific road-lane.

The input parameters are the 3 same that in MapParser.


## signal_parser.py
The signal_parser.py aims to be a specific parser for the signals, but is still in developement.