"""
Usefull classes for map_monitor. 
Some of them are replicated from ROS t4ac_msgs.
"""

class Node2D:
    """
    2D point object 

    Attributes:
        x: Coordinate x
        y: Coordinate y
    """
    def __init__ (self, x = 0, y = 0):
        self.x = float(x)
        self.y = float(y)
    
    def __str__ (self):
        return ("{}\n"
                "x = {}\n"
                "y = {}"
                .format(self.__class__, self.x, self.y))

class Node3D:
    """
    3D point object 

    Attributes:
        x: Coordinate x
        y: Coordinate y
        z: Coordinate z
    """
    def __init__ (self, x = 0, y = 0, z = 0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __str__ (self):
        return ("{}\n"
                "x = {}\n"
                "y = {}\n"
                "z = {}"
                .format(self.__class__, self.x, self.y, self.z))
    
class Lane:
    """
    Lane defined by right and left way, with an specific role

    Attributes:
        central_way: List of waypoints defining the center of the lane
        left_way: List of waypoints defining left edge of the lane
        right_way: List of waypoints defining right edge of the lane
    """
    def __init__ (self):
        self.central_way = []
        self.left_way = []
        self.right_way = []
        self.role = ""

    def __str__ (self):
        return ("{}\n"
                "central = {}\n"
                "left = {}\n"
                "right = {}\n"
                "role = {}\n"
                .format(self.__class__, self.central_way, self.left_way, 
                self.right_way, self.role))

class Landmark:
    """
    Landmark element defining where a regulatory element takes place

    Attributes:
        location: Location (x,y,z) of the landmark
        distance: Distance in meters from current location to landmark
        affecting_road: Id of road that is affected by this landmark
    """
    def __init__ (self, location = None, distance = None, 
                  affecting_road = None):
        self.location = location
        self.distance = distance
        self.affecting_road = affecting_road

    def __str__ (self):
        return ("{}\n"
                "location(x,y,z) = ({},{},{})\n)"
                "distance = {}\n"
                "affecting_road = {}\n"
                .format(self.__class__, self.location.x, self.location.y,
                self.location.z, self.distance, self.affecting_road))

class RegulatoryElement:
    """
    Regulatory element that is a physical element and also has some landmarks
    referenced

    Attributes:
        element_type: Type of regulatory element (e.g. StopSign, 
            TrafficLight, ...)
        id: Unique identifier 
        location: Location(x,y,z) of the physical regulatory element
        distance: Distance in meters from current location to element
        affecting_roads: Id of roads that are affected by this element
        landmarks: Landmarks associated to this regulatory element
    """
    def __init__ (self, element_type = None, id = None, location = None, 
                  distance = None, affecting_roads = None, landmarks = None):
        self.element_type = element_type
        self.id = id 
        self.location = location 
        self.distance = distance
        self.affecting_roads = affecting_roads
        self.landmarks = landmarks

    def __str__ (self):
        return ("{}\n"
                "element_type = {}\n"
                "id = {}\n"
                "location(x,y,z) = ({},{},{})\n"
                "distance = {}\n"
                "affecting_roads = {}\n"
                "landmarks = {}\n"
                .format(self.__class__, self.element_type, self.id, 
                self.location.x, self.location.y, self.location.z, 
                self.distance, self.affecting_roads, self.landmarks))
