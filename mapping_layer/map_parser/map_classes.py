"""
Some classes describing structures to parse a file in xodr format
    / header
        // geoReference
    / roads []
        // road
            /// link (roadLink)
                //// predecessor (roadPredecessor)
                //// successor (roadSuccessor)
            /// type
                //// speed
            /// planView
                //// geometry
            /// lanes
                //// laneOffset
                //// laneSections
                    ///// lane
                        ////// link (laneLink)
                            /////// predecessor (lanePredecessor)
                            /////// successor (laneSuccessor)
                        ////// width
                        ////// roadMark
                        ////// userData
            /// objects
                //// object
                    ///// cornerLocal
            /// signals
                //// signal
                    ///// validity
                    ///// userData
                //// signalReference
                    ///// validity
                    ///// userData
    / controllers []
        // controller
            /// control
    / junctions []
        // junction
            /// [] connection
                //// link (junctionLaneLink)
            /// [] controller
            /// userData
"""
##############
### HEADER ###

class T4ac_Header:
    def __init__(self):
        self.revMajor = ""
        self.revMinor = ""
        self.name = ""
        self.version = ""
        self.date = ""
        self.north = ""
        self.south = ""
        self.east = ""
        self.west = ""
        self.vendor = ""
        self.geoReference = T4ac_GeoReference()

class T4ac_GeoReference:
    def __init__(self):
        self.lat_0 = ""
        self.lon_0 = ""
        self.k = ""
        self.x_0 = ""
        self.y_0 = ""
        self.datum = ""
        self.units = ""
        self.geoidgrids = ""
        self.vunits = ""

### HEADER ###
##############


### ROADS ###
#############

class T4ac_Road:
    def __init__(self):
        self.name = ""
        self.length = ""
        self.id = ""
        self.junction = ""
        self.link = T4ac_RoadLink()
        self.type = T4ac_RoadType()
        self.planView = [] #T4ac_Geometry()
        self.elevationProfile = [] #T4ac_Elevation
        #lateralProfile
        self.lanes = T4ac_Lanes()
        self.objects = [] #T4ac_Object()
        self.signals = T4ac_Signals() 

class T4ac_RoadLink:
    def __init__(self):
        self.predecessor = T4ac_RoadPredecessor()
        self.successor = T4ac_RoadSuccessor()

class T4ac_RoadPredecessor:
    def __init__(self):
        self.elementType = ""
        self.elementId = ""
        self.contactPoint = ""

class T4ac_RoadSuccessor:
    def __init__(self):
        self.elementType = ""
        self.elementId = ""
        self.contactPoint = ""

class T4ac_RoadType:
    def __init__(self):
        self.s = ""
        self.type = ""
        self.country = ""
        self.speed = T4ac_Speed()

class T4ac_Speed:
    def __init__(self):
        self.max = 0
        self.unit = ""

class T4ac_Geometry:
    def __init__(self):
        self.s = ""
        self.x = ""
        self.y = ""
        self.hdg = ""
        self.length = ""
        self.type = ""
        self.curvature = ""

class T4ac_Elevation:
    def __init__(self):
        self.s = ""
        self.a = ""
        self.b = ""
        self.c = ""
        self.d = ""

### ROADS ###
#############


### LANES ###
#############

class T4ac_Lanes:
    def __init__(self):
        self.laneOffset = [] #T4ac_LaneOffset
        self.laneSections = [] #T4ac_LaneSection()

class T4ac_LaneOffset:
    def __init__(self):
        self.s = ""
        self.a = ""
        self.b = ""
        self.c = ""
        self.d = ""

class T4ac_LaneSection:
    def __init__(self):
        self.s = ""
        self.left = [] #T4ac_Lane()
        self.center = [] #T4ac_Lane()
        self.right = [] #T4ac_Lane()

class T4ac_Lane:
    def __init__(self):
        self.id = ""
        self.type = ""
        self.level = ""
        self.link = T4ac_LaneLink()
        self.width = [] #T4ac_LaneWidth()
        self.roadMark = [] #T4ac_RoadMark
        self.userData = [] #T4ac_VectorLane

class T4ac_LaneLink:
    def __init__(self):
        self.predecessor = T4ac_LanePredecessor()
        self.successor = T4ac_LaneSuccessor()

class T4ac_LanePredecessor:
    def __init__(self):
        self.id = ""
        
class T4ac_LaneSuccessor:
    def __init__(self):
        self.id = ""
        
class T4ac_LaneWidth:
    def __init__(self):
        self.sOffset = ""
        self.a = ""
        self.b = ""
        self.c = ""
        self.d = ""

class T4ac_RoadMark:
    def __init__(self):
        self.sOffset = ""
        self.type = ""
        self.material = ""
        self.color = ""
        self.width = ""
        self.laneChange = ""

class T4ac_VectorLane:
    def __init__(self):
        self.sOffset = ""
        self.laneId = ""
        self.traverDir = ""

### LANES ###
#############



### OBJECTS ###
###############

class T4ac_Object:
    def __init__(self):
        self.id = ""
        self.name = ""
        self.s = ""
        self.t = ""
        self.zOffset = ""
        self.hdg = ""
        self.roll = ""
        self.pitch = ""
        self.orientation = ""
        self.type = ""
        self.height = ""
        self.width = ""
        self.length = ""
        self.outline = [] #T4ac_CornerLocal()

class T4ac_CornerLocal:
    def __init__(self):
        self.u = ""
        self.v = ""
        self.z = ""

### OBJECTS ###
###############


### SIGNALS ###
###############

class T4ac_Signals:
    def __init__(self):
        self.signal = [] #T4ac_Signal()
        self.signalReference = [] #T4ac_SignalReference()

class T4ac_Signal:
    def __init__(self):
        self.name = ""
        self.id = ""
        self.s = ""
        self.t = ""
        self.zOffset = ""
        self.hOffset = ""
        self.roll = ""
        self.pitch = ""
        self.orientation = ""
        self.dynamic = ""
        self.country = ""
        self.type = ""
        self.subtype = ""
        self.value = ""
        self.text = ""
        self.height = ""
        self.width = ""
        self.validity = T4ac_Validity()
        self.userData = T4ac_VectorSignal()

class T4ac_SignalReference:
    def __init__(self):
        self.id = ""
        self.s = ""
        self.t = ""
        self.orientation = ""
        self.validity = T4ac_Validity()
        self.userData = T4ac_VectorSignal()

class T4ac_VectorSignal:
    def __init__(self):
        self.signalId = ""
        self.gateId = ""
        self.turnRelation = ""

class T4ac_Validity:
    def __init__(self):
        self.fromLane = ""
        self.toLane = ""

### SIGNALS ###
###############


### CONTROLLERS ###
###################

class T4ac_Controller:
    def __init__(self):
        self.name = ""
        self.id = ""
        self.sequence = ""
        self.control = [] #T4ac_ControlSignal()

class T4ac_ControlSignal:
    def __init__(self):
        self.signalId = ""
        self.type = ""

### CONTROLLERS ###
###################


### JUNCTIONS ###
#################

class T4ac_Junction:
    def __init__(self):
        self.id = ""
        self.name = ""
        self.connection = [] #T4ac_Connection()
        self.controller = [] #T4acControllerJunction()
        self.userData = T4ac_VectorJunction()

class T4ac_Connection:
    def __init__(self):
        self.id = ""
        self.incomingRoad = ""
        self.connectingRoad = ""
        self.contactPoint = ""
        self.laneLink = [] #T4ac_LaneLinkJunction()

class T4ac_LaneLinkJunction:
    def __init__(self):
        self.fromLane = ""
        self.toLane = ""

class T4ac_ControllerJunction:
    def __init__(self):
        self.id = ""
        self.type = ""
        self.sequence = ""

class T4ac_VectorJunction:
    def __init__(self):
        self.junctionId = ""

### JUNCTIONS ###
#################

### CARLA EQUIVALENT CLASSES ###
################################

class T4ac_Transform:
    def __init__(self):
        self.location = T4ac_Location()
        self.rotation = T4ac_Rotation()

class T4ac_Location:
    def __init__(self):
        self.x = ""
        self.y = ""
        self.z = ""

class T4ac_Rotation:
    def __init__(self):
        self.pitch = ""
        self.yaw = ""
        self.roll = ""

################################
### CARLA EQUIVALENT CLASSES ###