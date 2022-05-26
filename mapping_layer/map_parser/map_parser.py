"""
Parse elements from an xodr road map file to a python MapParser object
"""

from map_classes import T4ac_Header
from map_classes import T4ac_Road
from map_classes import T4ac_Controller
from map_classes import T4ac_Junction
from map_classes import T4ac_Geometry
from map_classes import T4ac_Elevation
from map_classes import T4ac_Lane
from map_classes import T4ac_LaneSection
from map_classes import T4ac_Object
from map_classes import T4ac_Signal
from map_classes import T4ac_SignalReference
from map_classes import T4ac_LaneOffset
from map_classes import T4ac_LaneWidth
from map_classes import T4ac_RoadMark
from map_classes import T4ac_VectorLane
from map_classes import T4ac_CornerLocal
from map_classes import T4ac_Junction
from map_classes import T4ac_Connection
from map_classes import T4ac_LaneLinkJunction
from map_classes import T4ac_ControllerJunction
from map_classes import T4ac_VectorJunction

class MapParser:

    def __init__(self, map_data, map_path, map_flag = 0):
        """
        Args
            map_data: (str) It can be given in two ways:
                            0) Name of the map without extension, i.e. 'Town01'
                            1) All map data saved as a string
            map_path: (str) Path of the map files
            map_flag: (bool) 0) map_data in case 0
                             1) map_data in case 1
        """
        if map_flag == False:
            self.xodr_map = self._get_map(map_data, map_path)
        elif map_flag == True:
            self.xodr_map = map_data.split('\n')
        self.header = self._get_header(self.xodr_map)
        self.roads = self._get_roads(self.xodr_map)
        self.road_ids = self._get_road_ids(self.roads)
        #self.controllers =
        self.junctions = self._get_junctions(self.xodr_map)
        self.junction_ids = self._get_junction_ids(self.junctions)

    def _get_map(self,map_name, map_path):
        """
        Get xodr map that can be parsed as a txt file.
        Each line is a different string in a list, so it can be easier 
        to refer to an specific line.

        Args:
            map_name: (string)
            map_path: (string)

        Returns:
            xodr_map: (list) Map parsed as a list of string for each line
        """
        xodr_path = map_path + map_name + ".xodr"
        with open(xodr_path, "r") as od_file:    
            xodr_map = od_file.readlines()
        return xodr_map 

    def _get_param(self, param, line_str):
        """
        Auxiliar function to parse a parameter from a line in a xodr file.
        Two parameters are given, the 'param' to parse, and the line number 
        to parse 'line_str'
        """
        value = ""
        pos_ini = line_str.find(param)+len(param)+1    # Position of the first value param into the sring (+1 value is because of " in xodr format)
        if(pos_ini != -1):
            for i in range(pos_ini, len(line_str)):
                if line_str[i] == '"':
                    break
                else:
                    value += line_str[i]
        else:
            pass
        return value 

    def _get_param_geo(self, param, line_str):
        """
        Particular mod to get params for georeference sintax
        """
        value = ""
        pos_ini = line_str.find(param)+len(param)    # Position of the first value param into the sring (+1 value is because of " in xodr format)
        if(pos_ini != -1):
            for i in range(pos_ini, len(line_str)):
                if line_str[i] == ' ':
                    break
                else:
                    value += line_str[i]
        else:
            pass
        return value 

    def _get_header(self, xodr_map):
        in_header = 0
        for i in range(0, len(xodr_map)):
            if (xodr_map[i].find("<header ") != -1 and (in_header == 0)):
                in_header = 1
                header = T4ac_Header()
                header.revMajor = self._get_param(" revMajor=", xodr_map[i])
                header.revMinor = self._get_param(" revMinor=", xodr_map[i])
                header.name = self._get_param(" name=", xodr_map[i])
                header.version = self._get_param(" version=", xodr_map[i])
                header.date = self._get_param(" date=", xodr_map[i])
                header.north = self._get_param(" north=", xodr_map[i])
                header.south = self._get_param(" south=", xodr_map[i])
                header.east = self._get_param(" east=", xodr_map[i])
                header.west = self._get_param(" west=", xodr_map[i])
                header.vendor = self._get_param(" vendor=", xodr_map[i])
            elif (xodr_map[i].find("<geoReference>") != -1 and (in_header == 1)):
                header.geoReference.lat_0 = self._get_param_geo(" +lat_0=", xodr_map[i])
                header.geoReference.lon_0 = self._get_param_geo(" +lon_0=", xodr_map[i])
                header.geoReference.k = self._get_param_geo(" +k=", xodr_map[i])
                header.geoReference.x_0 = self._get_param_geo(" +x_0=", xodr_map[i])
                header.geoReference.y_0 = self._get_param_geo(" +y_0=", xodr_map[i])
                header.geoReference.datum = self._get_param_geo(" +datum=", xodr_map[i])
                header.geoReference.units = self._get_param_geo(" +units=", xodr_map[i])
                header.geoReference.geoidgrids = self._get_param_geo(" +geoidgrids=", xodr_map[i])
                header.geoReference.vunits = self._get_param_geo(" +vunits=", xodr_map[i])
                break
            else:
                pass
        return header

    def _get_roads(self, xodr_map):
        roads = []
        in_road = 0
        in_roadLink = 0
        in_type = 0
        in_planView = 0
        in_elevationProfile = 0
        in_lanes = 0
        in_laneSection = 0
        in_left = 0
        in_center = 0
        in_right = 0
        in_lane = 0
        in_laneLink = 0
        in_objects = 0
        in_outline = 0
        in_signals = 0
        in_signal = 0
        in_signalReference = 0
        for i in range(0, len(xodr_map)):
            if (in_road == 0):
                if (xodr_map[i].find("<road ") != -1):
                    in_road = 1
                    road = T4ac_Road()
                    road.name = self._get_param(" name=", xodr_map[i])
                    road.length = float(self._get_param(" length=", xodr_map[i]))
                    road.id = int(self._get_param(" id=", xodr_map[i]))
                    road.junction = int(self._get_param(" junction=", xodr_map[i]))
            elif (in_road == 1):
                if (in_roadLink==0 and in_type==0 and in_planView==0 and in_elevationProfile == 0 and in_lanes==0 and in_objects==0 and in_signals==0):
                    if (xodr_map[i].find("<link>") != -1):
                        in_roadLink = 1
                    elif (xodr_map[i].find("<type ") != -1):
                        in_type = 1
                        road.type.s = self._get_param(" s=", xodr_map[i])
                        road.type.type = self._get_param(" type=", xodr_map[i])
                        road.type.country = self._get_param(" country=", xodr_map[i])
                    elif (xodr_map[i].find("<planView>") != -1):
                        in_planView = 1
                    elif (xodr_map[i].find("<elevationProfile>") != -1):
                        in_elevationProfile = 1
                    elif (xodr_map[i].find("<lanes>") != -1):
                        in_lanes = 1
                    elif (xodr_map[i].find("<objects>") != -1):
                        in_objects = 1
                    elif (xodr_map[i].find("<signals>") != -1):
                        in_signals = 1
                    elif (xodr_map[i].find("</road>") != -1):
                        in_road = 0
                        roads.append(road)


                elif (in_roadLink == 1):
                    if (xodr_map[i].find("<predecessor ") != -1):
                        road.link.predecessor.elementType = self._get_param(" elementType=", xodr_map[i])
                        road.link.predecessor.elementId = int(self._get_param(" elementId=", xodr_map[i]))
                        if (road.link.predecessor.elementType == 'road'):
                            road.link.predecessor.contactPoint = self._get_param(" contactPoint=", xodr_map[i])
                    elif (xodr_map[i].find("<successor ") != -1):
                        road.link.successor.elementType = self._get_param(" elementType=", xodr_map[i])
                        road.link.successor.elementId = int(self._get_param(" elementId=", xodr_map[i]))
                        if (road.link.successor.elementType == 'road'):
                            road.link.successor.contactPoint = self._get_param(" contactPoint=", xodr_map[i])
                    elif (xodr_map[i].find("</link>") != -1):
                        in_roadLink = 0


                elif (in_type == 1): 
                    if (xodr_map[i].find("<speed ") != -1):
                        road.type.speed.max = int(self._get_param(" max=", xodr_map[i]))
                        road.type.speed.unit = self._get_param(" unit=", xodr_map[i])
                    elif (xodr_map[i].find("</type>") != -1):
                        in_type = 0


                elif (in_planView == 1):
                    if (xodr_map[i].find("<geometry ") != -1):
                        geometry = T4ac_Geometry()
                        geometry.s = float(self._get_param(" s=", xodr_map[i]))
                        geometry.x = float(self._get_param(" x=", xodr_map[i]))
                        geometry.y = float(self._get_param(" y=", xodr_map[i]))
                        geometry.hdg = float(self._get_param(" hdg=", xodr_map[i]))
                        geometry.length = float(self._get_param(" length=", xodr_map[i]))
                        if (xodr_map[i+1].find("<line/>") != -1):
                            geometry.type = "line"
                        elif (xodr_map[i+1].find("<arc ") != -1):
                            geometry.type = "arc"
                            geometry.curvature = float(self._get_param(" curvature=", xodr_map[i+1]))
                        road.planView.append(geometry)
                    elif (xodr_map[i].find("</planView>") != -1):
                        in_planView = 0

                elif (in_elevationProfile == 1):
                    if (xodr_map[i].find("<elevation ") != -1):
                        elevation = T4ac_Elevation()
                        elevation.s = float(self._get_param(" s=", xodr_map[i]))
                        elevation.a = float(self._get_param(" a=", xodr_map[i]))
                        elevation.b = float(self._get_param(" b=", xodr_map[i]))
                        elevation.c = float(self._get_param(" c=", xodr_map[i]))
                        elevation.d = float(self._get_param(" d=", xodr_map[i]))
                        road.elevationProfile.append(elevation)
                    elif (xodr_map[i].find("</elevationProfile>") != -1):
                        in_elevationProfile = 0


                elif (in_lanes == 1):
                    if (in_laneSection == 0):
                        if (xodr_map[i].find("<laneOffset ") != -1):
                            laneOffset = T4ac_LaneOffset()
                            laneOffset.s = float(self._get_param(" s=", xodr_map[i]))
                            laneOffset.a = float(self._get_param(" a=", xodr_map[i]))
                            laneOffset.b = float(self._get_param(" b=", xodr_map[i]))
                            laneOffset.c = float(self._get_param(" c=", xodr_map[i]))
                            laneOffset.d = float(self._get_param(" d=", xodr_map[i]))
                            road.lanes.laneOffset.append(laneOffset)
                        elif (xodr_map[i].find("<laneSection ") != -1):
                            laneSection = T4ac_LaneSection()
                            in_laneSection = 1
                            laneSection.s = self._get_param(" s=", xodr_map[i])
                        elif (xodr_map[i].find("</lanes>") != -1):
                            in_lanes = 0
                    elif (in_laneSection == 1): 
                        if (in_left==0 and in_center==0 and in_right==0):                          
                            if (xodr_map[i].find("<left>") != -1):
                                in_left = 1
                            elif (xodr_map[i].find("<center>") != -1):
                                in_center = 1
                            elif (xodr_map[i].find("<right>") != -1):
                                in_right = 1
                            elif (xodr_map[i].find("</laneSection>") != -1):
                                road.lanes.laneSections.append(laneSection)
                                in_laneSection = 0

                        elif (in_left == 1):
                            if (in_lane == 0):
                                if (xodr_map[i].find("<lane ") != -1):
                                    in_lane = 1
                                    lane = T4ac_Lane()
                                    lane.id = int(self._get_param(" id=", xodr_map[i]))
                                    lane.type = self._get_param(" type=", xodr_map[i])
                                    lane.level = self._get_param(" level=", xodr_map[i])
                                elif (xodr_map[i].find("</left>") != -1):
                                        in_left = 0
                            elif (in_lane == 1):
                                if (in_laneLink == 1):
                                    if (xodr_map[i].find("<predecessor ") != -1):
                                        lane.link.predecessor.id = int(self._get_param(" id=", xodr_map[i]))
                                    elif (xodr_map[i].find("<successor ") != -1):
                                        lane.link.successor.id = int(self._get_param(" id=", xodr_map[i]))
                                    elif (xodr_map[i].find("</link>") != -1):
                                        in_laneLink = 0
                                elif (in_laneLink == 0):
                                    if (xodr_map[i].find("<link>") != -1):
                                        in_laneLink = 1
                                    elif (xodr_map[i].find("<width ") != -1):
                                        width = T4ac_LaneWidth()
                                        width.sOffset = float(self._get_param(" sOffset=", xodr_map[i]))
                                        width.a = float(self._get_param(" a=", xodr_map[i]))
                                        width.b = float(self._get_param(" b=", xodr_map[i]))
                                        width.c = float(self._get_param(" c=", xodr_map[i]))
                                        width.d = float(self._get_param(" d=", xodr_map[i]))
                                        lane.width.append(width)
                                    elif (xodr_map[i].find("<roadMark ") != -1):
                                        roadMark = T4ac_RoadMark()
                                        roadMark.sOffset = self._get_param(" sOffset=", xodr_map[i])
                                        roadMark.type = self._get_param(" type=", xodr_map[i])
                                        roadMark.material = self._get_param(" material=", xodr_map[i])
                                        roadMark.color = self._get_param(" color=", xodr_map[i])
                                        roadMark.width = self._get_param(" width=", xodr_map[i])
                                        roadMark.laneChange = self._get_param(" laneChange=", xodr_map[i])
                                        lane.roadMark.append(roadMark)
                                    elif (xodr_map[i].find("<userData>") != -1):
                                        userData = T4ac_VectorLane()
                                        userData.sOffset = self._get_param(" sOffset=", xodr_map[i])
                                        userData.laneId = self._get_param(" laneId=", xodr_map[i])
                                        userData.travelDir = self._get_param(" travelDir=", xodr_map[i])
                                        lane.userData.append(userData)
                                    elif (xodr_map[i].find("</lane>") != -1):
                                        in_lane = 0
                                        laneSection.left.append(lane)                                    
                                
                        elif (in_center == 1):
                            if (in_lane == 0):
                                if (xodr_map[i].find("<lane ") != -1):
                                    in_lane = 1
                                    lane = T4ac_Lane()
                                    lane.id = int(self._get_param(" id=", xodr_map[i]))
                                    lane.type = self._get_param(" type=", xodr_map[i])
                                    lane.level = self._get_param(" level=", xodr_map[i])
                                elif (xodr_map[i].find("</center>") != -1):
                                        in_center = 0
                            elif (in_lane == 1):
                                if (in_laneLink == 1):
                                    if (xodr_map[i].find("<predecessor ") != -1):
                                        lane.link.predecessor.id = int(self._get_param(" id=", xodr_map[i]))
                                    elif (xodr_map[i].find("<successor ") != -1):
                                        lane.link.successor.id = int(self._get_param(" id=", xodr_map[i]))
                                    elif (xodr_map[i].find("</link>") != -1):
                                        in_laneLink = 0
                                elif (in_laneLink == 0):
                                    if (xodr_map[i].find("<link>") != -1):
                                        in_laneLink = 1
                                    elif (xodr_map[i].find("<width ") != -1):
                                        width = T4ac_LaneWidth()
                                        width.sOffset = float(self._get_param(" sOffset=", xodr_map[i]))
                                        width.a = float(self._get_param(" a=", xodr_map[i]))
                                        width.b = float(self._get_param(" b=", xodr_map[i]))
                                        width.c = float(self._get_param(" c=", xodr_map[i]))
                                        width.d = float(self._get_param(" d=", xodr_map[i]))
                                        lane.width.append(width)
                                    elif (xodr_map[i].find("<roadMark ") != -1):
                                        roadMark = T4ac_RoadMark()
                                        roadMark.sOffset = self._get_param(" sOffset=", xodr_map[i])
                                        roadMark.type = self._get_param(" type=", xodr_map[i])
                                        roadMark.material = self._get_param(" material=", xodr_map[i])
                                        roadMark.color = self._get_param(" color=", xodr_map[i])
                                        roadMark.width = self._get_param(" width=", xodr_map[i])
                                        roadMark.laneChange = self._get_param(" laneChange=", xodr_map[i])
                                        lane.roadMark.append(roadMark)
                                    elif (xodr_map[i].find("<userData>") != -1):
                                        userData = T4ac_VectorLane()
                                        userData.sOffset = self._get_param(" sOffset=", xodr_map[i])
                                        userData.laneId = self._get_param(" laneId=", xodr_map[i])
                                        userData.travelDir = self._get_param(" travelDir=", xodr_map[i])
                                        lane.userData.append(userData)
                                    elif (xodr_map[i].find("</lane>") != -1):
                                        in_lane = 0
                                        laneSection.center.append(lane)     
                    
                        elif (in_right == 1):
                            if (in_lane == 0):
                                if (xodr_map[i].find("<lane ") != -1):
                                    in_lane = 1
                                    lane = T4ac_Lane()
                                    lane.id = int(self._get_param(" id=", xodr_map[i]))
                                    lane.type = self._get_param(" type=", xodr_map[i])
                                    lane.level = self._get_param(" level=", xodr_map[i])
                                elif (xodr_map[i].find("</right>") != -1):
                                        in_right = 0
                            elif (in_lane == 1):
                                if (in_laneLink == 1):
                                    if (xodr_map[i].find("<predecessor ") != -1):
                                        lane.link.predecessor.id = int(self._get_param(" id=", xodr_map[i]))
                                    elif (xodr_map[i].find("<successor ") != -1):
                                        lane.link.successor.id = int(self._get_param(" id=", xodr_map[i]))
                                    elif (xodr_map[i].find("</link>") != -1):
                                        in_laneLink = 0
                                elif (in_laneLink == 0):
                                    if (xodr_map[i].find("<link>") != -1):
                                        in_laneLink = 1
                                    elif (xodr_map[i].find("<width ") != -1):
                                        width = T4ac_LaneWidth()
                                        width.sOffset = float(self._get_param(" sOffset=", xodr_map[i]))
                                        width.a = float(self._get_param(" a=", xodr_map[i]))
                                        width.b = float(self._get_param(" b=", xodr_map[i]))
                                        width.c = float(self._get_param(" c=", xodr_map[i]))
                                        width.d = float(self._get_param(" d=", xodr_map[i]))
                                        lane.width.append(width)
                                    elif (xodr_map[i].find("<roadMark ") != -1):
                                        roadMark = T4ac_RoadMark()
                                        roadMark.sOffset = self._get_param(" sOffset=", xodr_map[i])
                                        roadMark.type = self._get_param(" type=", xodr_map[i])
                                        roadMark.material = self._get_param(" material=", xodr_map[i])
                                        roadMark.color = self._get_param(" color=", xodr_map[i])
                                        roadMark.width = self._get_param(" width=", xodr_map[i])
                                        roadMark.laneChange = self._get_param(" laneChange=", xodr_map[i])
                                        lane.roadMark.append(roadMark)
                                    elif (xodr_map[i].find("<userData>") != -1):
                                        userData = T4ac_VectorLane()
                                        userData.sOffset = self._get_param(" sOffset=", xodr_map[i])
                                        userData.laneId = self._get_param(" laneId=", xodr_map[i])
                                        userData.travelDir = self._get_param(" travelDir=", xodr_map[i])
                                        lane.userData.append(userData)
                                    elif (xodr_map[i].find("</lane>") != -1):
                                        in_lane = 0
                                        laneSection.right.append(lane)     


                elif (in_objects == 1):
                    if (in_outline == 0):                       
                        if (xodr_map[i].find("<object ") != -1):
                           _object = T4ac_Object()
                           _object.id = self._get_param(" id=", xodr_map[i])
                           _object.name = self._get_param(" name=", xodr_map[i])
                           _object.s = self._get_param(" s=", xodr_map[i])
                           _object.t = self._get_param(" t=", xodr_map[i])
                           _object.zOffset = self._get_param(" zOffset=", xodr_map[i])
                           _object.hdg = self._get_param(" hdg=", xodr_map[i])
                           _object.roll = self._get_param(" roll=", xodr_map[i])
                           _object.pitch = self._get_param(" pitch=", xodr_map[i])
                           _object.orientation = self._get_param(" orientation=", xodr_map[i])
                           _object.type = self._get_param(" type=", xodr_map[i])
                           _object.height = self._get_param(" height=", xodr_map[i])
                           _object.width = self._get_param(" width=", xodr_map[i])
                           _object.length = self._get_param(" length=", xodr_map[i])
                           road.objects.append(_object)
                        elif (xodr_map[i].find("<outline>") != -1):
                                in_outline = 1
                        elif (xodr_map[i].find("</objects>") != -1):
                            in_objects = 0

                    elif (in_outline == 1):
                        if (xodr_map[i].find("<cornerLocal ") != -1):
                            cornerLocal = T4ac_CornerLocal()
                            cornerLocal.u = self._get_param(" u=", xodr_map[i])
                            cornerLocal.v = self._get_param(" v=", xodr_map[i])
                            cornerLocal.z = self._get_param(" z=", xodr_map[i])
                            road.objects[-1].outline.append(cornerLocal)
                        elif (xodr_map[i].find("</outline>") != -1):
                            in_outline = 0


                elif (in_signals == 1):
                    if (in_signal == 0 and in_signalReference == 0):
                        if (xodr_map[i].find("<signal ") != -1):
                            in_signal = 1
                            signal = T4ac_Signal()
                            signal.name = self._get_param(" name=", xodr_map[i])
                            signal.id = self._get_param(" id=", xodr_map[i])
                            signal.s = self._get_param(" s=", xodr_map[i])
                            signal.t = self._get_param(" t=", xodr_map[i])
                            signal.zOffset = self._get_param(" zOffset=", xodr_map[i])
                            signal.hOffset = self._get_param(" hOffset=", xodr_map[i])
                            signal.roll = self._get_param(" roll=", xodr_map[i])
                            signal.pitch = self._get_param(" pitch=", xodr_map[i])
                            signal.orientation = self._get_param(" orientation=", xodr_map[i])
                            signal.dynamic = self._get_param(" dynamic=", xodr_map[i])
                            signal.country = self._get_param(" country=", xodr_map[i])
                            signal.type = self._get_param(" type=", xodr_map[i])
                            signal.subtype = self._get_param(" subtype=", xodr_map[i])
                            signal.value = self._get_param(" value=", xodr_map[i])
                            signal.text = self._get_param(" text=", xodr_map[i])
                            signal.height = self._get_param(" height=", xodr_map[i])
                            signal.width = self._get_param(" width=", xodr_map[i])
                        elif (xodr_map[i].find("<signalReference ") != -1):
                            in_signalReference = 1
                            signalReference = T4ac_SignalReference()
                            signalReference.id = self._get_param(" id=", xodr_map[i])
                            signalReference.s = self._get_param(" s=", xodr_map[i])
                            signalReference.t = self._get_param(" t=", xodr_map[i])
                            signalReference.orientation = self._get_param(" orientation=", xodr_map[i])
                        elif (xodr_map[i].find("</signals>") != -1):
                            in_signals = 0
                    elif (in_signal == 1 and in_signalReference == 0):
                        if (xodr_map[i].find("<validity ") != -1):
                            signal.validity.fromLane = self._get_param(" fromLane=", xodr_map[i])
                            signal.validity.toLane = self._get_param(" toLane=", xodr_map[i])
                        elif (xodr_map[i].find("<userData>") != -1):
                            signal.userData.signalId = self._get_param(" signalId=", xodr_map[i])
                            signal.userData.gateId = self._get_param(" gateId=", xodr_map[i])
                            signal.userData.turnRelation = self._get_param(" turnRelation=", xodr_map[i])
                        elif (xodr_map[i].find("</signal>") != -1):
                            in_signal = 0
                            road.signals.signal.append(signal)
                    elif (in_signal == 0 and in_signalReference == 1):
                        if (xodr_map[i].find("<validity ") != -1):
                            signalReference.validity.fromLane = self._get_param(" fromLane=", xodr_map[i])
                            signalReference.validity.toLane = self._get_param(" toLane=", xodr_map[i])
                        elif (xodr_map[i].find("<userData>") != -1):
                            signalReference.userData.signalId = self._get_param(" signalId=", xodr_map[i])
                            signalReference.userData.gateId = self._get_param(" gateId=", xodr_map[i])
                            signalReference.userData.turnRelation = self._get_param(" turnRelation=", xodr_map[i])
                        elif (xodr_map[i].find("</signalReference>") != -1):
                            in_signalReference = 0
                            road.signals.signalReference.append(signalReference)

        return roads

    def _get_junctions(self, xodr_map):
        junctions = []
        in_junction = 0
        in_connection = 0
        in_userData = 0

        for i in range(0, len(xodr_map)):
            if (in_junction == 0):
                if (xodr_map[i].find("<junction ") != -1):
                    in_junction = 1
                    junction = T4ac_Junction()
                    junction.id = int(self._get_param(" id=", xodr_map[i]))
                    junction.name = self._get_param(" name=", xodr_map[i])

            elif (in_junction == 1):
                if (in_connection == 0 and in_userData == 0):
                    if (xodr_map[i].find("<connection ") != -1):
                        in_connection = 1
                        connection = T4ac_Connection()
                        connection.id = int(self._get_param(" id=", xodr_map[i]))
                        connection.incomingRoad = int(self._get_param(" incomingRoad=", xodr_map[i]))
                        connection.connectingRoad = int(self._get_param(" connectingRoad=", xodr_map[i]))
                        connection.contactPoint = self._get_param(" contactPoint=", xodr_map[i])

                    elif (xodr_map[i].find("<controller ") != -1):
                        controller = T4ac_ControllerJunction()
                        controller.id = int(self._get_param(" id=", xodr_map[i]))
                        controller.type = int(self._get_param(" type=", xodr_map[i]))
                        controller.sequence = int(self._get_param(" sequence=", xodr_map[i]))
                        junction.controller.append(controller)

                    elif (xodr_map[i].find("<userData>") != -1):
                        in_userData = 1

                    elif (xodr_map[i].find("</junction>") != -1):
                        junctions.append(junction)
                        in_junction = 0

                elif (in_connection == 1):
                    if (xodr_map[i].find("<laneLink ") != -1):
                        laneLink = T4ac_LaneLinkJunction()
                        laneLink.fromLane = int(self._get_param(" from=", xodr_map[i]))
                        laneLink.toLane = int(self._get_param(" to=", xodr_map[i]))
                        connection.laneLink.append(laneLink)
                    elif (xodr_map[i].find("</connection>") != -1):
                        in_connection = 0
                        junction.connection.append(connection)

                elif (in_userData == 1):
                    if (xodr_map[i].find("<vectorJunction ") != -1):
                        vectorJunction = T4ac_VectorJunction()
                        vectorJunction.junctionId = self._get_param(" junctionId=", xodr_map[i])
                    elif (xodr_map[i].find("</userData>") != -1):
                        in_userData = 0
                        junction.userData = vectorJunction

        return junctions

    def _get_road_ids(self, roads):
        """
        Return a list with road ids
        This is usefull for getting an specific road in roads using its id
        """
        road_ids = []
        for road in roads:
            road_ids.append(road.id)
        return road_ids

    def _get_junction_ids(self, junctions):
        """
        Return a list with junctions ids
        This is usefull for getting an specific junction in junctions 
        using its id
        """
        junction_ids = []
        for junction in junctions:
            junction_ids.append(junction.id)
        return junction_ids
