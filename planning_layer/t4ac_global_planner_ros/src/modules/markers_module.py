"""
Module that implement functions to generate util marker objects to represent
different elements of the map in RVIZ, adapted for planning layer
"""
import rospy
import visualization_msgs.msg
import geometry_msgs.msg

def get_waypoint(waypoint, rgb = [0,0,0], lifetime = 0.2, scale = 0.2):
    """
    Get way marker to represent a T4ac_Waypoint in RVIZ

    Args:
        waypoint: T4ac_Waypoint()
        rgb: Colour
        lifetime: Lifetime of the marker in seconds
        scale: Scale for the marker

    Returns:
        waypoint_marker: Marker to represent a waypoint in RVIZ
    """
    waypoint_marker = visualization_msgs.msg.Marker()
    waypoint_marker.header.frame_id = "map"
    waypoint_marker.header.stamp = rospy.Time.now()
    waypoint_marker.ns = "waypoint_marker"
    waypoint_marker.action = visualization_msgs.msg.Marker.ADD
    waypoint_marker.pose.orientation.w = 1.0
    waypoint_marker.id = 0
    waypoint_marker.type = visualization_msgs.msg.Marker.POINTS
    waypoint_marker.color.r = rgb[0]
    waypoint_marker.color.g = rgb[1]
    waypoint_marker.color.b = rgb[2]
    waypoint_marker.color.a = 1.0
    waypoint_marker.scale.x = scale
    waypoint_marker.scale.y = scale
    waypoint_marker.lifetime = rospy.Duration(lifetime)

    point = geometry_msgs.msg.Point()
    point.x = waypoint.transform.location.x
    point.y = waypoint.transform.location.y
    point.z = waypoint.transform.location.z
    waypoint_marker.points.append(point)

    return waypoint_marker

def get_waypoints(waypoint_list, rgb = [0,0,0], lifetime = 0.2, scale = 0.2):
    """
    Get waypoints marker to represent a list of T4ac_Waypoint in RVIZ

    Args:
        waypoint_list: List of T4ac_Waypoint()
        rgb: Colour
        lifetime: Lifetime of the marker in seconds
        scale: Scale for the marker

    Returns:
        waypoints_marker: Marker to represent a waypoint list in RVIZ
    """
    waypoints_marker = visualization_msgs.msg.Marker()
    waypoints_marker.header.frame_id = "map"
    waypoints_marker.header.stamp = rospy.Time.now()
    waypoints_marker.ns = "waypoints_marker"
    waypoints_marker.action = visualization_msgs.msg.Marker.ADD
    waypoints_marker.pose.orientation.w = 1.0
    waypoints_marker.id = 0
    waypoints_marker.type = visualization_msgs.msg.Marker.LINE_STRIP
    waypoints_marker.color.r = rgb[0]
    waypoints_marker.color.g = rgb[1]
    waypoints_marker.color.b = rgb[2]
    waypoints_marker.color.a = 0.6
    waypoints_marker.scale.x = scale
    waypoints_marker.lifetime = rospy.Duration(lifetime)

    for waypoint in waypoint_list:
        point = geometry_msgs.msg.Point()
        point.x = waypoint.transform.location.x
        point.y = waypoint.transform.location.y
        point.z = waypoint.transform.location.z
        waypoints_marker.points.append(point)
    return waypoints_marker

def get_point(x, y, z, rgb = [0,0,0], lifetime = 0.2, scale = 0.2):
    """
    Get way marker to represent a xyz point in RVIZ

    Args:
        x, y, z: (flt, flt, flt)
        rgb: Colour
        lifetime: Lifetime of the marker in seconds
        scale: Scale for the marker

    Returns:
        point_marker: Marker to represent a xyz point in RVIZ
    """
    point_marker = visualization_msgs.msg.Marker()
    point_marker.header.frame_id = "map"
    point_marker.header.stamp = rospy.Time.now()
    point_marker.ns = "point_marker"
    point_marker.action = visualization_msgs.msg.Marker.ADD
    point_marker.pose.orientation.w = 1.0
    point_marker.id = 0
    point_marker.type = visualization_msgs.msg.Marker.POINTS
    point_marker.color.r = rgb[0]
    point_marker.color.g = rgb[1]
    point_marker.color.b = rgb[2]
    point_marker.color.a = 1.0
    point_marker.scale.x = scale
    point_marker.scale.y = scale
    point_marker.lifetime = rospy.Duration(lifetime)

    point = geometry_msgs.msg.Point()
    point.x = x
    point.y = y
    point.z = z
    point_marker.points.append(point)

    return point_marker