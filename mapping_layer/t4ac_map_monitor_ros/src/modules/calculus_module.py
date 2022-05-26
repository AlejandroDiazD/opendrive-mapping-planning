"""
Module to implement some generic calculus operations
"""

from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
import numpy as np

from . import monitor_classes

def inside_polygon(position, polygon):
    """
    This functions checks if a point is inside a certain lane (or area), 
    a.k.a. polygon.
    (https://jsbsan.blogspot.com/2011/01/saber-si-un-punto-esta-dentro-o-fuera.html)
    Takes a point and a polygon and returns if it is inside (1) or outside(0).
    
    Args:
        position: Position (x,y) of the point to be ehcked if is inside
        polygon: Polygon defined by a list of points (x,y)

    Returns: 
        Return True if position is inside the polygon and False if is outside
    """
    counter = 0
    xinters = 0
    detection = False

    p1 = monitor_classes.Node2D()
    p2 = monitor_classes.Node2D()

    p1 = polygon[0] # First column = x coordinate. Second column = y coordinate

    for i in range(1,len(polygon)+1):
        p2 = polygon[i%len(polygon)]

        if (position.y > min(p1.y,p2.y)):
            if (position.y <= max(p1.y,p2.y)):
                if (position.x <= max(p1.x,p2.x)):
                    if (p1.y != p2.y):
                        xinters = (position.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x
                        if (p1.x == p2.x or position.x <= xinters):
                            counter += 1
        p1 = p2

    if (counter % 2 == 0):
        detection = False
    else:
        detection = True
    #print("Is inside: ", detection)
    return detection

def fit_velocity_braking_distance_model():
    """
    This function creates a regression model to determine the braking distance
    in function of the velocity. It needs two arrays to create the model

    Data: https://cdn3.capacitateparaelempleo.org/assets/76k9ldq.pdf
    Theory: http://www.dgt.es/PEVI/documentos/catalogo_recursos/didacticos/did_adultas/velocidad.pdf
    """
    velocity_braking_distance_model = LinearRegression()
    a = np.array((40,50,60,70,80,90,100,110,120,130,140)).reshape(11,1) 
    b = np.array((18.62,26.49,35.65,46.09,57.82,70.83,85.13,100.72,117.59,
                  135.75,155.20)).reshape(11,1)
    pf = PolynomialFeatures(degree = 2)
    a = pf.fit_transform(a.reshape(-1,1)) # The x-axis is transformed to polynomic
    velocity_braking_distance_model.fit(a, b)
    return velocity_braking_distance_model, pf

def braking_n_distance(odometry):
    """
    Calculate how many waypoints (n) has to monitorize depending on the 
    velocity of the ego_vehicle. For that purpose third party function is
    used (fit_velocity_braking_distance_model())

    Args:
        odometry: Value nav_msgs/Odometry.msg of the ego_vehicle

    Returns:
        n: Number of waypoints in front of the vehicle to monitorize
    """
    # Calculate braking distance
    vel_km_h = odometry.twist.twist.linear.x*3.6 # m/s to km/h
    velocity_braking_distance_model, pf = (
        fit_velocity_braking_distance_model())
    braking_distance = velocity_braking_distance_model.predict(
        pf.fit_transform([[vel_km_h]]))

    # Calculate n waypoints
    n = int(braking_distance[0,0])
    return n

    