from math import pi
import numpy as np

rel_coord = np.array([pi/2, 8])
def geographic_coordinates_to_relative_coordinates(latitude: float, longitude: float):
    """Method to convert a latitude longitude coordinate into a relative coordinate for the rover"""
    global rel_coord
    delta = (np.random.rand(2)-0.5)*(2*pi/10, 1)
    rel_coord+=delta*0.1
    return rel_coord