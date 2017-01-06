import sys
import StringIO

import numpy as np

vertical = np.array([0, 1])
horizontal = np.array([1, 0])

black = 0, 0, 0
gray = 200, 200, 200
white = 255, 255, 255
blue = 180, 180, 255
red = 255, 100, 100
green = 100, 255, 100

def degrees(rad):
    return rad * 180/np.pi

def radians(deg):
    return deg * np.pi/180

def normalize(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = normalize(v1)
    v2_u = normalize(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def sigangle(v1, v2):
    """Signed angle between two vectors (-ve if v1 is CCW from v2)"""
    v1_u = normalize(v1)
    v2_u = normalize(v2)
    ang = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    # Right-angle to v2
    perp = [v2_u[1], -v2_u[0]]
    # Check handedness
    if np.dot(v1_u, perp) < 0:
        return -ang
    else:
        return ang

def rotate(v, angle):
    """Rotate a vector v; angle in radians"""
    mat = np.matrix([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]])
    return np.array(v * mat).squeeze()

def printVec(vec):
    """Pretty-print a floating point vector/np array"""
    print(prettyVec(vec))

def prettyVec(vec):
    """Pretty-format a floating point vector/np array"""
    out = StringIO.StringIO()
    out.write('[')
    for i in xrange(len(vec)):
        out.write('{0:.2f}'.format(vec[i]))
        if i < len(vec) - 1:
            out.write(', ')
    out.write(']')
    str = out.getvalue()
    out.close()
    return str
