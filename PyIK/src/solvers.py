from __future__ import print_function

import math
import pdb

import numpy as np

from util import *

class Circle:
    def __init__(self, center, radius):
        self.c = np.array(center)
        self.r = radius

    def intersect(self, other):
        dist = np.linalg.norm(self.c - other.c)
        if dist > self.r + other.r:
            # No contact
            return None
        elif dist == 0 and self.r == other.r:
            # Coincident
            return np.inf
        elif dist + min(self.r, other.r) < max(self.r, other.r):
            # Contained
            return None
        else:
            # Two intersections
            a = (self.r**2 - other.r**2 + dist**2) / (2 * dist)
            h = np.sqrt(self.r**2 - a**2)

            p2 = self.c + (a * (other.c - self.c)) / dist
            i1 = np.array(p2)
            i1[0] += h * (other.c[1] - self.c[1]) / dist
            i1[1] -= h * (other.c[0] - self.c[0]) / dist
            i2 = np.array(p2)
            i2[0] -= h * (other.c[1] - self.c[1]) / dist
            i2[1] += h * (other.c[0] - self.c[0]) / dist
            return i1, i2

class PhysicalSolver:
    def __init__(self, len_main, len_linkage, len_ac_lower, len_ac_upper):
        self.len_main = len_main
        self.len_linkage = len_linkage
        self.ac_lower = len_ac_lower
        self.ac_upper = len_ac_upper

    def solve_forearm(self, left, right):
        base_angle = left - right
        A = self.len_linkage
        B = self.ac_upper
        C = self.len_main
        D = self.ac_lower
        # Repeated application of cosine rule yields the forearm angle
        # X is a diagonal across the irregular quatrilateral (opposite
        # base_angle)
        Xsq = D**2 + C**2 - 2*D*C*np.cos(base_angle)
        X = np.sqrt(Xsq)
        # foo and bar are the two angles adjacent to X in the quat
        cosFoo = np.clip((Xsq + C**2 - D**2) / (2*X*C), -1, 1)
        cosBar = np.clip((Xsq + B**2 - A**2) / (2*X*B), -1, 1)
        foo = np.arccos(cosFoo)
        bar = np.arccos(cosBar)
        # together they form the angle between the main arm and forearm
        return foo + bar

    def inverse_forearm(self, desired):
        A = self.len_linkage
        B = self.ac_upper
        C = self.len_main
        D = self.ac_lower
        # Repeated application of cosine rule yields the forearm angle
        # Y is a diagonal across the irregular quatrilateral (opposite
        # desired)
        Ysq = C**2 + B**2 - 2*C*B*np.cos(desired)
        Y = np.sqrt(Ysq)
        # foo and bar are the two angles adjacent to Y in the quat
        cosFoo = np.clip((Ysq + D**2 - A**2) / (2*Y*D), -1, 1)
        cosBar = np.clip((Ysq + C**2 - B**2) / (2*Y*C), -1, 1)
        foo = np.arccos(cosFoo)
        bar = np.arccos(cosBar)
        # together they form the angle between the main arm and actuator
        base_angle = foo + bar
        return base_angle

class IKSolver:
    def __init__(self, len0, len1, wrist_len, base_offset, origin = [0, 0, 0]):
        self.origin = np.array(origin)
        self.elbow = np.array([0, 0])
        self.radial = 0
        self.swing = 0
        self.len0 = len0
        self.len1 = len1
        self.wrist_len = wrist_len
        self.base_offset = base_offset
        self.wrist_x = 0.0
        self.wrist_y = 0.0
        self.wrist_normal = np.array([0,0,1])

    def setWristDir(self, normal):
        """Stores a 3D target normal vector - used to angle the wrist joint"""
        self.wrist_normal = normal
        self.calcWristAngles()

    def calcWristAngles(self):
        """
        Calculate the yaw and pitch for the wrist joints to reach the desired
        wrist normal direction
        """
        (x,y,z) = self.wrist_normal
        self.wrist_x = np.pi/2 - np.arctan2(z, x) - self.swing
        self.wrist_y = np.arctan2(-y, z)

    def setGoal(self, goal):
        """
        Set 3D end-effector goal point for IK.
        Returns True if the resulting configuration is valid.
        """
        self.goal = np.array(goal)
        return self.resolveIK()

    def shoulder(self, theta):
        """Shoulder position (top-down) given swing angle"""
        return np.array([
            # base offset is [x, z]
            self.base_offset[1]*np.sin(theta) + self.base_offset[0]*np.cos(theta),
            self.base_offset[1]*np.cos(theta) - self.base_offset[0]*np.sin(theta)
        ])

    def resolveIK(self):
        # Top-down IK - resolve swing angle and radial distance
        # NOTE 'td' here means "top-down", while 'pl' means "planar" or side-on
        self.origintd = np.array([self.origin[0], self.origin[2]])
        self.goaltd = np.array([self.goal[0], self.goal[2]]) # X and Z position
        deltatd = self.goaltd - self.origintd

        # Resolve swing angle by bisection method: point straight at the
        # target as a starting point
        start_theta = np.arctan2(deltatd[0], deltatd[1])
        # Resting angle to shoulder joint
        shoulder_theta = sigangle(self.base_offset, vertical)
        # print ("Shoulder theta ",shoulder_theta)
        if shoulder_theta < 0:
            # start_theta will be < target
            theta_high = start_theta - shoulder_theta
            theta_low = start_theta
        else:
            # start_theta will be > target
            theta_high = start_theta
            theta_low = start_theta - shoulder_theta
        # print ("Starting low ", theta_low)
        # print ("Starting high ", theta_high)
        # Bisection loop
        iters = 0
        while iters < 20:
            self.swing = (theta_high + theta_low) * 0.5
            f_mid = self.swing - sigangle(
                self.goaltd-self.shoulder(self.swing),
                vertical
            )
            # print (f_mid)
            if abs(f_mid) < 0.005:
                break
            else:
                if f_mid < 0:
                    theta_low = self.swing
                    # print ("New low ", theta_low)
                else:
                    theta_high = self.swing
                    # print ("New high ", theta_high)
            iters += 1
        # if (iters > 2):
        #     print(iters)

        # Radial distance (shoulder to goal)
        self.radial = np.linalg.norm(self.goaltd - self.shoulder(self.swing))

        # Planar IK - calculate elbow pos using circles about the shoulder and
        # wrist joints
        zoffs = self.base_offset[1]
        self.goalpl = np.array([zoffs + self.radial, self.goal[1]])
        # Wrist is offset back from target end-effector position
        self.wristpl = self.goalpl - [self.wrist_len, 0]
        self.originpl = np.array([self.origin[0] + zoffs, self.origin[1]])

        c1 = Circle(self.originpl, self.len0)
        c2 = Circle(self.wristpl, self.len1)
        points = c1.intersect(c2)
        if points is not None and points != np.inf:
            # valid, pick higher point
            self.valid = True
            if points[0][1] > points[1][1]:
                self.elbow = points[0]
            else:
                self.elbow = points[1]
        else:
            self.valid = False
        # Recalculate wrist angles
        self.calcWristAngles()
        # Return the validity flag
        return self.valid
