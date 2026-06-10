#!/usr/bin/env python3

from math import pi, floor
from math import sin, cos
from math import sqrt
from math import atan2, asin, acos
import numpy as np
import copy

## TODO ##
# Imprement your direct and inverse kinematics here

class Kinematics():

    def __init__(self):
        # Definiraj varijable
        return

    def get_dk(self, q):
        # Implement here direct kinematics
        # INPUT: q as a vector 4x1
        # OUTPUT: w as a vector 6x1

        # TODO:


        # OUTPUT:
        w = np.zeros(6)
        #w[0] = ...
        #w[1] = ...
        #w[2] = ...
        #w[3] = ...
        #w[4] = ...
        #w[5] = ...

        #return w
        return "This function has not been implemented yet."

    def get_ik(self, w, q0 = None):
        # Implement here inverse kinematics
        # INPUT (1): w 6x1 as a tool configuration vector, w = [x, y, z, wx, wy, wz]
        # OUTPUT: q_all 4xN as all inverse solutions

        # TODO

        # Output
        q_all = []
        # q_closest = self.get_closest_ik(q_all, q0)

        # return q_closest
        return "This function has not been implemented yet."
        

    def get_closest_ik(self, q_all, q0):
        # Find closest IK solution to robot pose q0
        # INPUT (1): q_all all IK solutions, 6xN
        # INPUT (2): q0 current joint state configuration
        # OUTPUT: q 6x1

        # TODO:

        # Output
        q = np.zeros(6)
        #q[0] = ..
        #q[1] = ..
        #q[2] = ..
        #q[3] = ..
        #q[4] = ..
        #q[5] = ..
        #return q
        return "This function has not been implemented yet."
    
    def wrap2PI(self, x):
        return (x-2*pi*floor(x/(2*pi)+0.5))
