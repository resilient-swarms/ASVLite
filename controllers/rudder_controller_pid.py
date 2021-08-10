import math
from wave import Wave
from asv import Asv_propeller, Asv_specification, Asv_dynamics, Asv
from geometry import Dimensions
import multiprocessing as mp
import numpy as np
from collections import deque

class Rudder_controller:
    def __init__(self, asv): 
        self.max_rudder_angle = 30 #deg
        self.asv = asv
        self.error = 0.0 # error in each time step
        self.previous_error  = 0.0 # error in the previous time step
        self.cumulative_error = deque(maxlen=25) # integral of errors
        self.delta_error = 0.0 # differential of error
        self.K = np.array([4, 2, 3]) # P,I,D gain terms
             
    def get_rudder_angle(self, waypoint):
        # Compute the relative angle between the vehicle heading and the waypoint.
        p1 = self.asv.origin_position
        p2 = self.asv.cog_position
        p3 = waypoint
        # Angle between two lines with slope m1, m2 = atan((m1-m2)/(1 + m1*m2))
        m1 = math.atan2((p2.y-p1.y), (p2.x-p1.x))
        m2 = math.atan2((p3.y-p1.y), (p3.x-p1.x))
        theta = math.atan((m1-m2)/(1 + m1*m2)) # radians
        theta = theta * 180.0 / math.pi # deg
        # Set error as the difference of the current heading and the desired heading.
        self.previous_error = self.error
        self.error = theta
        #print(self.error)
        self.cumulative_error.append(self.error)
        cumulative_error = np.array(self.cumulative_error).sum(axis=0)
        self.delta_error = self.error - self.previous_error
        # Compute the rudder angle
        E = np.array([self.error, cumulative_error, self.delta_error]) # P, I, D errors.
        phi = np.dot(np.transpose(self.K), E) # deg because error is in deg.
        # Limit the rudder angle within the range (-30, 30)
        if phi > 30.0:
            phi = 30.0
        elif phi < -30.0:
            phi = -30.0
        return phi
