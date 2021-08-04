import math
from wave import Wave
from asv import Asv_propeller, Asv_specification, Asv_dynamics, Asv
from geometry import Dimensions
import multiprocessing as mp

class Rudder_controller:
    def __init__(self, asv): 
        self.max_rudder_angle = 30 #deg
        self.asv = asv
             
    def get_rudder_angle(self, waypoint):
        # Compute the relative angle between the vehicle heading and the path.
        p1 = self.asv.origin_position
        p2 = self.asv.cog_position
        p3 = waypoint
        # Angle between two lines with slope m1, m2 = atan((m1-m2)/(1 + m1*m2))
        m1 = math.atan2((p2.y-p1.y), (p2.x-p1.x))
        m2 = math.atan2((p3.y-p1.y), (p3.x-p1.x))
        theta = math.atan((m1-m2)/(1 + m1*m2)) # radians
        theta = theta * 180.0 / math.pi # deg
        # Limit the rudder angle in range (-30, 30)
        if theta < -30.0:
            return -30.
        elif theta > 30.0:
            return 30.0
        else:
            return theta
