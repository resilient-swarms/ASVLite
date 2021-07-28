import math
import random
import numpy as np
from wave import Wave
from asv import Asv_propeller, Asv_specification, Asv_dynamics, Asv
from geometry import Dimensions


class Rudder_controller:
    '''
    The required rudder angle is computed from the linear equation:
    phi = w_0 + w_1*theta + w_2*d + w_3*v
    where:
    phi     - controller's output, rudder angle.
    theta   - desired vehicle heading direction.
    d       - shortest distance between the vehicle and the path.
    v       - surge velocity of the vehicle.
    
    The above equation can be written using vectors as:
    phi = W^T . X 
    where:
    W = [w_0, w_1, w_2, w_3]
    X = [1, theta, d, v]
    '''
    def __init__(self, asv_spec):
        # Parameters
        self.W = np.ones(4) # Initialise to all ones. Actual values will be computed when training. 
        # Create ASV
        self.asv_spec = asv_spec
        # Compute the optimal parameters
        self._gradient_descent()
    
    def _gradient_descent(self):
        alpha = 0.01 # learning rate
        for i in range(100):
            d, cost = self._cost()
            self.W = self.W - alpha * cost 
            print(d)
    
    def _cost(self):
        '''
        Method to compute the cost of using the current set of parameters, W, for the controller.
        This method computes the cost by simulating path following with paths originating at (0, 0)
        and radiating out at angle in range (-180 deg, 180 deg) incrementing at 30 deg. The simulation is
        repeated for sea states with significant wave height ranging from 0.5m to 4m. The simulation is also
        repeated for 7 different starting positions - (0,0), (5,0), (5,5), (0, 5), (-5, 0), (-5, -5), (0, -5)
        Cost is computed as the cumulative of square of the deviations from the desired paths.
        '''
        significant_wave_hts = np.array([1.0])
        #significant_wave_hts = np.arange(0.5, 4.0, 0.5)
        # asv_headings = np.array([0])
        asv_headings = np.arange(-math.pi, math.pi, math.pi/6)
        # start_points = [(0,0,0), (5,0,0), (5,5,0), (0,5,0), (-5,0,0), (-5,-5,0), (0,-5,0)]
        start_points = [(0,0,0)]
        # Simulate path and compute the cost:
        time_step_size = 100 # milli-sec
        cumulative_error = np.array([0.0, 0.0, 0.0, 0.0])
        cumulative_d = 0.0
        n = 0
        for significant_wave_ht in significant_wave_hts:
            for asv_heading in asv_headings:
                for start_point in start_points:
                    asv = Asv()
                    asv.spec = self.asv_spec
                    asv.dynamics.time_step_size = time_step_size/1000.0 # sec
                    asv.dynamics.time = 0.0
                    # The equation of a line baced on the slope is y = mx + c. But since all the path (lines)
                    # passes through the origin the above equation can be written as y = mx (ie. c = 0).
                    asv.origin_position = Dimensions(*start_point)
                    asv.attitude = Dimensions(0.0, 0.0, 0.0)
                    # The general equation of line: Ax + By + C = 0. 
                    A = math.tan(asv_heading)
                    B = -1
                    C = 0
                    # Create wave
                    rand_seed = random.randint(1, 3)
                    wave_heading = 0.0 # The wave heading does not have a significant affect on the vehicle dynamics. So always assuming it as a constant. 
                    wave = Wave(significant_wave_ht, wave_heading, rand_seed)
                    # Init ASV
                    asv.init(wave)  
                    # Simulate for a fixed number of time steps. 
                    count_time_steps = 1000
                    for t in range(count_time_steps): 
                        # Compute the shortest distance from vehicle to the path.
                        # For a line Ax + By + C = 0, the shortest distance of point (x_t, y_t) to line = |A.x_t + B.y_t + C|/sqrt(A^2 + B^2)
                        x_t = asv.cog_position.x
                        y_t = asv.cog_position.y
                        d = abs(A*x_t + B*y_t + C)/math.sqrt(A**2 + B**2)  
                        # Compute the dynamics for the current time step
                        time = t * asv.dynamics.time_step_size # sec
                        required_heading = asv_heading
                        surge_velocity = asv.dynamics.V[0] 
                        rudder_angle = self.get_rudder_angle(required_heading, d, surge_velocity)
                        asv.compute_dynamics(rudder_angle, time)
                        # Compute the error in the vehicle's path
                        # Error is computed as the square of diveations from the path.
                        cumulative_error += np.array([d*1, d*required_heading, d*d, d*surge_velocity])
                        cumulative_d += d
                        n += 1
                        # # Print the output
                        # print(asv.cog_position.x, asv.cog_position.y)
        return cumulative_d/n, cumulative_error/n
             
    def get_rudder_angle(self, required_heading, shortest_distance, surge_velocity):
        X = np.array([1.0, required_heading, shortest_distance, surge_velocity])
        angle = np.dot(np.transpose(self.W), X)
        return angle
