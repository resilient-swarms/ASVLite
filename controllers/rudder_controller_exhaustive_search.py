import math
import random
import numpy as np
from wave import Wave
from asv import Asv_propeller, Asv_specification, Asv_dynamics, Asv
from geometry import Dimensions
import multiprocessing as mp
from tqdm import tqdm # to show a progress bar

class Rudder_controller:
    '''
    The required rudder angle is computed from the linear equation:
    phi = w_0 + w_1*sin(theta) + w_2*cos(theta) + w_3*v
    where:
    phi     - controller's output, rudder angle.
    theta   - relative angle between vehicle heading and the waypoint.
    v       - surge velocity of the vehicle.
    
    The above equation can be written using vectors as:
    phi = W^T . X 
    where:
    W = [w_0, w_1, w_2, w_3]
    X = [1, sin(theta), cos(theta), v]
    '''
    def __init__(self, asv_spec): 
        self.max_rudder_angle = 30 #deg
        self.asv_spec = asv_spec
        #self.W = np.ones(4) # Initialise to all ones. Actual values will be computed when training.
        # Compute the optimal parameters
        self._exhaustive_search()
    
    def _exhaustive_search(self):
        Ws = []
        # There is a lot of simulations to run, therefore run it as multiple processes to save time.
        pool = mp.Pool(mp.cpu_count()) # Create a pool of processes. 
        # for w_0 in np.arange(-2.0, 2.0, 0.1):
        #     for w_1 in np.arange(-2.0, 2.0, 0.1):
        #         for w_2 in np.arange(-2.0, 2.0, 0.1):
        #             for w_3 in np.arange(-2.0, 2.0, 0.1):
        for w_0 in np.arange(-100.0, 100.0, 5.0):
            for w_1 in np.arange(-100.0, 100.0, 5.0):
                for w_2 in np.arange(-100.0, 100.0, 5.0):
                    for w_3 in np.arange(-100.0, 100.0, 5.0):
                        Ws.append([w_0, w_1, w_2, w_3])
        # This is going to take very long time therefore show a progress bar to see the progress and remaining time for completion.
        results = [] 
        for result in tqdm(pool.imap_unordered(self._simulations, Ws), total=len(Ws)): # tqdm adds and manages the progress bar.
            results.append(result) # append the return for each call to self._simulations to the list.
        #time_logs = pool.map(self._simulations, Ws) # If I was not using a progress bar, then just this line was sufficient for multiprocessing all simulations. 
        # Write time log to file
        time_logs = results
        f = open("./time_log.txt", "w")
        f.write("{}".format(time_logs))
        f.close()
    
    def _simulations(self, W):
        # Log time for all simulations
        time_log = []
        # Simulate path and compute the cost:
        for significant_wave_ht in [2.0]:
            for start_point in [(5,0,0)]:
                for asv_heading in [90]:
                    time = self._simulate_condition(significant_wave_ht, start_point, asv_heading, W)
                    time_log.append(time)
        avg_time = np.average(np.array(time_log)) # Average of simulation time for the current set of parameters
        #print(W, avg_time)
        if avg_time < 50:
            # The vehicle reached waypoint before max_time. This set of parameters may be the solution, therefore save them on a file.
            f = open("./solution.txt", "a")
            f.write("{}, {}\n".format(W, avg_time))
            f.close()
        return [W, avg_time]
    
    def _simulate_condition(self, significant_wave_ht, start_point, asv_heading, W):
        # Log the path
        path = []
        waypoint = Dimensions(15.0, 15.0, 0)
        # Create wave
        rand_seed = random.randint(1, 100)
        wave_heading = 0.0 # The wave heading does not have a significant affect on the vehicle dynamics. So always assuming it as a constant. 
        wave = Wave(significant_wave_ht, wave_heading, rand_seed)
        # Create and init the vehicle for simulation.
        asv = Asv()
        asv.spec = self.asv_spec
        time_step_size = 40.0 # milli-sec
        asv.dynamics.time_step_size = time_step_size/1000.0 # sec
        asv.origin_position = Dimensions(*start_point)
        asv.attitude = Dimensions(0.0, 0.0, asv_heading*math.pi/180.0)
        asv.init(wave)  
        # Simulate for a fixed amount of time. 
        max_time = 50.0 #sec
        time = 0.0
        asv.dynamics.time = 0.0
        while time < max_time: 
            # Compute the relative angle between the vehicle heading and the path.
            p1 = asv.origin_position
            p2 = asv.cog_position
            p3 = waypoint
            # Angle between two lines with slope m1, m2 = atan((m1-m2)/(1 + m1*m2))
            m1 = math.atan2((p2.y-p1.y), (p2.x-p1.x))
            m2 = math.atan2((p3.y-p1.y), (p3.x-p1.x))
            theta = math.atan((m1-m2)/(1 + m1*m2)) # radians
            # Get the rudder angle
            v = asv.dynamics.V[0] / 10.0 # Normalize velocity
            phi = self._get_rudder_angle(theta, v, W) # rudder angle in deg
            # Compute the distance of the vehicle from the waypoint
            delta_y = waypoint.y - asv.origin_position.y
            delta_x = waypoint.x - asv.origin_position.x
            distance = math.sqrt(delta_x**2 + delta_y**2)
            proximity_margin = 2.0
            # Compute the dynamics for the current time step
            if distance > proximity_margin:
                time += (time_step_size/1000.0)
                asv.compute_dynamics(phi, time)
            else:
                # Reached the waypoint
                break
        return time
             
    def _get_rudder_angle(self, theta, v, W):
        W = np.array(W)
        X = np.array([1.0, math.sin(theta), math.cos(theta), v])
        angle = math.tanh(np.dot(np.transpose(W), X)) # This will give a value between -1 and 1
        angle = angle * self.max_rudder_angle 
        return angle
