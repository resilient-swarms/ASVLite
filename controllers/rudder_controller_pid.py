import math
from wave import Wave
from asv import Asv_propeller, Asv_specification, Asv_dynamics, Asv
from geometry import Dimensions
import multiprocessing as mp
import numpy as np
from collections import deque
from random import randrange
from tqdm import tqdm # to show a progress bar

class Rudder_controller:
    def __init__(self, asv_spec, K=None): 
        self.max_rudder_angle = 30 #deg
        self.asv_spec = asv_spec
        self.error = 0.0 # error in each time step
        self.previous_error  = 0.0 # error in the previous time step
        self.cumulative_error = deque(maxlen=25) # integral of errors
        self.delta_error = 0.0 # differential of error
        # P,I,D gain terms
        if K == None:
            for self.i in tqdm(range(10), leave=False):
                P = randrange(10)
                I = randrange(10)
                D = randrange(10)
                self.K = np.array([P,I,D])
                #self.K = np.array([6.0, 2.0, 3.0]) # Set an initial value and then tune the parameters. 
                self._tune_controller()
        else:
            self.K = np.array(K)        
             
    def get_rudder_angle(self, asv, waypoint, using_earth_coordinate_system):
        # Compute the relative angle between the vehicle heading and the waypoint.
        theta = None
        p1 = asv.origin_position
        p2 = asv.cog_position
        p3 = waypoint
        if using_earth_coordinate_system == True:
            # Ref: https://www.movable-type.co.uk/scripts/latlong.html
            lat1 = p1.x * math.pi/180.0
            lat3 = p3.x * math.pi/180.0
            long1 = p1.y * math.pi/180.0
            long3 = p3.y * math.pi/180.0
            theta1 = asv.attitude.z if asv.attitude.z < math.pi else asv.attitude.z - 2.0*math.pi 
            y2 = math.sin(long3-long1) * math.cos(lat3)
            x2 = math.cos(lat1)*math.sin(lat3) - math.sin(lat1)*math.cos(lat3)*math.cos(long3-long1)
            theta2 = math.atan2(y2, x2)
            theta = theta2 - theta1 # radians
            #print("{} {} {}".format(theta1 * 180/math.pi, theta2 * 180/math.pi, theta*180/math.pi))
        else:
            # Angle between two lines with slope m1, m2 = atan((m1-m2)/(1 + m1*m2))
            m1 = math.atan2((p2.y-p1.y), (p2.x-p1.x))
            m2 = math.atan2((p3.y-p1.y), (p3.x-p1.x))
            theta = math.atan((m1-m2)/(1 + m1*m2)) # radians
        # Convert theta to deg
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
   
    def _simulate_asv_in_sea_state(self, sea_state_and_PID):
        significant_wave_ht, wave_heading, P, I, D = sea_state_and_PID
        start_point = Dimensions(1000.0, 1000.0, 0)
        waypoint = Dimensions(1000.0, 1500.0, 0)
        # Init waves
        rand_seed = randrange(100)
        wave = Wave(significant_wave_ht, wave_heading, rand_seed)
        # Init ASV
        asv = Asv()
        asv.spec = self.asv_spec
        time_step_size = 40 # milli sec
        asv.dynamics.time_step_size = time_step_size/1000.0 # sec
        asv.dynamics.time = 0.0
        asv.origin_position = start_point
        asv.attitude = Dimensions(0.0, 0.0, 90.0)
        asv.init(wave)
        # Init controller
        controller = Rudder_controller(asv, [P,I,D])
        # Simulate
        time = 0.0
        max_time = 60 # sec
        while time < max_time:
            time += time_step_size/1000.0
            # Compute the dynamics
            rudder_angle = controller.get_rudder_angle(asv, waypoint)
            asv.compute_dynamics(rudder_angle, time)
        # Compute the performance
        # TODO: Correct distance computation when using earth coordinate system.
        p1 = asv.origin_position
        p3 = waypoint
        starting_distance = 1000.0 # m
        final_distance = math.sqrt((p1.x - p3.x)**2 + (p1.y - p3.y)**2)
        return (starting_distance - final_distance)/starting_distance
    
    def _tune_controller(self):  
        f = open("./experiments/{}_{}_{}.txt".format(self.K[0], self.K[1], self.K[2]), "w")   
        pool = mp.Pool(mp.cpu_count()) # Create a pool of processes to run in parallel. 
        delta = 0.25
        P_current, I_current, D_current = list(self.K)     
        # Policy Gradient Reinforcement Learning
        iterations = tqdm(range(25), leave=False) # This is going to take some time, therefore show a progress bar.
        iterations.set_description("Policy iterations")
        for n in iterations: 
            performance_average = []
            PIDs = []
            for P in [P_current-delta, P_current, P_current+delta]:
                for I in [I_current-delta, I_current, I_current+delta]:
                    for D in [D_current-delta, D_current, D_current+delta]:
                        PIDs.append([P,I,D])
            PIDs = tqdm(PIDs, leave=False)   
            PIDs.set_description("Controller variants")         
            for PID in PIDs:
                P,I,D = PID
                sea_states_and_PID = []
                for significant_wave_ht in np.arange(1.0, 4.0, 0.5):
                    for wave_heading in np.arange(0.0, 360.0, 45.0):
                        sea_states_and_PID.append([significant_wave_ht, wave_heading, P, I, D])
                results = [] 
                for result in pool.imap_unordered(self._simulate_asv_in_sea_state, sea_states_and_PID): # Run multiple simulations in parallel
                    results.append(result) # append the return for each call to self._simulate_asv_in_sea_state to the list. 
                # Compute performance for each combination of PID:
                performance_average.append([P, I, D, np.average(np.array(results))])
            # Compute the next set of PID terms
            performance_average = np.array(performance_average)
            f.write("{} {} {} {} {}\n".format(  P_current, 
                                                I_current, 
                                                D_current, 
                                                np.average(performance_average, axis=0)[-1], 
                                                np.std(performance_average, axis=0)[-1]))
            def compute_average_performance(index, K):
                mask = []
                for item in performance_average:
                    mask_value = True if item[index] == K else False
                    mask.append(mask_value)
                average_performance = performance_average[mask].mean(axis=0)[-1]
                return average_performance
            # Compute the average performance for all cases with P_current-delta
            performance_average_P_minus = compute_average_performance(0, P_current-delta)
            # Compute the average performance for all cases with P_current
            performance_average_P = compute_average_performance(0, P_current)
            # Compute the average performance for all cases with P_current+delta
            performance_average_P_plus = compute_average_performance(0, P_current+delta)
            # Compute the average performance for all cases with I_current-delta
            performance_average_I_minus = compute_average_performance(1, I_current-delta)
            # Compute the average performance for all cases with I_current
            performance_average_I = compute_average_performance(1, I_current)
            # Compute the average performance for all cases with I_current+delta
            performance_average_I_plus = compute_average_performance(1, I_current+delta)
            # Compute the average performance for all cases with D_current-delta
            performance_average_D_minus = compute_average_performance(2, D_current-delta)
            # Compute the average performance for all cases with D_current
            performance_average_D = compute_average_performance(2, D_current)
            # Compute the average performance for all cases with D_current+delta
            performance_average_D_plus = compute_average_performance(2, D_current+delta)
            # Compute the Adjustment vector.
            A = [0,0,0] 
            # Adjustment for P
            if performance_average_P_minus < performance_average_P > performance_average_P_plus:
                A[0] = 0
            elif performance_average_P_plus > performance_average_P_minus:
                A[0] = 1
            else:
                A[0] = -1
            # Adjustment for I            
            if performance_average_I_minus < performance_average_I > performance_average_I_plus:
                A[1] = 0
            elif performance_average_I_plus > performance_average_I_minus:
                A[1] = 1
            else:
                A[1] = -1
            # Adjustment for D            
            if performance_average_D_minus < performance_average_D > performance_average_D_plus:
                A[2] = 0
            elif performance_average_D_plus > performance_average_D_minus:
                A[2] = 1
            else:
                A[2] = -1
            # Compute the new gain terms
            A = np.array(A)
            #A = A/np.linalg.norm(A)
            K_current = np.array([P_current, I_current, D_current])
            K_current = K_current + A*delta
            P_current, I_current, D_current = list(K_current)
            self.K = K_current
        f.close()

# p "performance.txt" u 4 w l, "performance.txt" u ($0):4:5 with yerrorbars
