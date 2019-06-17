# parse_sensehat

# Scripts to parse sensehat for raspberry pi. Uses lcm and stores original strings

# https://pythonhosted.org/sense-hat/api/#imu-sensor

# Author: Blair Thornton
# Date: 18/09/2017
# Modified: Finn Murphy IP - 06/04/2018
import os

import lcm,sys
import threading
import time
import math
import numpy as np
from control.pid import pid

sys.path.append("..")
import lcm_handler.config as lcm_global
from lib_sensors.sensehat.imu import imu
from lcm_handler.lcm_handler import lcm_handler
from lib_navigation.navigation.tracking_nav import tracking_nav
from lib_navigation.qualisys_tracking import qualisys_tracking
from lib_sensors.image.visual_track import visual_track

# some constants definitions to improve readability of channels / inputs mapping 
_SURGE = 0
_SWAY  = 1
_YAW   = 2

class pid_control(threading.Thread):
    def __init__(self,control_path,control_filename,surge_gain,sway_gain,yaw_gain,control_limits,behaviour_type):
        threading.Thread.__init__(self)    
        lc = lcm.LCM()
        
        #roll_field_of_view = 24.2 # in degrees, full angle
        #pitch_field_of_view = 19.0 # in degrees, full angle
        #roll_offset = -4 # how the roll sensor is set relative to the camera
        #pitch_offset = 0 # how the roll sensor is set relative to the camera

#	print ("PID Control thread started *******************")

        input=[]
        input_previous=[]
        input_derivative=[]
        input_integrated=[]
        input_additive=[]
        control_command=[]
        err_counter = 0 # add a new error counter for the bad
        #initialise control parameters
        for i in range(3):
          input.append(0)
          input_previous.append(0)
          input_derivative.append(0)
          input_integrated.append(0)
          control_command.append(0)
          input_additive.append(0)
      
        epoch_previous = round(time.time()*1000)/1000
        
        #enter infinite loop, stopped by ._Thread_stop()
        while True:
          #default just control heading
          input[0]=0
          input[1]=0
          input[2]=0

          # doing REMOTE_CONTROL ****************************************************************
          if behaviour_type == "remote_control":
            #get joystick channel information
            subscription=lc.subscribe("joystick", lcm_handler)
            ok=0
            while ok==0:
              ok=lc.handle_timeout(1000)
            
            lc.unsubscribe(subscription)

            # It can be decoded in lcm_global
            # dirty forwarding section.... IMPROVE THIS!
            joystick_data = lcm_global.lcm_joystick_axis
            print ("joystick data:", joystick_data )
            input[_SURGE] = -joystick_data[1]		# The SURGE is controlled by the vertical movement of the LEFT joystick
            input[_SWAY]  =  joystick_data[0]		# The SWAY is controlled by the lateral movement of the LEFT joystick
            input[_YAW]   =  joystick_data[2]		# The YAW is controlled by the horizontal movement of the RIGHT joystick

          # doing QUALISYS_TRACKING ****************************************************************
          if behaviour_type == "qualisys_tracking":
            #get qualisys input and waypoints
            subscription=lc.subscribe("qualisys", lcm_handler)
            ok=0
            while ok==0:
              ok=lc.handle_timeout(1000)
            
            lc.unsubscribe(subscription)

            #get waypoints
            subscription=lc.subscribe("tracking_nav", lcm_handler)
            ok=0
            while ok==0:
              ok=lc.handle_timeout(100)         
            
            #print lcm_global.lcm_tracking_waypoint            
            lc.unsubscribe(subscription)

            #Check data integrity for any error
            if lcm_global.lcm_qualisys_integrity == 0:
              err_counter += 1  # increase the error counter
              if err_counter > 5: # if error counter is higher than 5 consecutive bad packets, we stop
                input[0]=0
                input[1]=0
                input[2]=0
            # If data integrity is ok, we use it
            else:
      	      print "[pid_control] wp[]", lcm_global.lcm_tracking_waypoint[0], lcm_global.lcm_tracking_waypoint[1]
      	      print "[pid_control] pose[]", lcm_global.lcm_qualisys_pose[0], lcm_global.lcm_qualisys_pose[1]
      	      error_0 = lcm_global.lcm_tracking_waypoint[0]-lcm_global.lcm_qualisys_pose[0]
              error_1 = lcm_global.lcm_tracking_waypoint[1]-lcm_global.lcm_qualisys_pose[1]
              error_2 = lcm_global.lcm_tracking_waypoint[2]-lcm_global.lcm_qualisys_pose[2]
	            # for debugging purposes, we compute and then print the errors
      	      print "[pid_control] err[0]", error_0
      	      print "[pid_control] err[1]", error_1
      	      print "[pid_control] err[2]", error_2

      	      input[0] = error_0*math.cos(math.radians(lcm_global.lcm_qualisys_pose[2])) #inertial reference frame
              input[1] = error_1*math.sin(math.radians(lcm_global.lcm_qualisys_pose[2])) #inertial reference frame #inertial reference frame
              input[2] = error_2
              err_counter = 0;  # reset the error counter

            if input[2] > 180:
              input[2] = 360-input[2]
            if input[2] < -180:
              input[2] = 360+input[2]

          # doing VISUAL_TRACKING ******************************************************************
          if behaviour_type == "visual_tracking":
            #get visual input and waypoints

            subscription=lc.subscribe("visual_track", lcm_handler)
            ok=0
            while ok==0:
              ok=lc.handle_timeout(1000)

            lc.unsubscribe(subscription)

            #if doing visual tracking
            subscription=lc.subscribe("tracking_nav", lcm_handler)
            ok=0
            while ok==0:
              ok=lc.handle_timeout(100)

            #print lcm_global.lcm_tracking_waypoint
            lc.unsubscribe(subscription)
            # print(lcm_global.lcm_imu_orientation[0]) #Roll
            # print(lcm_global.lcm_imu_orientation[1]) #Pitch
            if lcm_global.lcm_visual_xy[0] == -9999 and lcm_global.lcm_visual_xy[1] == -9999:
              input[0]=0
              input[1]=0
            else:
              input[0]=lcm_global.lcm_tracking_waypoint[0]+lcm_global.lcm_visual_xy[0] #inertial reference frame
              input[1]=lcm_global.lcm_tracking_waypoint[1]+lcm_global.lcm_visual_xy[1] #inertial reference frame
              # input[0]=lcm_global.lcm_tracking_waypoint[0]+surge_inertial #inertial reference frame
              # input[1]=lcm_global.lcm_tracking_waypoint[1]+sway_inertial #inertial reference frame
              # input[0]=lcm_global.lcm_tracking_waypoint[0]+lcm_global.lcm_visual_xy[0]#visual sensor_surge*cos(heading)-visual sensor_sway*sin(heading)
              # print "input for surge_sway:", lcm_global.lcm_visual_xy[0], lcm_global.lcm_visual_xy[1]

            subscription=lc.subscribe("imu", lcm_handler)
            ok=0
            while ok==0:
              ok=lc.handle_timeout(100)
            #print lcm_global.lcm_imu_orientation
            lc.unsubscribe(subscription)

            input[2]=lcm_global.lcm_tracking_waypoint[2]-lcm_global.lcm_imu_orientation[2]

            if input[2] > 180:
              input[2] = 360-input[2]
            if input[2] < -180:
              input[2] = 360+input[2]

          # COMMON SECTION #########################################################################
          # APPLY THE PID CONTROLLER DEFINED IN THE VEHICLE.YAML
          # WARNING: a solution is to create a PID controller with gains [1 0 0], so the control_command is directly the remote_control input
          # here, the input[] is converted into the msg.pid() structure that will be read by pwm_thruster.py
          epoch_time = round(time.time()*1000)/1000
          time_interval = epoch_time-epoch_previous

          # WARNING: DISABLED ALL THE MOTORS FOR TESTING IN THE BUILDING
#          input[0]=0
#          input[1]=0
#          input[2]=0

          #take the variability of the time interval into account for the integration and for the derivative
          input_additive[_SURGE] = input[_SURGE]* time_interval
          input_additive[_SWAY]  = input[_SWAY] * time_interval
          input_additive[_YAW]   = input[_YAW]  * time_interval

          input_derivative = np.subtract(input, input_previous)/time_interval
          input_integrated = np.add(input_integrated, input_additive)

          control_command[_SURGE] = surge_gain[0]*input[_SURGE] + surge_gain[1]*input_derivative[_SURGE] + surge_gain[2]*input_integrated[_SURGE]
          if abs(control_command[_SURGE]) > control_limits[_SURGE]:
            control_command[_SURGE] = np.sign(control_command[_SURGE])*control_limits[_SURGE]

          control_command[_SWAY] = sway_gain[0]*input[_SWAY] + sway_gain[1]*input_derivative[_SWAY] + sway_gain[2]*input_integrated[_SWAY]
          if abs(control_command[1]) > control_limits[1]:
            control_command[1] = np.sign(control_command[1])*control_limits[1]

          control_command[_YAW] = yaw_gain[0]*input[_YAW] + yaw_gain[1]*input_derivative[_YAW] + yaw_gain[2]*input_integrated[_YAW]
          if abs(control_command[_YAW]) > control_limits[_YAW]:
            control_command[_YAW] = np.sign(control_command[_YAW])*control_limits[_YAW]
          #print "input:", input
          #print "thrust:", control_command

          pid_string = "'epoch_time':" + format(epoch_time,'.3f') +  ", 'surge_command':" + str(control_command[0]) + ", 'sway_command':" + str(control_command[1]) + ", 'yaw_command':" + str(control_command[2]) + ", 'P_input':" + "['surge':" + str(input[0]) +  ", 'sway':" + str(input[1]) +  ", 'yaw':" + str(input[2]) + "]"   + ", 'I_input':" + "['surge':" + str(input_integrated[0]) +  ", 'sway':" + str(input_integrated[1]) +  ", 'yaw':" + str(input_integrated[2]) + "]" + ", 'D_input':" + "['surge':" + str(input_derivative[0]) +  ", 'sway':" + str(input_derivative[1]) +  ", 'yaw':" + str(input_derivative[2]) + "]"

          epoch_time_previous=epoch_time

          # control_command contains the actual control signal
          # generate lcm
          msg=pid()
          msg.timestamp=epoch_time
          msg.input=input
          msg.input_derivative=input_derivative
          msg.input_integrated=input_integrated
          msg.control_command=control_command
          msg.pid_string=pid_string

          # publish lcm
          lc.publish("pid", msg.encode())
          #write to logs
          with open(control_path + control_filename ,'a') as fileout:
            fileout.write(pid_string +"\n")
            fileout.close()

        input_prev = input
