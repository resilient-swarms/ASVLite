# pwm_thruster

# Scripts to parse pwm_thruster data Uses lcm and stores original strings

# Author: Blair Thornton

# Date: 18/09/2017


import os, select 
import lcm, sys
import time
import math
import threading
import maestro
from thruster.pwm import pwm
from control.pid import pid

sys.path.append("..")
from lcm_handler.lcm_handler import lcm_handler
import lcm_handler.config as lcm_global

output_prefix = "[pwm_thruster] "

class pwm_thruster(threading.Thread):
    def __init__(self, actuator_filepath,actuator_filename,actuator_number,actuator_label,thruster_number,actuator_type,actuator_clockwise_direction,actuator_surge_contribution,actuator_sway_contribution,actuator_yaw_contribution,actuator_forward_thrust_max,actuator_reverse_thrust_max,signal_flag,actuator_signal,pwm_neutral,pwm_forward_max,pwm_reverse_max,pwm_forward_min,pwm_reverse_min,pwm_acceleration,thrust_command,pwm_command, start_flag):
        
        threading.Thread.__init__(self)     
        lc = lcm.LCM()
        
        #load maestro pwm controll
        servo = maestro.Controller()
        
        # to initialise send a stop script
        for i in range(len(pwm_command)):                    
            pwm_command[i] = int(pwm_neutral[i])
            servo.setTarget(i,int(round(pwm_command[i]*4)))                        
                        
        time.sleep(2)
        servo.stopScript()
        
        for i in range(len(pwm_command)):
            
            servo.setAccel(i,pwm_acceleration[i]) # set servos acceleration to 10 (10 to 255)
            if pwm_forward_max[i]>pwm_reverse_max[i]:
                servo.setRange(i,int(round(pwm_reverse_max[i]*4)),int(round(pwm_forward_max[i]*4)))
            else:
                servo.setRange(i,int(round(pwm_forward_max[i]*4)),int(round(pwm_reverse_max[i]*4)))
            
       
        #enter infinite loop, stopped by ._Thread_stop()
        while True:
            # If system is already running (has started)     
            if start_flag == 1:
            
                # WARNING: WE SHOULD USE ANOTHER CHANNEL
                # subscribe/unsubscribe pair to retrieve at least a single package from 'pid' channel
                subscription=lc.subscribe("pid", lcm_handler)
                ok=0
                while ok==0:
                        ok=lc.handle_timeout(100)
                
                lc.unsubscribe(subscription)
                
                control_command=lcm_global.lcm_control_command

                epoch_time = round(time.time()*1000)/1000

                # for each actuator...
                for i in range(actuator_number):
                      
                  if actuator_type[i] == 'thruster':
                         
                    #adds the contribution to surge,sway and yaw needed to generate a thrust command for each thruster
                    thrust_command[i] = control_command[0]/actuator_surge_contribution[i]
                    thrust_command[i] = thrust_command[i] + control_command[1]/(actuator_sway_contribution[i])
                    thrust_command[i] = thrust_command[i] + control_command[2]/(actuator_yaw_contribution[i])
                    
                    for i in range(len(thrust_command)):
                      #cap the max and min thrust of each actuator                  
                      if thrust_command[i] > actuator_forward_thrust_max[i]:
                        thrust_command[i] = actuator_forward_thrust_max[i]
                      if thrust_command[i] < actuator_reverse_thrust_max[i]:
                        thrust_command[i] = actuator_reverse_thrust_max[i]
                
                    # signal_flag is defined as function argument (when creating this thread by the parent)
                    if signal_flag==1:
                        # for each trust command (should be the same as n_thrusters)
                        for i in range(len(thrust_command)):
                           if actuator_signal[i] == 'pwm':
                               
                               # check if must be reversed according to the clockwise_direction flag
                               if thrust_command[i] == 0:                             
                                 pwm_command[i] = (pwm_neutral[i])                                                           
                               elif (thrust_command[i] > 0 and actuator_clockwise_direction[i] == 1) or (thrust_command[i] > 0 and actuator_clockwise_direction[i] == -1): 
                                 pwm_command[i]= int(round(thrust_command[i]*(pwm_forward_max[i]-pwm_forward_min[i])/actuator_forward_thrust_max[i]+pwm_forward_min[i]))
                               elif (thrust_command[i] < 0 and actuator_clockwise_direction[i] == 1) or (thrust_command[i] < 0 and actuator_clockwise_direction[i] == -1):
                                 pwm_command[i]= int(round(thrust_command[i]*(pwm_reverse_max[i]-pwm_reverse_min[i])/actuator_reverse_thrust_max[i]+pwm_reverse_min[i]))
                               
                               #cap the values
                               if (pwm_command[i] > pwm_forward_max[i]) and (actuator_clockwise_direction[i] == 1):
                                 pwm_command[i] = int(pwm_forward_max[i])
                               if (pwm_command[i] < pwm_reverse_max[i]) and (actuator_clockwise_direction[i] == 1):
                                 pwm_command[i] = int(pwm_reverse_max[i])
                               if (pwm_command[i] < pwm_forward_max[i]) and (actuator_clockwise_direction[i] == -1):
                                 pwm_command[i] = int(pwm_forward_max[i])
                               if (pwm_command[i] > pwm_reverse_max[i]) and (actuator_clockwise_direction[i] == -1):
                                 pwm_command[i] = int(pwm_reverse_max[i])
                                        
                # WARNING: here is the actual translation from the control output reference to the PWM in the maestro
                for i in range(len(pwm_command)):
                    #pwm_interval[i]
                    servo.setTarget(i,int(round(pwm_command[i]*4))) #the *4 factor is due to maestro.py using quarter-microseconds
            
            else: # Set to neutral all actuators
                epoch_time = round(time.time()*1000)/1000
                if actuator_signal[i] == 'pwm':
                    for i in range(len(pwm_command)):                    
                        pwm_command[i] = int(pwm_neutral[i])
                        servo.setTarget(i,int(round(pwm_command[i]*4)))                        
                        
                    time.sleep(2)
                    servo.stopScript()
                                               
            pwm_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'thruster_n':" + str(thruster_number) + ", 'label':" + str(actuator_label) + ", 'thrust_command':" + str(thrust_command) + ", 'pwm_command':" + str(pwm_command)       
        
            # generate lcm
            msg=pwm()
            msg.timestamp=epoch_time
            msg.number=thruster_number
            msg.label=actuator_label
            msg.thrust_command=thrust_command
            msg.pwm_command=pwm_command
            msg.pwm_string=pwm_string
            
            # print "PWM:", pwm_command
            # publish lcm
            # the data from the pwm signal is forwarded to the pwm channel, so it can be stored for analysis.
            # the actual pwm control is done previously with servo.setTarget
            lc.publish("pwm", msg.encode())
            #write to logs
            if actuator_filepath == 'null':
              continue
            else:
              with open(actuator_filepath + actuator_filename ,'a') as fileout:
                fileout.write(pwm_string +"\n")
                fileout.close()