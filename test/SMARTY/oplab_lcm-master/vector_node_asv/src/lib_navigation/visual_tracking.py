# parse_sensehat

# Scripts to parse sensehat for raspberry pi. Uses lcm and stores original strings

# https://pythonhosted.org/sense-hat/api/#imu-sensor

# Author: Blair Thornton
# Date: 18/09/2017

import os,sys
import lcm
import threading
import time
import math
import numpy as np
import navigation.tracking_nav as tracking_nav
sys.path.append("..")
import lcm_handler.config as lcm_global
from lcm_handler.lcm_handler import lcm_handler

# output_prefix = '[visual_tracking]'

class visual_tracking(threading.Thread):
    def __init__(self,waypoint_file,waypoint_filename,waypoint_number,waypoint_label,waypoint_north,waypoint_east, waypoint_heading,waypoint_timeout):
    
        threading.Thread.__init__(self)    
        lc = lcm.LCM()
        
        epoch_time_previous = round(time.time()*1000)/1000
        i=0
        #enter infinite loop, stopped by ._Thread_stop()
        while True:
          while i <= waypoint_number:
             #get image for x and y
             epoch_time = round(time.time()*1000)/1000 
             elapse = epoch_time - epoch_time_previous
             
             #tol = waypoint_tolerance[i] 
             #x = lcm_global.lcm_pid_angle[0]
             #y = lcm_global.lcm_pid_angle[1]
             #if x - tol <  waypoint_east[i] < x + tol and  y - tol <  waypoint_north[i] < y + tol:
             #  i=i+1
             #  epoch_time_previous=epoch_time
               
             #  waypoint_string = "'epoch_time':" + format(epoch_time,'.3f') +  ", 'waypoint_label':" + waypoint_label[i] + ", 'elapse':" + str(elapse) +", 'waypoint':[" + str(waypoint_north[i]) + "," + str(waypoint_east[i]) + "," + str(waypoint_heading[i]) + "], 'timeout':" + str(waypoint_timeout[i]) 
             #  epoch_time_previous=epoch_time
             
             if elapse > waypoint_timeout[i]:
               i=i+1
               epoch_time_previous=epoch_time
               
             waypoint_string = "'epoch_time':" + format(epoch_time,'.3f') +  ", 'waypoint_label':" + waypoint_label[i] + ", 'elapse':" + str(elapse) +", 'waypoint':[" + str(waypoint_north[i]) + "," + str(waypoint_east[i]) + "," + str(waypoint_heading[i]) + "], 'timeout':" + str(waypoint_timeout[i]) 
             epoch_time_previous=epoch_time
        
          # generate lcm
             msg=tracking_nav()
             msg.timestamp=epoch_time
             msg.label=waypoint_label[i]
             msg.elapse=elapse
             msg.waypoint=(waypoint_north[i],waypoint_east[i],waypoint_heading[i])
             msg.timeout=waypoint_timeout[i]
             msg.waypoint_string=waypoint_string
            
          # publish lcm
             lc.publish("tracking_nav", msg.encode())
          #write to logs
             with open(waypoint_file + waypoint_filename ,'a') as fileout:
               fileout.write(waypoint_string +"\n")
               fileout.close()
                 	