import lcm
import sys

import config as lcm_global
sys.path.append("..")
from lib_sensors.sensehat.imu import imu
from lib_sensors.sensehat.env import env
from lib_sensors.image.image import image
from lib_sensors.joystick.joystick import joystick
from lib_sensors.image.visual_track import visual_track
from lib_navigation.navigation.tracking_nav import tracking_nav
from lib_control.thruster.pwm import pwm
from lib_control.control.pid import pid

# Decode lcm data according to the associated channel, using the "decode" fuction provided on each library
def lcm_handler(channel, data):
    
    if channel == "imu":
        msg = imu.decode(data)
        #global lcm_imu_orientation
        lcm_global.lcm_imu_orientation=msg.orientation
    elif channel == "env":
        msg = env.decode(data)
    elif channel == "image":
        msg = image.decode(data)
    elif channel == "tracking_nav":
        msg = tracking_nav.decode(data)
        #global lcm_tracking_waypoint
        lcm_global.lcm_tracking_waypoint=msg.waypoint         
    elif channel == "visual_track":
        msg = visual_track.decode(data)
        #global lcm_visual_xy
        lcm_global.lcm_visual_xy=[msg.x,msg.y]       
    elif channel == "pwm":
        msg = pwm.decode(data)
    elif channel == "pid":
        msg = pid.decode(data)
        # global lcm_control_command
        lcm_global.lcm_control_command=msg.control_command
    elif channel == "qualisys":
        msg = qualisys.decode(data)
        lcm_global.lcm_qualisys_pose=[msg.northings,msg.westings,msg.heading_degrees]
    elif channel == "joystick":
        msg = joystick.decode(data)
        # Transfer data to the global variable
        lcm_global.lcm_joystick_axis=msg.axis_value


        
