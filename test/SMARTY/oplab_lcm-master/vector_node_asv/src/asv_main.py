# asv_main

# main script for controlling vector_node_asv

# Author: Blair Thornton
# Date: 18/09/2017

"""main control of vector_node_asv
        
        python asv_main.py start -c <config_dir> -o <output_dir>
        python asv_main.py stop
        
        
   if no <config_dir> is provided, the default path will be "~/configuration"
   if no <output_dir> is provided, the default path will be "~/output"

   Mission name and behaviour read from "~/configuration/mission.yaml" file for default paths
   Vehicle configuration read from "~/configuration/vehicle.yaml" file for default paths

   
   Requires PyYAML which can be downloaded from http://pyyaml.org/wiki/PyYAML
   Go to the folder where the downloaded file (at time of writting)
        
            http://pyyaml.org/download/pyyaml/PyYAML-3.12.tar.gz

   in the extracted folder, execute the following terminal commands 

        $ sudo python setup.py install
        $ python setup.py test
                     
     """

# Recommended Python Style Guide: https://www.python.org/dev/peps/pep-0008/

# Import librarys
import sys, os, time
import yaml, lcm, math
import signal, fcntl
import picamera
import numpy as np
import pygame

from threading import Thread
from sense_hat import SenseHat
from array import array
from time import sleep
from os.path import expanduser

## WARNING JUST FOR TESTS
from lib_control import maestro

from lib_sensors.parse_sensehat import parse_sensehat
from lib_sensors.parse_strain_gauge import parse_strain_gauge
from lib_sensors.parse_camera_module_v2 import parse_camera_module_v2
from lib_control.pwm_thruster import pwm_thruster
from lib_control.pid_control import pid_control
from lib_control.control.pid import pid
from lib_navigation.visual_tracking import visual_tracking
from lib_navigation.qualisys_tracking import qualisys_tracking

#WARNING: Added Joystick support
from lib_sensors.joystick_thread import joystick_thread

#global variables
function_handler = 0
output_prefix = "[asv_main] "

def run_once():
    global function_handler
    function_handler = open(os.path.realpath(__file__),'r')
    try:
        fcntl.flock(function_handler,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except:
        home=expanduser('~')
        command=sys.argv[1]
    
        # mode called by start_mission.sh
        if command == 'start':
            print(output_prefix + "Abort: asv_main.py is already runnning")
            sys.exit(0)

        # mode called by stop_mission.sh
        if command == 'stop':
            global start_flag
            start_flag = 0
            # updates and writes start_flag
            with open(home + '/check/' + 'start_flag.txt','w') as fileout:
                fcntl.flock(fileout,fcntl.LOCK_EX|fcntl.LOCK_NB)
                fileout.write(str(start_flag))
                fcntl.flock(fileout,fcntl.LOCK_UN)
            fileout.close()

def signal_handler(signal,frame):
    global start_flag
    start_flag = 0
    with open(home + '/check/' + 'start_flag.txt','w') as fileout:
        fcntl.flock(fileout,fcntl.LOCK_EX|fcntl.LOCK_NB)
        fileout.write(str(start_flag))
        fcntl.flock(fileout,fcntl.LOCK_UN)
    fileout.close()

run_once()
signal.signal(signal.SIGINT, signal_handler)

def mission_configuration(config_path,output_path):
    
    global start_flag
    # initialise vehicle flags
    platform_flag = 0
    log_format_flag = 0
    imu_flag = 0
    environment_flag = 0
    led_flag = 0
    camera_flag = 0
    store_image_interval = 0 # store 1 in how many, where 0 is don't store
    
    nmea_flag = 0
    control_flag = 0
    joystick_flag = 0
    actuator_flag= 0 # 1 is loaded
    thruster_number = 0
    signal_flag = 0 # 1 is pwm
    sensehat_flag = 0
    camera_settings = 0 #first time, need to set parameters
    model_flag = 0 # if 1 model has been loaded
    strain_flag = 0 # if 1 strain gauges are being used
    
    # initialise mission flags    
    mission_flag = 0
    feature_flag = 0 # visual feature to track in visual processing
     
    # declare lists
    actuator_type=[]
    actuator_orientation=[]
    actuator_clockwise_direction=[]
    actuator_surge_offset=[]
    actuator_sway_offset=[]
    actuator_forward_thrust_max=[]
    actuator_reverse_thrust_max=[]
    actuator_signal=[]
    pwm_neutral=[]
    pwm_forward_max=[]
    pwm_reverse_max=[]
    pwm_forward_min=[]
    pwm_reverse_min=[]
    pwm_acceleration=[]
    white=[]
    red=[] # convention [r,b,g]
    green=[]
    yellow=[]
    blue=[]
    success=[] # colour to use for LEDs when visual tracking is successful
    surge_gain=[] # convention [kp,ki,kd]
    sway_gain=[]
    yaw_gain=[]
    control_limits=[] # convention [Tx,Ty,Mz] i.e. surge (N), sway (N), yaw (Nm)
    
    
    actuator_surge_contribution=[]
    actuator_sway_contribution=[]
    actuator_yaw_contribution=[]
    actuator_label=[]
    waypoint_label=[]
    thrust_command=[]
    pwm_command=[]
    waypoint_north=[]
    waypoint_east=[]
    waypoint_heading=[]
    waypoint_timeout=[]
    
    #initialise control parameters
      
    #input=[4,-0,0]

    print('\n' + output_prefix + 'Loading mission.yaml')    
    mission = config_path + '/mission.yaml'
    with open(mission,'r') as stream:
        load_mission = yaml.load(stream)   
        #for i in range(0,len(load_mission)): 
        if 'mission' in load_mission:
            mission_flag=1
            mission_name = load_mission['mission']
            print output_prefix + "<mission>:", mission_name
        if 'behaviour' in load_mission:
            behaviour_type = load_mission['behaviour']
            print output_prefix + "<behaviour_type>:", behaviour_type
        if 'feature' in load_mission:
            feature_flag=1
            feature_type = load_mission['feature']
        if 'waypoints' in load_mission:
            waypoint_type = load_mission['waypoints']['type']
            waypoint_number = load_mission['waypoints']['number']
          
            for i in range(waypoint_number):           
              waypoint_label.append('waypoint_' + str(i))
              waypoint_north.append(load_mission['waypoints'][waypoint_label[i]]['north'])
              waypoint_east.append(load_mission['waypoints'][waypoint_label[i]]['east'])
              waypoint_heading.append(load_mission['waypoints'][waypoint_label[i]]['heading'])
              waypoint_timeout.append(load_mission['waypoints'][waypoint_label[i]]['timeout'])

        if 'colour' in load_mission:
            target_H_minimum = load_mission['colour']['minimum_HSV']['H']
            target_S_minimum = load_mission['colour']['minimum_HSV']['S']
            target_V_minimum = load_mission['colour']['minimum_HSV']['V']
            
            target_H_maximum = load_mission['colour']['maximum_HSV']['H']
            target_S_maximum = load_mission['colour']['maximum_HSV']['S']
            target_V_maximum = load_mission['colour']['maximum_HSV']['V']
             
            target_track_frames= load_mission['colour']['track_frames']#number of frames to remebers
            minimum_radius_ratio= load_mission['colour']['minimum_radius_ratio']#number of frames to remebers
            
    print('\n' + output_prefix + 'Loading vehicle.yaml')    
    vehicle = config_path + '/vehicle.yaml'
    with open(vehicle,'r') as stream:
        load_vehicle = yaml.load(stream)
        if 'platform' in load_vehicle:
            platform_flag=1
            platform_name = load_vehicle['platform']    
        if 'log' in load_vehicle:
            log_format_flag=1
            log_format = load_vehicle['log']['format']
        if 'sensors' in load_vehicle:
            if 'imu' in load_vehicle['sensors']:
              imu_flag=1
              imu_device=load_vehicle['sensors']['imu']['device']
              if imu_device== 'sensehat' and sensehat_flag ==0:
                sense = SenseHat()
                sensehat_flag=1
                sense.set_imu_config(True, True, True)
                
              imu_filepath= output_path + '/' + load_vehicle['sensors']['imu']['filepath']
              imu_filename= load_vehicle['sensors']['imu']['filename']
              
            if 'environment' in load_vehicle['sensors']:
              environment_flag=1
              environment_device= load_vehicle['sensors']['environment']['device']
              if environment_device== 'sensehat' and sensehat_flag ==0:
                sense = SenseHat()
                sensehat_flag=1
                
              environment_filepath= output_path + '/' + load_vehicle['sensors']['environment']['filepath']
              environment_filename= load_vehicle['sensors']['environment']['filename']
              
            if 'camera' in load_vehicle['sensors']:
              camera_flag=1
              camera_device= load_vehicle['sensors']['camera']['device']
              camera_filepath= output_path + '/' + load_vehicle['sensors']['camera']['filepath']
              camera_filename= load_vehicle['sensors']['camera']['filename']
              store_image_interval =  load_vehicle['sensors']['camera']['store']
              horizontal_resolution =  load_vehicle['sensors']['camera']['horizontal_resolution']
              horizontal_field_of_view =  load_vehicle['sensors']['camera']['horizontal_field_of_view']
              vertical_field_of_view =  load_vehicle['sensors']['camera']['vertical_field_of_view']
              horizontal_angular_offset =  load_vehicle['sensors']['camera']['horizontal_angular_offset']
              vertical_angular_offset =  load_vehicle['sensors']['camera']['vertical_angular_offset']
            if 'strain' in load_vehicle['sensors']:
              strain_flag=1
              strain_filepath= output_path + '/' + load_vehicle['sensors']['strain']['filepath']
              strain_filename= load_vehicle['sensors']['strain']['filename']

            if 'nmea' in load_vehicle['sensors']:
              nmea_flag=1
              nmea_filepath= output_path + '/' + load_vehicle['sensors']['nmea']['filepath']
              nmea_filename= load_vehicle['sensors']['nmea']['filename']

        if 'leds' in load_vehicle:
            led_flag=1
            led_device= load_vehicle['leds']['device']
            if led_device == 'sensehat':
              if sensehat_flag== 0:
                 sense = SenseHat()
              if 'white' in load_vehicle['leds']:
                white.append(load_vehicle['leds']['white']['red'])
                white.append(load_vehicle['leds']['white']['green'])
                white.append(load_vehicle['leds']['white']['blue'])
              if 'red' in load_vehicle['leds']:
                red.append(load_vehicle['leds']['red']['red'])
                red.append(load_vehicle['leds']['red']['green'])
                red.append(load_vehicle['leds']['red']['blue'])
              if 'green' in load_vehicle['leds']:
                green.append(load_vehicle['leds']['green']['red'])
                green.append(load_vehicle['leds']['green']['green'])
                green.append(load_vehicle['leds']['green']['blue'])
              if 'blue' in load_vehicle['leds']:
                blue.append(load_vehicle['leds']['blue']['red'])
                blue.append(load_vehicle['leds']['blue']['green'])
                blue.append(load_vehicle['leds']['blue']['blue'])
              if 'yellow' in load_vehicle['leds']:
                yellow.append(load_vehicle['leds']['yellow']['red'])
                yellow.append(load_vehicle['leds']['yellow']['green'])
                yellow.append(load_vehicle['leds']['yellow']['blue'])
                
        if 'joystick' in load_vehicle:
            joystick_flag=1
            joystick_filepath= output_path + '/' + load_vehicle['joystick']['filepath']
            joystick_filename= load_vehicle['joystick']['filename']

        if 'control' in load_vehicle:
            control_flag=1
            control_filepath= output_path + '/' + load_vehicle['control']['filepath']
            control_filename= load_vehicle['control']['filename']
            if load_vehicle['control']['method'] == 'PID':
              if 'surge' in load_vehicle['control']:
                surge_gain.append(load_vehicle['control']['surge']['kp'])
                surge_gain.append(load_vehicle['control']['surge']['ki'])
                surge_gain.append(load_vehicle['control']['surge']['kd'])
              if 'sway' in load_vehicle['control']:
                sway_gain.append(load_vehicle['control']['sway']['kp'])
                sway_gain.append(load_vehicle['control']['sway']['ki'])
                sway_gain.append(load_vehicle['control']['sway']['kd'])
              if 'yaw' in load_vehicle['control']:
                yaw_gain.append(load_vehicle['control']['yaw']['kp'])
                yaw_gain.append(load_vehicle['control']['yaw']['ki'])
                yaw_gain.append(load_vehicle['control']['yaw']['kd'])
              if 'limits' in load_vehicle['control']:
                control_limits.append(load_vehicle['control']['limits']['surge_thrust'])
                control_limits.append(load_vehicle['control']['limits']['sway_thrust'])
                control_limits.append(load_vehicle['control']['limits']['yaw_moment'])
                
        if 'model' in load_vehicle:
          model_flag=1
          mass = load_vehicle['model']['mass']         
          surge_added_mass = load_vehicle['model']['surge_added_mass']
          sway_added_mass = load_vehicle['model']['sway_added_mass']
          surge_drag_coefficient = load_vehicle['model']['surge_drag_coefficient']
          sway_drag_coefficient = load_vehicle['model']['sway_drag_coefficient']
          inertia = load_vehicle['model']['inertia']
          yaw_drag_coefficient = load_vehicle['model']['yaw_drag_coefficient']
        
        if 'actuators' in load_vehicle:
          actuator_flag=1
          actuator_number = load_vehicle['actuators']['number']
          actuator_filepath= output_path + '/' + load_vehicle['actuators']['filepath']
          actuator_filename= load_vehicle['actuators']['filename']
              
          for i in range(actuator_number):
            
            actuator_label.append('actuator_' + str(i))
            load_actuators = load_vehicle['actuators']
            print output_prefix + "Loading:", actuator_label[i], "as", load_actuators[actuator_label[i]]['type']
            
            if load_actuators[actuator_label[i]]['type'] == 'thruster':
              thruster_number=thruster_number+1   # add a new thruster to the list
              thrust_command.append(0)            # add 0 to the current command list (no movement)

              actuator_type.append(load_actuators[actuator_label[i]]['type'])
              actuator_orientation.append(load_actuators[actuator_label[i]]['orientation'])
              actuator_surge_offset.append(load_actuators[actuator_label[i]]['surge_offset'])
              actuator_sway_offset.append(load_actuators[actuator_label[i]]['sway_offset'])

              actuator_forward_thrust_max.append(load_actuators[actuator_label[i]]['forward_thrust_max'])
              actuator_reverse_thrust_max.append(load_actuators[actuator_label[i]]['reverse_thrust_max'])
              
              actuator_surge_contribution.append(np.cos(np.deg2rad(actuator_orientation[i])))
              actuator_sway_contribution.append(np.sin(np.deg2rad(actuator_orientation[i])))
              # Fix #15: Add np.sign(actuator_sway_offset[i]) sign correction factor, because the cosine function is even and doesn't distinguish for YAW contrib side
              actuator_yaw_contribution.append(np.sign(actuator_sway_offset[i])*math.sqrt(actuator_surge_offset[i]*actuator_surge_offset[i]+actuator_sway_offset[i]*actuator_sway_offset[i])*np.cos(np.deg2rad(actuator_orientation[i])-np.arctan(actuator_sway_offset[i]/actuator_surge_offset[i])))
              
              if load_actuators[actuator_label[i]]['forward_direction'] == 'clockwise':
                actuator_clockwise_direction.append(1)
              elif load_actuators[actuator_label[i]]['forward_direction'] == 'counter_clockwise':
                actuator_clockwise_direction.append(-1)
              else:
                print output_prefix + "ERROR: thruster forward_direction must be clockwise or counter_clockwise"
                return
              
            if 'pwm' in load_actuators[actuator_label[i]]:
              signal_flag=1
              actuator_signal.append('pwm')
              pwm_neutral.append(load_actuators[actuator_label[i]]['pwm']['neutral'])
              pwm_forward_max.append(load_actuators[actuator_label[i]]['pwm']['forward_max'])
              pwm_reverse_max.append(load_actuators[actuator_label[i]]['pwm']['reverse_max'])
              pwm_forward_min.append(load_actuators[actuator_label[i]]['pwm']['forward_min'])
              pwm_reverse_min.append(load_actuators[actuator_label[i]]['pwm']['reverse_min'])
              pwm_acceleration.append(load_actuators[actuator_label[i]]['pwm']['acceleration'])
              pwm_command.append(load_actuators[actuator_label[i]]['pwm']['neutral'])
        
    # generate output paths
    print output_prefix + "Generating output paths"
    if joystick_flag == 1:
                    
        if os.path.isdir(joystick_filepath) == 0:
            try:
                os.mkdir(joystick_filepath)
            except Exception as e:
                print output_prefix + "Warning:",e
        with open( joystick_filepath +'/' + joystick_filename,'w') as fileout:   
            fileout.close()


    if imu_flag == 1:
                   
        if os.path.isdir(imu_filepath) == 0:
            try:
                os.mkdir(imu_filepath)
            except Exception as e:
                print output_prefix + "Warning:",e
        with open(imu_filepath +'/' + imu_filename,'w') as fileout:   
            fileout.close()

    if environment_flag == 1:
                    
        if os.path.isdir(environment_filepath) == 0:
            try:
                os.mkdir(environment_filepath)
            except Exception as e:
                print output_prefix + "Warning:",e
        with open( environment_filepath +'/' + environment_filename,'w') as fileout:   
            fileout.close()
    if control_flag == 1:
                    
        if os.path.isdir(control_filepath) == 0:
            try:
                os.mkdir(control_filepath)
            except Exception as e:
                print output_prefix + "Warning:",e
        with open( control_filepath +'/' + control_filename,'w') as fileout:   
            fileout.close()
     
    if camera_flag == 1 and store_image_interval > 0: #only if images are to be stored
                    
        if os.path.isdir(camera_filepath) == 0:
            try:
                os.mkdir(camera_filepath)
            except Exception as e:
                print output_prefix + "Warning:",e
        with open( camera_filepath +'/' + camera_filename,'w') as fileout:
            fileout.close()
    
    if actuator_flag == 1:
                   
        if os.path.isdir(actuator_filepath) == 0:
            try:
                os.mkdir(actuator_filepath)
            except Exception as e:
                print output_prefix + "Warning:",e
        with open( actuator_filepath +'/' + actuator_filename,'w') as fileout:   
            fileout.close()
 
    if strain_flag == 1:
                   
        if os.path.isdir(strain_filepath) == 0:
            try:
                os.mkdir(strain_filepath)
            except Exception as e:
                print output_prefix + "Warning:",e
        with open( strain_filepath +'/' + strain_filename,'w') as fileout:   
            fileout.close()
 
    if nmea_flag == 1:
                   
        if os.path.isdir(nmea_filepath) == 0:
            try:
                os.mkdir(nmea_filepath)
            except Exception as e:
                print "Warning:",e
        with open( nmea_filepath +'/' + nmea_filename,'w') as fileout:   
            fileout.close()
                    
    # copy configurations to output
    print output_prefix + "Stashing configurations in output path"
    command_line = 'cp -r ' + config_path + ' ' + output_path +'\t'
    os.system(command_line)
    
    print "\n" + output_prefix + "Entering main loop"
    
    #check for change is flag condition
    try:
         with open(home + '/check/' + 'start_flag.txt','r') as fileout:
            start_flag=int(fileout.read())
         fileout.close()
    except:
        start_flag =1
            
    if start_flag == 1:
                
        if led_flag==1:           
            if led_device == 'sensehat':
                searching=white
                if behaviour_type == "visual_tracking":
                    success=red
                    searching=green
                    
                if behaviour_type == "check_routine":
                    searching=white

                if behaviour_type == "visual_odometry":
                    success=red
                    searching=blue

                if behaviour_type == "remote_control":
                    success = green
                    searching = yellow

                if behaviour_type == "qualisys_tracking":
                    success=blue
                    searching=green
                
                sense.clear(searching[0],searching[1],searching[2])
                led_flag=2 # only set when a change is needed
                    
        if sensehat_flag==1:
            if imu_flag == 1:
                imu_file = imu_filepath +'/' + imu_filename
            else:
                imu_file = 'null'
            if environment_flag == 1:
                environment_file = environment_filepath +'/' + environment_filename
            else:
                environment_file = 'null'                       
            
            sensehat_thread= Thread(target=parse_sensehat, args=(imu_flag,imu_file,environment_flag,environment_file))            
            sensehat_thread.start()
            print output_prefix + "Initiate: Sensor thread"
            
        if camera_flag==1:
            camera_file  = 'null'
            if store_image_interval > 0:
                camera_file = camera_filepath
            if camera_device == 'camera_module_v2':
                
                vertical_resolution=horizontal_resolution*3/4
                
                target_lower=(target_H_minimum, target_S_minimum, target_V_minimum)
                target_upper=(target_H_maximum ,target_S_maximum, target_V_maximum)          
                camera_thread= Thread(target=parse_camera_module_v2,args=(camera_file,camera_filename,behaviour_type,horizontal_resolution, vertical_resolution, horizontal_field_of_view, vertical_field_of_view, horizontal_angular_offset, vertical_angular_offset, store_image_interval,target_lower, target_upper,success,searching, target_track_frames,minimum_radius_ratio))
                camera_thread.start()
                print output_prefix + "Initiate: Camera thread"
                # wait for cameras to initialise

        if strain_flag==1:
            strain_file = strain_filepath +'/' + strain_filename
            strain_thread= Thread(target=parse_strain_gauge, args=(strain_flag,strain_file))            
            strain_thread.start()
            print output_prefix + "Initiate: Strain thread"
            
        if nmea_flag==1:
            nmea_file = nmea_filepath +'/' + nmea_filename
            print output_prefix + "Initiate: NMEA thread"
        #insert navigation model here
        
        # Configuration defined: platform operation mode
        if behaviour_type == 'remote_control':

            sense.show_message ("Remote mode", 0.04, yellow, blue)
            time.sleep(1)

            print output_prefix + "Remote control operation mode.\t Searching for joysticks..."
            pygame.joystick.init()
            num_joystick = pygame.joystick.get_count()
            pygame.quit()
            if (num_joystick == 0):
              print output_prefix + "No joystick was detected!\n\t'joystick' thread won't be launched"
            else:
              print output_prefix + "Total detected devices: " + str(num_joystick)
              print output_prefix + "Launching 'joystick' thread..."
            # Start thread for remote control mode
              tracking_thread= Thread(target=joystick_thread,args=(joystick_filepath, joystick_filename,0))
#              tracking_thread= Thread(target=visual_tracking,args=(control_filepath,'waypoints.txt'))
              tracking_thread.start()

        if behaviour_type == 'visual_tracking':
            
            sense.show_message ("Visual tracking", 0.04, yellow, blue)
            time.sleep(1)

            print output_prefix + "Launching: Visual tracking thread"
            tracking_thread= Thread(target=visual_tracking,args=(control_filepath,'waypoints.txt',waypoint_number,waypoint_label,waypoint_north,waypoint_east, waypoint_heading,waypoint_timeout))
            tracking_thread.start()
                
        if behaviour_type == 'qualisys_tracking':
            
            sense.show_message ("Qualisys tracking", 0.04, yellow, blue)
            time.sleep(1)

            print output_prefix + "Launching: Qualisys GPS tracking thread"
            tracking_thread= Thread(target=qualisys_tracking,args=(control_filepath,'waypoints.txt',waypoint_number,waypoint_label,waypoint_north,waypoint_east, waypoint_heading,waypoint_timeout,nmea_file))
            tracking_thread.start()

        print output_prefix + "Sleep for 10 seconds..."
        time.sleep(10)                  
                
        #temporary just for lab        
        if control_flag==1:                                                       
            print output_prefix + "Launching: pid_control thread"            
            control_thread= Thread(target=pid_control,args=(control_filepath,control_filename,surge_gain,sway_gain,yaw_gain,control_limits,behaviour_type))
            control_thread.start()
                                       
        if actuator_flag==1:               
            actuator_file = actuator_filepath
                    
            if 'thruster' in actuator_type:                
              print output_prefix + "Launching: thruster thread"            
              thruster_thread = Thread(target=pwm_thruster,args=(actuator_filepath,actuator_filename,actuator_number,actuator_label,thruster_number,actuator_type,actuator_clockwise_direction,actuator_surge_contribution,actuator_sway_contribution,actuator_yaw_contribution,actuator_forward_thrust_max,actuator_reverse_thrust_max,signal_flag,actuator_signal,pwm_neutral,pwm_forward_max,pwm_reverse_max,pwm_forward_min,pwm_reverse_min,pwm_acceleration,thrust_command,pwm_command,start_flag))        
              thruster_thread.start()
    
        #insert motion model here
                            
    print "\n" + output_prefix + "To stop type: ./stop_mission.sh"              
            #if behaviour_type == "check_routine":
    
    while start_flag==1:
        continue
    
    # it seems that the process wasn't started... so we enter the shutdown sequence
    if start_flag==0:
       
        print "\n" + output_prefix + "Entered end sequence"        
        if control_flag==1:
            control_thread._Thread_stop()
        # reset all control commands and wait for actuator loop to execute
            epoch_time = round(time.time()*1000)/1000
          
            # Send STOP sequence (0 values for all the fields)
            msg=pid()
            msg.timestamp=epoch_time
            msg.input=[0,0,0]
            msg.input_derivative=[0,0,0]
            msg.input_integrated=[0,0,0]
            msg.control_command=[0,0,0]
            pid_string = "'epoch_time':" + format(epoch_time,'.3f') +  ", 'surge_command':, 'sway_command':, 'yaw_command': 0, 'P_input': ['surge': 0, 'sway': 0, 'yaw': 0], 'I_input': ['surge': 0, 'sway': 0, 'yaw': 0], 'D_input': ['surge': 0, 'sway': 0, 'yaw': 0]" 
            msg.pid_string=pid_string
          
                # publish lcm
            lc.publish("pid", msg.encode())
            print output_prefix + "Waiting for actuators to stop"
            sleep(5)
            
#        fileout.close()

def usage():
# incorrect usage message
    print "     asv_main.py <command>"
    print "         <command> start <option> -c <config_dir>"
    print "                         <option> -o <output_dir>"
    print "         <command> stop"
    return -1  

if __name__ == '__main__':

    # initialise flags
    global start_flag
    start_flag=0
    config_flag=0
    output_flag=0
    # set default paths
    home=expanduser('~')
    config_path= home + '/configuration'
    output_path= home + '/output'
    

    # read command
    if (int((len(sys.argv)))) < 2:
        print output_prefix + "Error: not enough arguments"
        usage()
    else:   
        command=sys.argv[1]
        
        if command == "start":
            start_flag=1
            print "\n" + output_prefix + "Starting ASV_MAIN"

            with open(home + '/check/' + 'start_flag.txt','w') as fileout:
                fcntl.flock(fileout,fcntl.LOCK_EX|fcntl.LOCK_NB)
                fileout.write(str(start_flag))
                fcntl.flock(fileout,fcntl.LOCK_UN)
            fileout.close()
                       
            
            if (int((len(sys.argv)))) > 3:
                if sys.argv[2] == "-c":
                    config_path = sys.argv[3]
                    config_flag=1
                if sys.argv[2] == "-o":
                    output_path = sys.argv[3]
                    output_flag=1
            if (int((len(sys.argv)))) == 6:
                if sys.argv[4] == "-c":
                    config_path = sys.argv[5]
                    config_flag=1
                if sys.argv[4] == "-o":
                    output_path = sys.argv[5]
                    output_flag=1

            if config_flag == 1:
                print output_prefix + "<config_path>:", config_path
            else:
                print output_prefix + "<config_path> not set, using default:", config_path

            if output_flag == 1:
                print output_prefix + "<output_path>:", output_path
            else:
                print output_prefix + "<output_path> not set, using default:", output_path
            
            # enter the mission loop
            sys.exit(mission_configuration(config_path,output_path))                 
            
        elif command == "stop":
            if start_flag == 1:
                print output_prefix + "Stopping asv_main"
            
            start_flag=0
            with open(home + '/check/' + 'start_flag.txt','w') as fileout:
                fcntl.flock(fileout,fcntl.LOCK_EX|fcntl.LOCK_NB)
                fileout.write(str(start_flag))
                fcntl.flock(fileout,fcntl.LOCK_UN)
            fileout.close()
            
        else:
            usage()