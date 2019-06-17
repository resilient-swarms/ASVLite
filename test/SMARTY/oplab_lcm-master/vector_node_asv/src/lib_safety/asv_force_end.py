# asv_force_end

# main script for forcing vector_node_asv to end

# Author: Blair Thornton
# Date: 18/09/2017

"""main control of vector_node_asv
        
        python asv_force_end.py start -c <config_dir> -o <output_dir>
        
        
   if no <config_dir> is provided, the defaul path will be "~/configuration"
   if no <output_dir> is provided, the defaul path will be "~/output"

   Mission name and behaviour read from "~/configuration/mission.yaml" file for default paths
   Vehicle configuration read from "~/configuration/vehicle.yaml" file for default paths

   
   Requires PyYAML which can be downloaded from http://pyyaml.org/wiki/PyYAML
   Go to the folder where the downloaded file (at time of writting)
        
            http://pyyaml.org/download/pyyaml/PyYAML-3.12.tar.gz

   in the extracted folder, execute the following terminal commands 

        $ sudo python setup.py install
        $ python setup.py test
                     
     """

# Import librarys
import sys, os
import yaml, time
from threading import Thread
from os.path import expanduser
from sense_hat import SenseHat

sys.path.append(os.path.join(sys.path[0],'..'))

from lib_control.pwm_thruster import pwm_thruster

output_prefix = "[asv_force_end] "

def asv_force_end(config_path,output_path):
    actuator_label=[]
    thrust_command=[]
    actuator_type=[]
    actuator_surge_contribution=[]
    actuator_sway_contribution=[]
    actuator_yaw_contribution=[]
    actuator_clockwise_direction=[]
    actuator_signal=[]
    pwm_neutral=[]
    pwm_command=[]
    pwm_acceleration=[]
    pwm_forward_max=[]
    pwm_reverse_max=[]
    pwm_forward_min=[]
    pwm_reverse_min=[]
    actuator_forward_thrust_max=[]
    actuator_reverse_thrust_max=[]
    actuator_flag=0
    thruster_number=0
    start_flag=0 #inactive
    
    
    print output_prefix + 'Entered force end sequence\n'        
    print output_prefix + 'Loading vehicle.yaml'    
    vehicle = config_path + '/vehicle.yaml'
    with open(vehicle,'r') as stream:
        load_vehicle = yaml.load(stream)
        if 'sensors' in load_vehicle:
            if 'imu' in load_vehicle['sensors']:
              imu_flag=1
              imu_device=load_vehicle['sensors']['imu']['device']
              if imu_device== 'sensehat':
                sense = SenseHat()
                sensehat_flag=1
                sense.set_imu_config(True, True, True)
                
            if 'environment' in load_vehicle['sensors']:
              environment_flag=1
              environment_device= load_vehicle['sensors']['environment']['device']
              if environment_device== 'sensehat' and sensehat_flag ==0:
                sense = SenseHat()
                sensehat_flag=1
            
            if 'camera' in load_vehicle['sensors']:
              camera_flag=1
              camera_device= load_vehicle['sensors']['camera']['device']
              store_image_interval =  load_vehicle['sensors']['camera']['store']
              horizontal_resolution =  load_vehicle['sensors']['camera']['horizontal_resolution']
               
        if 'leds' in load_vehicle:
            led_flag=1
            led_device= load_vehicle['leds']['device']
            if led_device == 'sensehat':
              if sensehat_flag== 0:
                 sense = SenseHat()
              
        if 'actuators' in load_vehicle:
          actuator_flag=1
          actuator_number = load_vehicle['actuators']['number']
              
          for i in range(actuator_number):
            
            actuator_label.append('actuator_' + str(i))
            load_actuators = load_vehicle['actuators']
            print output_prefix + "loading:", actuator_label[i], "as", load_actuators[actuator_label[i]]['type']

            actuator_filepath= output_path + '/' + load_vehicle['actuators']['filepath']
            actuator_filename= load_vehicle['actuators']['filename']
            if load_actuators[actuator_label[i]]['type'] == 'thruster':
              thruster_number=thruster_number+1
              thrust_command.append(0)

              actuator_type.append(load_actuators[actuator_label[i]]['type'])
               
              actuator_surge_contribution.append(1)
              actuator_sway_contribution.append(1)
              actuator_yaw_contribution.append(1)
              
              actuator_forward_thrust_max.append(load_actuators[actuator_label[i]]['forward_thrust_max'])
              actuator_reverse_thrust_max.append(load_actuators[actuator_label[i]]['reverse_thrust_max'])
              
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
        
        print output_prefix + 'Stopping actuators and lights\n'
        if actuator_flag==1:               
            actuator_filepath = 'null'
                   
            if 'thruster' in actuator_type:                
                thruster_thread = Thread(target=pwm_thruster,args=(actuator_filepath,actuator_filename,actuator_number,actuator_label,thruster_number,actuator_type,actuator_clockwise_direction,actuator_surge_contribution,actuator_sway_contribution,actuator_yaw_contribution,actuator_forward_thrust_max,actuator_reverse_thrust_max,signal_flag,actuator_signal,pwm_neutral,pwm_forward_max,pwm_reverse_max,pwm_forward_min,pwm_reverse_min,pwm_acceleration,thrust_command,pwm_command,start_flag))        
                thruster_thread.start()
                time.sleep(3)
                print output_prefix + "thrusters stopped"
                  
        #if actuator_flag==1:                               
        #    if 'thruster' in actuator_type:                
        #        thruster_thread._Thread_stop()
                
        if led_flag==1:
            sense.clear()
            print output_prefix + "lights out"
    
        print output_prefix + "End sequence complete. \n\nIt is safe to handle the vehicle"
      
        return -1

def usage():
# incorrect usage message
    print "     asv_force_endm.py <option1> <option2>"
    print "          -c <config_dir> "
    print "          -o <output_dir> "
    return -1

if __name__ == '__main__':

    home=expanduser('~')
    config_path= home + '/configuration'

    if (int((len(sys.argv)))) > 3:
        if sys.argv[1] == "-c":
            config_path = sys.argv[2]
                    
        if sys.argv[1] == "-o":
            output_path = sys.argv[2]
                    
        if (int((len(sys.argv)))) == 5:
            if sys.argv[3] == "-c":
                config_path = sys.argv[4]
                    
            if sys.argv[3] == "-o":
                output_path = sys.argv[4]
                    
        else:
            usage()
    else:
        usage()
    
    sys.exit(asv_force_end(config_path,output_path))                   