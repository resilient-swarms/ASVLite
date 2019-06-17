# parse_sensehat

# Scripts to parse sensehat for raspberry pi. Uses lcm and stores original strings

# https://pythonhosted.org/sense-hat/api/#imu-sensor

# Author: Blair Thornton
# Date: 18/09/2017

import os
import lcm
import time
import math
import threading

from sense_hat import SenseHat
from sensehat.imu import imu
from sensehat.env import env

class parse_sensehat(threading.Thread):
    def __init__(self, imu_flag, imu_file, environment_flag, environment_file):
        threading.Thread.__init__(self)
        
        lc = lcm.LCM()
        
        #enter infinite loop, stopped by ._Thread_stop()
        while True:
          if imu_flag == 1:
            # read in data
            sense=SenseHat()
            epoch_time = round(time.time()*1000)/1000
            orientation = sense.get_orientation()
            magnetic_compass = sense.get_compass_raw()
            gyroscope = sense.get_gyroscope_raw()
            acceleration = sense.get_accelerometer_raw()

            orientation_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'orientation':" + str(orientation)
            magnetic_compass_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'magnetic_compass':" + str(magnetic_compass)
            gyroscope_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'gyroscope':" + str(gyroscope)
            acceleration_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'acceleration':" + str(acceleration)

            #print "Time:", format(epoch_time, '.1f') , orientation
            
            # generate lcm
            msg=imu()
            msg.timestamp=epoch_time
            msg.orientation=(orientation['roll'],orientation['pitch'],orientation['yaw'])
            msg.magnetic_compass=(magnetic_compass['x'],magnetic_compass['y'],magnetic_compass['z'])
            msg.gyroscope=(math.degrees(gyroscope['x']),math.degrees(gyroscope['y']),math.degrees(gyroscope['z']))
            msg.acceleration=(acceleration['x'],acceleration['y'],acceleration['z'])
            msg.orientation_string=orientation_string
            msg.magnetic_compass_string=magnetic_compass_string
            msg.gyroscope_string=gyroscope_string
            msg.acceleration_string=acceleration_string

            # publish lcm
            lc.publish("imu", msg.encode())
            
            #write to logs
            with open(imu_file ,'a') as fileout:
                fileout.write(orientation_string +"\n")
                fileout.write(magnetic_compass_string+"\n")
                fileout.write(gyroscope_string+"\n")
                fileout.write(acceleration_string+"\n")
            fileout.close()
                    
          if environment_flag == 1:
                
            sense=SenseHat()
            epoch_time = round(time.time()*1000)/1000
             
            # read in data
            humidity = sense.get_humidity()
            temperature = sense.get_temperature()
            pressure = sense.get_pressure()
                
            humidity_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'humidity':" + str(humidity)
            temperature_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'temperature':" + str(temperature)
            pressure_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'pressure':" + str(pressure)
                   	   
            # generate lcm
            msg=env()
            msg.timestamp=epoch_time
            msg.humidity=humidity
            msg.temperature=temperature
            msg.pressure=pressure
            msg.humidity_string=humidity_string
            msg.temperature_string=temperature_string
            msg.pressure_string=pressure_string
           
            # publish lcm
            lc.publish("env", msg.encode())
            
            #write to logs
            with open(environment_file ,'a') as fileout:
                fileout.write(humidity_string+"\n")
                fileout.write(temperature_string+"\n")
                fileout.write(pressure_string+"\n")
            fileout.close()
         	