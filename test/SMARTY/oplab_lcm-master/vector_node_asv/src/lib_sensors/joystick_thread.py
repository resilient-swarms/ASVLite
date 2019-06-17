# joystick_thread

# Scripts acting as interface from PyGame - joystick, to the LCM channel 'joystick'
# Author: Jose Cappelletto
# Date: 29/01/2019

import os,sys
import lcm
import time
import math
import threading
import pygame
from sense_hat import SenseHat
from pygame.locals import *

# Import LCM generated message structure
from joystick.joystick import joystick

output_prefix = "[joystick_thread] "

class joystick_thread(threading.Thread):
    def __init__(self, joystick_path, joystick_filename, joystick_id):

        threading.Thread.__init__(self)

        print output_prefix + "Thread started"
        # Start PyGame
        pygame.init()
        # Start the Joystick module
        pygame.joystick.init()
        # if pygame.joystick.get_init():
        #     print output_prefix + "Joystick PyGame module loaded"
        # Retrieve the Joystick object for the specified Joystick (ID)
        my_joystick = pygame.joystick.Joystick(joystick_id)
        # Start that joystick
        my_joystick.init()
        # if my_joystick.get_init():
        #     print output_prefix + "Joystick " + str (joystick_id) + " started OK..."

        num_axes = my_joystick.get_numaxes()

        # There is an issue when running any pygame module through and ssh tunnel without X forwarding 
        # WARNING
        # pygame.display.init()
        # if pygame.display.get_init():
        #    print output_prefix + "PyGame Display module started OK!!!!!!!!!!!!!!"

        # Clear any previously queued event. We start with an empty list
        pygame.event.clear()

        # Initialise and clear LED matrix
        sense = SenseHat()
        sense.clear()

        axis_value = [None] * num_axes 
        lc = lcm.LCM()
        pixel_x = pixel_y = 0
#        print output_prefix + "PyGame configuration done..."

        #enter infinite loop, stopped by ._Thread_stop()

        while True:
          # check if any event in queue
          time.sleep(0.2)
          # we could peek() for any existing event, for the specific type we are looking for
          for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
            	# we could use BUTTON events to switch modes (even trigger kill/stop actions)
                print output_prefix + "Button pressed"
            # check if any AXIS has changed
            if event.type == pygame.JOYAXISMOTION:
                for i in range( num_axes ):
                    axis = my_joystick.get_axis(i)
                    # print (output_prefix + "Axis {} value: {:>6.3f}".format(i, axis) )
                    axis_value[i] = axis

                # ( Issue #13 ) Use SenseHat as visual output for registered axis values (X-Y for Sway-Surge)
                # Direct map of (-1.0,+ 1.0) to pixel coordinates 0-7
                # Ghost effect of previous coordinate
                sense.set_pixel(pixel_x, pixel_y, 10, 10, 10)

                pixel_y = int(( axis_value[0] + 0.99)*4.0) 
                pixel_x = int((-axis_value[1] + 0.99)*4.0) 
                green_channel = int((axis_value[2]+1.0)*127)	# We change the pixel colour according to the YAW reference value (CCW: RED, CW: GREEN)

                sense.set_pixel(pixel_x, pixel_y, 255 - green_channel, green_channel, 100)
                print output_prefix + "(X,Y,C) = " + str(pixel_x) + "," + str(pixel_y) + ", " + str(green_channel)
                # time.sleep(0.2)
                #sense.set_pixel()

                epoch_time = round(time.time()*1000)/1000	#retrieve system time as timestamp
                joystick_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'joystick':" + str(axis_value)
                #print strain_string
                # generate lcm
                msg=joystick()
                msg.timestamp=epoch_time
                msg.axis_value = axis_value
                msg.joystick_string=joystick_string

                # publish lcm
                lc.publish("joystick", msg.encode())
          
                #write to logs
                with open(joystick_path + joystick_filename ,'a') as fileout:
		            fileout.write(joystick_string +"\n")
		            fileout.close()

        fileout.close()