# parse_sensehat

# Scripts to parse sensehat for raspberry pi. Uses lcm and stores original strings

# https://pythonhosted.org/sense-hat/api/#imu-sensor

# Author: Blair Thornton
# Date: 18/09/2017

import os, io
import lcm
import time
import math
import threading
import copy
from picamera import PiCamera
from picamera.array import PiRGBArray
from sense_hat import SenseHat
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import numpy as np
#from PIL import Image
import lcm_handler.config as lcm_global
from datetime import datetime
from image.image import image
from image.visual_track import visual_track

output_prefix = "[parse_camera_module_v2] "

class parse_camera_module_v2(threading.Thread):
#    def __init__(self,camera_file,camera_filename,behaviour_type,camera,horizontal_resolution, vertical_resolution,store_image_interval,target_lower,target_upper,success,searching,track_frames):
    def __init__(self,camera_file,camera_filename,behaviour_type,horizontal_resolution, vertical_resolution, horizontal_field_of_view, vertical_field_of_view, horizontal_angular_offset, vertical_angular_offset,store_image_interval,target_lower,target_upper,success,searching,track_frames,minimum_radius_ratio):
        threading.Thread.__init__(self)
        
        lc = lcm.LCM()
        toggle_gain = 0
        #initialise camera
        camera=PiCamera()
        camera.resolution = (horizontal_resolution, vertical_resolution)        
        camera.iso = 800
        # Wait for the automatic gain control to settle
        time.sleep(3)
        # Now fix the values
        camera.framerate = 3
        camera.shutter_speed = camera.exposure_speed #us
        camera.exposure_mode = 'off'
        #camera.drc_strength = 'high'#dynamic range
        #g = camera.awb_gains
        time.sleep(5)
        # print g
        #camera.awb_mode = 'off'
        #camera.led = False
        #camera.awb_gains = g
        camera.awb_mode = 'auto'
        camera.led = False
        
        rawCapture=PiRGBArray(camera, size=(horizontal_resolution, vertical_resolution))
        
        image_count=1 #image number counter
        storage_count = 1 #counter to store images
        sense = SenseHat() #sense hat: need to run through logic as this device may not be present
        
        #enter infinite loop, stopped by ._Thread_stop()
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
          
          #get a frame and store timestamp
          raw_image= frame.array
          
          epoch_time = round(time.time()*1000)/1000
          datetime_string = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
          # show the frame
          #cv2.imshow("Frame",image)
          
          #clear the stream for nex image
          rawCapture.truncate(0)
        
          x_c=-9999
          y_c=-9999
          
          if behaviour_type == "visual_tracking":
            pts=deque(maxlen=track_frames)
        
        
            # copy iomage for analysis
            processed_image=frame.array.copy()
            # extract colour and get rid of small noise using YCrCb space
            hsv = cv2.cvtColor(processed_image, cv2.COLOR_BGR2HSV)
#            YCrCb = cv2.cvtColor(processed_image, cv2.COLOR_BGR2YCrCb)
#            mask = cv2.inRange(YCrCb, target_lower, target_upper)
           
            # this happens for red values where the hue wraps at 180
            if target_lower[0] > target_upper[0]:
                mask1 = cv2.inRange(hsv, (0, target_lower[1], target_lower[2]), (target_upper[0], target_upper[1], target_upper[2]))
                mask2 = cv2.inRange(hsv, (target_lower[0], target_lower[1], target_lower[2]), (180, target_upper[1], target_upper[2]))
                mask = cv2.add(mask1,mask2)
            else:
                mask = cv2.inRange(hsv, target_lower, target_upper)
            
            mask = cv2.erode(mask, None, iterations=1)
            mask = cv2.dilate(mask, None, iterations=1)
            #find contours in the mask and initialise the current x,y centre of ball
            cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
            centre= None
            #cv2.imshow("Frame",mask)
            
            #only proceed if at least one of the counters was found
            if len(cnts)>0:
              #find the largest contour in the mask and use it to
              #find the minimum enclosing circle and centroid
              c = max(cnts, key=cv2.contourArea)
              
              ((i,j), radius)= cv2.minEnclosingCircle(c)
              
              #print radius/horizontal_resolution
              
              if radius >= horizontal_resolution*minimum_radius_ratio:
                #calulate relative to image centre
                y_c=(horizontal_resolution/2-i)/(horizontal_resolution/2)
                x_c=(vertical_resolution/2-j)/(vertical_resolution/2)
                #print i,j
                M = cv2.moments(c)
                if M["m00"]==0 or M["m01"]==0 or M["m10"]==0:
                    sense.clear(searching[0],searching[1],searching[2])
                    print epoch_time
                    print M
                    
                else:
                    centre=(int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
                    # set LEDs to colour for success
                    sense.clear(success[0],success[1],success[2])
              
                    # draw the circle and centroid and update tracked points
                    cv2.circle(processed_image, (int(i), int(j)),int(radius),
                        (0, 255, 255), 2)
                    cv2.circle(processed_image, centre, 5, (0, 0, 255), -1)
        
              else:
                sense.clear(searching[0],searching[1],searching[2])
             
            else:
              sense.clear(searching[0],searching[1],searching[2])
              
            
            # update the points queue
            pts.appendleft(centre)

            # loop over the set of tracked points
            for i in xrange(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
		if pts[i - 1] is None or pts[i] is None:
			continue

		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
            
	    # show the frame to our screen
	    #cv2.imshow("Frame", processed)
            # 12/04/2018 Blair shifted visual angles and offsets to yaml and parse_camera_module_v2.py 
            if x_c != -9999 and y_c != -9999:

                if lcm_global.lcm_imu_orientation[0] > 180:
                    roll_angle = lcm_global.lcm_imu_orientation[0] - 360                
                else:
                    roll_angle = lcm_global.lcm_imu_orientation[0]
                
                if lcm_global.lcm_imu_orientation[1] > 180:
                    pitch_angle = lcm_global.lcm_imu_orientation[1] - 360    
                else:
                    pitch_angle = lcm_global.lcm_imu_orientation[1] 
            
            # signs intentionally different due to how camera reads image vertically vs horizontally w.r.t the imu pitch and roll
                x_relative_angle = (vertical_field_of_view/2 * x_c - (vertical_angular_offset + pitch_angle))/(vertical_field_of_view/2)
                y_relative_angle = (horizontal_field_of_view/2 * y_c + (horizontal_angular_offset + roll_angle))/(horizontal_field_of_view/2)
                #x_relative_angle = x_c
                #y_relative_angle = y_c
                
                print output_prefix + "Time:", format(epoch_time, '.1f') , "x_c:" , format(x_relative_angle,'.2f'), "y_c:" , format(y_relative_angle,'.2f')
                
            else:
                x_relative_angle = x_c
                y_relative_angle = y_c
                
            #read in image and resize, store count globally
            tracking_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'count':" + str(image_count) + ", 'x':" + str(x_c) + ", 'y':" + str(y_c) + ", 'x_relative_angle':" + str(x_relative_angle) + ", 'y_relative_angle':" + str(y_relative_angle) 
                        
            msg=visual_track()
            msg.timestamp=epoch_time
            msg.count=image_count
            msg.x=x_relative_angle
            msg.y=y_relative_angle
            msg.tracking_string=tracking_string
            #print x_c, y_c
            # publish lcm
            lc.publish("visual_track", msg.encode())
            
            #write to logs
            with open(camera_file + "visual_track.txt",'a') as fileout:
                fileout.write(tracking_string +"\n")
            fileout.close()
            
          if storage_count == store_image_interval:
            store_flag=1
            storage_count=0
          else:
            store_flag=0
            
      
          if store_flag == 1:    
            
            # generate file name to store the image
            image_filename=camera_filename.split('.')            
            
            filename = datetime_string[0:len(datetime_string)-3] + '_' + image_filename[0] + '.jpg'            
            image_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'image':" + filename +  ", 'count':" + str(image_count) 
            #camera.capture(camera_file + filename)#
            cv2.imwrite(camera_file + filename,raw_image)
            if behaviour_type == "visual_tracking":
              filename = datetime_string[0:len(datetime_string)-3] + '_' + image_filename[0] + '_proc.jpg'
              cv2.imwrite(camera_file + filename, processed_image)
              
#            
#            camera.capture(camera_file + 'stream.jpg')
            # generate lcm message
            msg=image()
            msg.timestamp=epoch_time
            msg.count=image_count
            msg.filename=filename
            msg.image_string=image_string          
     
            # publish lcm
            lc.publish("image", msg.encode())
            
            #write to logs
            with open(camera_file + camera_filename,'a') as fileout:
                fileout.write(image_string +"\n")
            fileout.close()
            
          image_count=image_count+1
          storage_count=storage_count+1
          cv2.destroyAllWindows()