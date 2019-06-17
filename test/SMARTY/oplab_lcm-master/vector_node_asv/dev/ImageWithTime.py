import os
import cv2
import numpy as np
from picamera import PiCamera
import time
from cv2 import *
from datetime import datetime
rescount = 0 #A counter for the resolution
KeypointCount = []
photocount = 0
timestamplist = []
iteratex = 0
timestamp = []
foldertime = datetime.utcnow().strftime("%Y_%m_%d_%H_%M_%S") #Create new folder to store for each run through
BigFolder = '/home/pi/Tests/'+str(foldertime)+'/'
cam = PiCamera()
reslist = [192,320,640,1280,1920] #Resolutions for camera
if not os.path.exists(BigFolder): #Create folders
    os.makedirs(BigFolder)
for res in reslist: #Each resolution
    s1 = time.time() #Used for seeing how long it takes
    #m1 = int(datetime.utcnow().strftime("%M"))
    if not os.path.exists(BigFolder+str(reslist[rescount])): #Create folder to store this resolutions data
        os.makedirs(BigFolder+str(reslist[rescount]))
    filename = BigFolder+str(reslist[rescount])+'/points_detected'+'.txt' #Text file for the points detected in each photo
    with open(filename,'w+') as file:
        filename2 = BigFolder+str(reslist[iteratex])+'/frames_taken.txt' #Text file for the time each photo is taken, to see frames per second
        with open(filename2,'w+') as f:
            stop = time.time() +10 #Setting a 10s time limit for when needed
            while photocount < 10: #To ensure only 10 photos are taken
                cam.resolution =(reslist[iteratex],int(3*reslist[iteratex]/4)) #Setting the current camera resolution
                datetime_string = datetime.utcnow().strftime("%Y_%m_%d_%H_%M_%S_%f") #For frames_taken.txt
                if not os.path.exists(BigFolder+str(reslist[rescount])+'/RawPhotos/'): #Create folder for photos that have been taken
                    os.makedirs(BigFolder+str(reslist[rescount])+'/RawPhotos/')
                cam.capture(BigFolder+str(reslist[iteratex])+'/RawPhotos/' + str(photocount)+'.jpg') #Taking and storing photos
                timestamp.append(datetime_string) #Storing and writing when a photo is taken
                f.write(str(timestamp[photocount])+ '\n')
                img = cv2.imread(BigFolder+str(reslist[rescount])+'/RawPhotos/'+str(photocount) + '.jpg') #Using cv2 to detect features
                grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                sift = cv2.xfeatures2d.SIFT_create()
                kp,des = sift.detectAndCompute(grey,None)
                img = cv2.drawKeypoints(grey,kp, outImage = np.array([]))
                if not os.path.exists(BigFolder+str(reslist[rescount])+'/PointsPhotos/'): #Create folder for photos after SIFT filter
                    os.makedirs(BigFolder+str(reslist[rescount])+'/PointsPhotos/')
                cv2.imwrite(BigFolder+str(reslist[rescount])+'/PointsPhotos/'+str(photocount) +'.jpg',img) #Storing new photo
                KeypointCount.append(len(kp)) #Noting number of keypoints detected in each photo
                s = str(len(kp))
                #print(len(kp))
                file.write(str(KeypointCount[photocount])+ '\n') #Adding to points_detected.txt
                if not os.path.exists(BigFolder+str(reslist[rescount])+'/PointLocations/'): #Create folder for the co-ordinates of features
                    os.makedirs(BigFolder+str(reslist[rescount])+'/PointLocations/') #for each individual photo
                filename3 = BigFolder+str(reslist[rescount])+'/'+'PointLocations/'+str(photocount)+'.txt'
                with open(filename3,'w+') as f3: #Creating a file for each photo taken
                    for m in range(len(kp)): #Storing each keypoint's location
                        timestamplist.append(kp[m].pt)
                        f3.write(str(timestamplist[m])+'\n')
                f3.close() #Closing the keypoint location filename
                photocount +=1 #Changing and resetting values for next iteration
                timestamplist = []
                s2 = time.time() #Time for end of run
                #m2 = int(datetime.utcnow().strftime("%M"))
            f.close() #Close timestamp file
            iteratex +=1 #Change iteration of resolution
    photocount = 0 #Resetting values
    stop = 0
    frames = len(timestamp) #Used for measuring number of photos in a given time
    print('Camera Resolution: ' + str(reslist[rescount])) #Print camera resolution
    S = s2-s1 #Printing the time taken for 10 photos
    #M = m2-m1
    print('Time Taken for 10 photos: '+str(S)+'secs')
    #print('Photos taken in 10seconds ' + str(frames))
    timestamp = [] #Resetting values for the for loops
    rescount +=1
    KeypointCount = []
    file.close()