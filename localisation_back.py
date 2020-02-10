# -*- coding: utf-8 -*-
"""
Created on Sun Jul 05 15:01:58 2015

@author: ACSECKIN
"""

import vrep
import time
import cv2
import sys
import socket
from math import *
import numpy as np
from scipy import optimize
import matplotlib.pyplot as mpl
import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import math


vrep.simxFinish(-1)

clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)



if clientID!=-1:
    print 'Connected to remote API server'
    print 'Vision Sensor object handling'
    res, v1 = vrep.simxGetObjectHandle(clientID, 'Cam', vrep.simx_opmode_oneshot_wait)
    print 'Getting first image'
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_streaming)
    while (vrep.simxGetConnectionId(clientID) != -1):
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_buffer)
        if err == vrep.simx_return_ok:
            img = np.array(image,dtype=np.uint8)
            img.resize([resolution[1],resolution[0],3])
            img=cv2.flip(img,0)
            cv2.imshow('orig',img)   
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters =  aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
            l=[]
            position=[]
            if ids is not None:
                for i in range(len(ids)):
                    c = corners[i][0]
                    cv2.rectangle(img, (c[0][0], c[0][1]), (c[2][0], c[2][1]), (0, 0, 255), 2) 
                    if ids[i]==3:
                        x0=c[:, 0].mean()
                        y0=c[:, 1].mean() 
                        l.append(math.sqrt(np.square(c[0][0]-c[1][0])+np.square(c[0][1]-c[1][1])))
                    if ids[i]==2:
                        x1=c[:, 0].mean()
                        y1=c[:, 1].mean()
                        l.append(math.sqrt(np.square(c[0][0]-c[1][0])+np.square(c[0][1]-c[1][1])))
                    if ids[i]==4:
                        x2=c[:, 0].mean()
                        y2=c[:, 1].mean()
                        l.append(math.sqrt(np.square(c[0][0]-c[1][0])+np.square(c[0][1]-c[1][1])))
                    if ids[i]==1:
                        x3=c[:, 0].mean()
                        y3=c[:, 1].mean()
                        l.append(math.sqrt(np.square(c[0][0]-c[1][0])+np.square(c[0][1]-c[1][1])))
                l=np.array(l)    
                l=l.mean()
                position.append(5)
                position.append(5)
                
                position.append(5-(y1-y0)/(l+1.50))
                position.append(5-(x1-x0)/(l+1.75))
                position.append(5-(y2-y0)/(l+1.75))
                position.append(5-(x2-x0)/(l+1.75))
                position.append(5-(y3-y0)/(l+1.75))
                position.append(5-(x3-x0)/(l+1.50))    
                print position  
                #print math.sqrt(np.square(c[0][0]-c[1][0])+np.square(c[0][1]-c[1][1]))
                #    print math.sqrt(np.square(c[2][0]-c[3][0])+np.square(c[2][1]-c[3][1]))
                cv2.imshow('image',img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif err == vrep.simx_return_novalue_flag:
            print "no image yet"
            pass
        elif err == vrep.simx_return_remote_error_flag:
          print err
          print clientID
else:
  print "Failed to connect to remote API Server"
  vrep.simxFinish(clientID)

cv2.destroyAllWindows()
