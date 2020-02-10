# -*- coding: utf-8 -*-
"""
Created on Sun Jul 05 15:01:58 2015

@author: ACSECKIN
"""
import matlab.engine
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
eng = matlab.engine.start_matlab()
import scipy.io as sio 

vrep.simxFinish(-1)

clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

position0=[5,5,5,5,5,5,5,5]
droneori=np.array([5,0,2])
visiondata=[]

if clientID!=-1:
    print 'Connected to remote API server'
    print 'Vision Sensor object handling'
    res, v1 = vrep.simxGetObjectHandle(clientID, 'Cam', vrep.simx_opmode_oneshot_wait)
    print 'Getting new images'
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_streaming)
    res, HandleTransceiver1 = vrep.simxGetObjectHandle(clientID,'Transceiver1',vrep.simx_opmode_blocking);
    res, HandleTransceiver2 = vrep.simxGetObjectHandle(clientID,'Transceiver2',vrep.simx_opmode_blocking);
    res, HandleTransceiver3 = vrep.simxGetObjectHandle(clientID,'Transceiver3',vrep.simx_opmode_blocking);
    res, HandleTransceiver4 = vrep.simxGetObjectHandle(clientID,'Transceiver4',vrep.simx_opmode_blocking);


    res, motorFlyF = vrep.simxGetObjectHandle(clientID,'Quadricopter',vrep.simx_opmode_blocking);
    res, motorFlyFtarget = vrep.simxGetObjectHandle(clientID,'flytarget#0',vrep.simx_opmode_blocking);
    res, motorFlyF1 = vrep.simxGetObjectHandle(clientID,'quadricopter_propellr1',vrep.simx_opmode_blocking);
    res, motorFlyF2 = vrep.simxGetObjectHandle(clientID,'quadricopter_propellr2',vrep.simx_opmode_blocking);
    res, motorFlyF3 = vrep.simxGetObjectHandle(clientID,'quadricopter_propellr3',vrep.simx_opmode_blocking);
    res, motorFlyF4 = vrep.simxGetObjectHandle(clientID,'quadricopter_propellr4',vrep.simx_opmode_blocking);

    res, pos = vrep.simxGetObjectPosition (clientID, motorFlyF, -1, vrep.simx_opmode_streaming);
    res, pos = vrep.simxGetObjectPosition (clientID, HandleTransceiver1, -1, vrep.simx_opmode_streaming);
    res, pos = vrep.simxGetObjectPosition (clientID, HandleTransceiver2, -1, vrep.simx_opmode_streaming);
    res, pos = vrep.simxGetObjectPosition (clientID, HandleTransceiver3, -1, vrep.simx_opmode_streaming);
    res, pos = vrep.simxGetObjectPosition (clientID, HandleTransceiver4, -1, vrep.simx_opmode_streaming);

    time.sleep(1)
    Anchor1 = np.array([0, 0], dtype=float)
    Anchor2 = np.array([0, 0], dtype=float)
    Anchor3 = np.array([0, 0], dtype=float)
    Anchor4 = np.array([0, 0], dtype=float)
  #  vrep.simxSetObjectPosition(clientID, motorFlyF,-1, droneori,vrep.simx_opmode_oneshot) 
  #  vrep.simxSetObjectPosition(clientID, motorFlyFtarget,-1, droneori,vrep.simx_opmode_oneshot)  
  #  time.sleep(5)  

    while (vrep.simxGetConnectionId(clientID) != -1):
        
        
        for ii in range(0,2001):
            err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_buffer)
           
            if err == vrep.simx_return_ok:
                img = np.array(image,dtype=np.uint8)
                img.resize([resolution[1],resolution[0],3])
                img=cv2.flip(img,0)
            
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
                parameters =  aruco.DetectorParameters_create()
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                frame_markers = aruco.drawDetectedMarkers(img.copy(), corners, ids)
                l=[]
                position=[]
           
                if ids is not None:
                    for i in range(0,len(ids)):
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
                    visiondata.append(position)
                    save_fn = 'vision.mat'   
                    sio.savemat(save_fn, {'array_x': visiondata})
                    
                    
                    for iii in range(0,7):
                        judge=position0[iii]-position[iii]
                        if math.fabs(judge)>0.3:
                            flag=True
                            break
                        else:
                            flag=False       
                    if ii==0:
                        retback = eng.Plot_TOF(float(position[0]),float(position[1]),float(position[2]),float(position[3]),float(position[4]),float(position[5]),float(position[6]),float(position[7]))
                    cv2.imshow('image',img)
                    if flag==False:
                        print ii
                        vrep.simxSetObjectPosition(clientID, motorFlyFtarget,-1, retback[ii],vrep.simx_opmode_oneshot)    
                        posres=eng.Test_Function_TWTOF(retback[ii],float(position[0]),float(position[1]),float(position[2]),float(position[3]),float(position[4]),float(position[5]),float(position[6]),float(position[7]))
                    
                        time.sleep(0.01)
                    else:
                       
                       print 0
                       ii=0
                       res = vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
                       time.sleep(1)                 
                       vrep.simxSetObjectPosition(clientID, HandleTransceiver3,-1, np.array([position[0],position[1],0]),vrep.simx_opmode_oneshot) 
                       vrep.simxSetObjectPosition(clientID, HandleTransceiver2,-1, np.array([position[2],position[3],0]),vrep.simx_opmode_oneshot) 
                       vrep.simxSetObjectPosition(clientID, HandleTransceiver1,-1, np.array([position[4],position[5],0]),vrep.simx_opmode_oneshot) 
                       vrep.simxSetObjectPosition(clientID, HandleTransceiver4,-1, np.array([position[6],position[7],0]),vrep.simx_opmode_oneshot) 
                       res = vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
                       time.sleep(1)
                       position0=position
                       retback = eng.Plot_TOF(float(position[0]),float(position[1]),float(position[2]),float(position[3]),float(position[4]),float(position[5]),float(position[6]),float(position[7]))
                       break
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
