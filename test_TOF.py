import matlab.engine
import time
import vrep
import numpy as np
eng = matlab.engine.start_matlab()

vrep.simxFinish(-1)

clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID!=-1:

    res, CamHandle = vrep.simxGetObjectHandle(clientID, 'Cam', vrep.simx_opmode_oneshot_wait)
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
    res, marker = vrep.simxGetObjectHandle(clientID,'Plane',vrep.simx_opmode_blocking);

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
    res, Anchor1 = vrep.simxGetObjectPosition (clientID, HandleTransceiver1, -1, vrep.simx_opmode_buffer);
    res, Anchor2 = vrep.simxGetObjectPosition (clientID, HandleTransceiver2, -1, vrep.simx_opmode_buffer);
    res, Anchor3 = vrep.simxGetObjectPosition (clientID, HandleTransceiver3, -1, vrep.simx_opmode_buffer);
    res, Anchor4 = vrep.simxGetObjectPosition (clientID, HandleTransceiver4, -1, vrep.simx_opmode_buffer);

    retback = eng.Plot_TOF(Anchor1[0],Anchor1[1],Anchor2[0],Anchor2[1],Anchor3[0],Anchor3[1],Anchor4[0],Anchor4[1])
    for i in range(0,len(retback)):

        vrep.simxSetObjectPosition(clientID, motorFlyFtarget,-1, retback[i],vrep.simx_opmode_oneshot)    
        posres=eng.Test_Function_TWTOF(retback[i],Anchor1[0],Anchor1[1],Anchor2[0],Anchor2[1],Anchor3[0],Anchor3[1],Anchor4[0],Anchor4[1])
        print posres
        print retback[i]
        time.sleep(0.01) 
   # posres = np.array([0, 0], dtype=float)
    
    
else:
  print('failed')
  vrep.simxFinish(clientID)


