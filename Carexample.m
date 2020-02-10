clc;
clear;

% P = py.sys.path;
% if count(py.sys.path,'') == 0
%     insert(py.sys.path,int32(0),'');
% end

disp('Opening connection to V-REP environment');
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
if (clientID<0)
    disp('Failed connecting');
else
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
end
toc = 1;
m_dDistance = 0.0;
m_iresult = 0;
detectedPoint = zeros(1,3);
%[~,handles.car_origin]=vrep.simxGetObjectHandle(clientID,'car_coord_frame',vrep.simx_opmode_blocking);



%%获取anchor坐标
%     [res,HandleCam] = vrep.simxGetObjectHandle(clientID, 'Cam', vrep.simx_opmode_oneshot_wait);
    
%     [err, resolution, image] = vrep.simxGetVisionSensorImage(clientID, HandleCam, 0, vrep.simx_opmode_streaming);
%     [err, resolution, image] = vrep.simxGetVisionSensorImage2(clientID, HandleCam, 0, vrep.simx_opmode_oneshot_wait);
%     py.mymod.Cam(image,resolution);
%         img = im2double(image);
%          py.cv2.imshow("aaa",img);
%         py.cv2.cvtColor(image,py.cv2.COLOR_BGR2GRAY);
        
        
%     if (vrep.simxGetConnectionId(clientID) ~= -1)
%         [err, resolution, image] = vrep.simxGetVisionSensorImage(clientID, HandleCam, 0, vrep.simx_opmode_oneshot_wait);
%         if err == vrep.simx_return_ok
%             a = 1;
%             b=2;
%             imshow(image);
%             pause(10);
%         end
%     else
%         vrep.simxFinish(clientID)
%     end
%%



[res,HandleTransceiver1]=vrep.simxGetObjectHandle(clientID,'Transceiver1',vrep.simx_opmode_blocking);
[res,HandleTransceiver2]=vrep.simxGetObjectHandle(clientID,'Transceiver2',vrep.simx_opmode_blocking);
[res,HandleTransceiver3]=vrep.simxGetObjectHandle(clientID,'Transceiver3',vrep.simx_opmode_blocking);
[res,HandleTransceiver4]=vrep.simxGetObjectHandle(clientID,'Transceiver4',vrep.simx_opmode_blocking);

[res,motorHandlesFLeft]=vrep.simxGetObjectHandle(clientID,'joint_front_left_wheel',vrep.simx_opmode_blocking);
[res,motorHandlesFRight]=vrep.simxGetObjectHandle(clientID,'joint_front_right_wheel',vrep.simx_opmode_blocking);
[res,motorHandlesBRight]=vrep.simxGetObjectHandle(clientID,'joint_back_right_wheel',vrep.simx_opmode_blocking);
[res,motorHandlesBLeft]=vrep.simxGetObjectHandle(clientID,'joint_back_left_wheel',vrep.simx_opmode_blocking);
% [res,DistanceHandles]=vrep.simxGetObjectHandle(clientID,'Distance_sensor',vrep.simx_opmode_blocking);
[res,ProximityHandles]=vrep.simxGetObjectHandle(clientID,'Distance_sensor',vrep.simx_opmode_blocking);

[res,motorFlyF]=vrep.simxGetObjectHandle(clientID,'Quadricopter',vrep.simx_opmode_blocking);
[res,motorFlyFtarget]=vrep.simxGetObjectHandle(clientID,'Quadricopter_target',vrep.simx_opmode_blocking);
[res,motorFlyF1]=vrep.simxGetObjectHandle(clientID,'quadricopter_propellr1',vrep.simx_opmode_blocking);
[res,motorFlyF2]=vrep.simxGetObjectHandle(clientID,'quadricopter_propellr2',vrep.simx_opmode_blocking);
[res,motorFlyF3]=vrep.simxGetObjectHandle(clientID,'quadricopter_propellr3',vrep.simx_opmode_blocking);
[res,motorFlyF4]=vrep.simxGetObjectHandle(clientID,'quadricopter_propellr4',vrep.simx_opmode_blocking);



% initialise getter for carbot orientation and position
pos = zeros(1,3);
[res,pos] = vrep.simxGetObjectPosition (clientID, motorFlyF, -1, vrep.simx_opmode_streaming);
m_iAnchorTag = zeros(4,3);
[res,pos] = vrep.simxGetObjectPosition (clientID, HandleTransceiver1, -1, vrep.simx_opmode_streaming);
[res,pos] = vrep.simxGetObjectPosition (clientID, HandleTransceiver2, -1, vrep.simx_opmode_streaming);
[res,pos] = vrep.simxGetObjectPosition (clientID, HandleTransceiver3, -1, vrep.simx_opmode_streaming);
[res,pos] = vrep.simxGetObjectPosition (clientID, HandleTransceiver4, -1, vrep.simx_opmode_streaming);



[errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector] = vrep.simxReadProximitySensor(clientID,ProximityHandles,vrep.simx_opmode_streaming);
 vrep.simxSetJointTargetVelocity(clientID,motorHandlesFLeft,0,vrep.simx_opmode_oneshot);
 vrep.simxSetJointTargetVelocity(clientID,motorHandlesFRight,0,vrep.simx_opmode_oneshot);
 vrep.simxSetJointTargetVelocity(clientID,motorHandlesBRight,0,vrep.simx_opmode_oneshot);
 vrep.simxSetJointTargetVelocity(clientID,motorHandlesBLeft,0,vrep.simx_opmode_oneshot);
 
 pause(1);  %初始化等待

[res,pos] = vrep.simxGetObjectPosition (clientID, motorFlyF, -1, vrep.simx_opmode_buffer);

targetNumber = 1000;
targetxyz = circle(5,0,0,targetNumber);

[res,m_iAnchorTag(1,:)] = vrep.simxGetObjectPosition (clientID, HandleTransceiver1, -1, vrep.simx_opmode_buffer);
[res,m_iAnchorTag(2,:)] = vrep.simxGetObjectPosition (clientID, HandleTransceiver2, -1, vrep.simx_opmode_buffer);
[res,m_iAnchorTag(3,:)] = vrep.simxGetObjectPosition (clientID, HandleTransceiver3, -1, vrep.simx_opmode_buffer);
[res,m_iAnchorTag(4,:)] = vrep.simxGetObjectPosition (clientID, HandleTransceiver4, -1, vrep.simx_opmode_buffer);


% m_iAnchorTag(1,1) = -50;
% m_iAnchorTag(1,2) = -50;
% m_iAnchorTag(2,1) = 50;
% m_iAnchorTag(2,2) = -50;
% m_iAnchorTag(3,1) = 50;
% m_iAnchorTag(3,2) = 50;
% m_iAnchorTag(4,1) = -50;
% m_iAnchorTag(4,2) = 50;
m_z = 1;
figure(1)
plot3(m_iAnchorTag(:,1)*10,m_iAnchorTag(:,2)*10,m_iAnchorTag(:,3)*10,'gs','LineWidth',2);
xlabel('X coordinate of target');
ylabel('Y coordinate of target');
zlabel('Z coordinate of target');
title('TW-TOF Localization');
axis([-100 100 -100 100 0 6]);
drawnow;
hold on;

while 1
    if toc<(targetNumber*2+1)
        [res,carbot.vrep_euler] = vrep.simxGetObjectPosition (clientID, motorFlyF, -1, vrep.simx_opmode_buffer);
        if res==0
            AnchorPos = zeros(4,2);
            LocTag = zeros(1,2);
            Estimate_Tag = zeros(1,2);
            LocTag(1,1) = carbot.vrep_euler(1,1)*10;
            LocTag(1,2) = carbot.vrep_euler(1,2)*10;
            for m_NumAnchor = 1:4
                AnchorPos(m_NumAnchor,1) = m_iAnchorTag(m_NumAnchor,1)*10;
                AnchorPos(m_NumAnchor,2) = m_iAnchorTag(m_NumAnchor,2)*10;
            end
            Estimate_Tag = TW_TOF(LocTag,AnchorPos,1e-9);
            plot3(carbot.vrep_euler(1,1)*10,carbot.vrep_euler(1,2)*10,m_z,'b-*','LineWidth',1);
            plot3(Estimate_Tag(1,1),Estimate_Tag(1,2),m_z,'r-.x','LineWidth',1);
            drawnow;
            toc = toc+1;
        end
        res = vrep.simxSetObjectPosition(clientID,motorFlyFtarget,-1,targetxyz(toc,:),vrep.simx_opmode_oneshot);  
    else 
        toc = 1;
    end
    if m_z<6
        m_z = m_z+1;
    else
        m_z = 1;
    end
end
res = vrep.simxSetObjectPosition(clientID,motorFlyFtarget,-1,targetxyz(1,:),vrep.simx_opmode_oneshot);
vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
vrep.simxFinish(clientID);
vrep.delete();
%%
function [cicxyz] = circle(R,cx,cy,nb_pts)
%%%%%%%%%%%%%%%%%%%
% 画圆函数
%%%%%%%%%%%%%%%%%%%
cicxyz = zeros(nb_pts*2+1,3);
alpha=0:pi/nb_pts:2*pi;%角度[0,2*pi]
%R=2;%半径
x=R*cos(alpha)+cx;
y=R*sin(alpha)+cy;
cicxyz(:,1) = x';
cicxyz(:,2) = y';
cicxyz(:,3) = 2 ;
end