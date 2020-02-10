% V-REP turtlebot path planning

%% Initialise
addpath(genpath('TRRTstar'))
addpath('vrep_api')
addpath('fcns')
load('AFRC_table_sim_cm.mat')

map(50:80,20:50) = 1;
map_size = size(map);

turtlebot.start = [95,17.5];
turtlebot.heading = 0;
turtlebot.wheel_radius = 3.3;
turtlebot.wheel_sep = 8.1;
turtlebot.world_frame_translation = [9.65, -6.25, 0];

tolerance.heading = 3*pi/180; % robot considered to have reached heading when within 3 degrees
tolerance.goal = 3; % robot considered to have reached goal when 1 cm from target
tolerance.target = 5; % tolerance for path tracking

vel.steer = 100*pi/180; % wheel velocity for pure steering
vel.forw_max = 10; % max forward velocity
vel.rot_max = 20*pi/180; % max rotational velocity

% path planning parameters
params.stepsize = 3;
params.maxiter = 3000;
params.plot_results = 1;

% generate random goal
goal_found = 0;

while goal_found == 0
    goal = [rand*(map_size(2)-1)+1,rand*(map_size(1)-1)+1];
    
    if map(round(goal(2)),round(goal(1)))==1
        continue
    end
    
    goal_found = 1;
    turtlebot.goal = goal;
end

%% path planning
% if collision_check(turtlebot.start,turtlebot.goal,map,2) == 1 
    % cannot reach goal in straight segment from start - use path planning
    [solution,~,problem] = TRRTstar( turtlebot.start,turtlebot.goal,map,params.maxiter,params.stepsize,params.plot_results );
    path = solution.xy;
% else 
%     path = [turtlebot.start;turtlebot.goal];
%     problem.x_min = 0.5; problem.y_min = 0.5;
%     problem.x_max = length(map(1,:))+0.5; problem.y_max = length(map(:,1))+0.5;
%     problem.cost_type = 'c';
%     initPlot(map,[],problem);
%     plot(path(:,1),path(:,2),'bo','MarkerSize',5)
%     plot(path(:,1),path(:,2),'k','LineWidth',2);
% end

drawnow

%% Connect to V-REP
disp('Opening connection to V-REP environment');
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    % get object handles
    [~,handles.turtlebot]=vrep.simxGetObjectHandle(clientID,'Turtlebot3_Burger',vrep.simx_opmode_blocking);
    vrep.simxGetPingTime(clientID);
    [~,handles.tb3_origin]=vrep.simxGetObjectHandle(clientID,'tb3_coord_frame',vrep.simx_opmode_blocking);
    vrep.simxGetPingTime(clientID);
    [~,handles.left_motor]=vrep.simxGetObjectHandle(clientID,'left_motor',vrep.simx_opmode_blocking);
    vrep.simxGetPingTime(clientID);
    [~,handles.right_motor]=vrep.simxGetObjectHandle(clientID,'right_motor',vrep.simx_opmode_blocking);
    vrep.simxGetPingTime(clientID);
    [~,handles.tb3_plate]=vrep.simxGetObjectHandle(clientID,'tb3_plate',vrep.simx_opmode_blocking);
    vrep.simxGetPingTime(clientID);
    
    % initialise getter for turtlebot orientation and position
    [~,turtlebot.vrep_euler] = vrep.simxGetObjectOrientation(clientID, handles.turtlebot, handles.tb3_origin, vrep.simx_opmode_streaming);
    vrep.simxGetPingTime(clientID);
    turtlebot.vrep_pos = vrepGetPose(vrep, clientID, handles.turtlebot, turtlebot.world_frame_translation, 1);
    vrep.simxGetPingTime(clientID);
    turtlebot.vrep_pos = vrepGetPose(vrep, clientID, handles.turtlebot, turtlebot.world_frame_translation, 2);
    
    %% steer robot to heading direction of path
    % get target heading
    direction = path(2,:) - path(1,:);
    heading = atan2(direction(2),direction(1));
    
    % get robot orientation
    [~,turtlebot.vrep_euler] = vrep.simxGetObjectOrientation(clientID, handles.turtlebot, handles.tb3_origin, vrep.simx_opmode_buffer);
    vrep.simxGetPingTime(clientID);
    turtlebot.heading = turtlebot.vrep_euler(1);
    heading_diff = heading - turtlebot.heading;
    
    % steer
    disp('Steering robot to start heading direction...')
    while norm(heading_diff) > tolerance.heading
        if sign(heading_diff) == -1
            % steer right
            vrep.simxSetJointTargetVelocity(clientID, handles.left_motor,vel.steer,vrep.simx_opmode_streaming);
            vrep.simxSetJointTargetVelocity(clientID, handles.right_motor,-1*vel.steer,vrep.simx_opmode_streaming);
        else
            vrep.simxSetJointTargetVelocity(clientID, handles.left_motor,-1*vel.steer,vrep.simx_opmode_streaming);
            vrep.simxSetJointTargetVelocity(clientID, handles.right_motor,vel.steer,vrep.simx_opmode_streaming);
        end
        [~,turtlebot.vrep_euler] = vrep.simxGetObjectOrientation(clientID, handles.turtlebot, handles.tb3_origin, vrep.simx_opmode_buffer);
        vrep.simxGetPingTime(clientID);
        turtlebot.heading = turtlebot.vrep_euler(1);
        heading_diff = heading - turtlebot.heading;
    end
    
    vrep.simxSetJointTargetVelocity(clientID, handles.left_motor,0,vrep.simx_opmode_streaming);
    vrep.simxSetJointTargetVelocity(clientID, handles.right_motor,0,vrep.simx_opmode_streaming);
   
    %% follow planned path
    disp('Following planned path...')
    goal_reached = 0;
    turtlebot.vrep_pos = vrepGetPose(vrep, clientID, handles.turtlebot, turtlebot.world_frame_translation, 2);
    [~,turtlebot.vrep_euler] = vrep.simxGetObjectOrientation(clientID, handles.turtlebot, handles.tb3_origin, vrep.simx_opmode_buffer);
    vrep.simxGetPingTime(clientID);
    turtlebot.heading = turtlebot.vrep_euler(1);
    
    % get target
    target = path(2,:);
    idx = 0;
    
    % drive along path
    while goal_reached == 0
        target_vector = target-turtlebot.vrep_pos(1:2); % direction vector from robot to target
        dist2target = sqrt(sum(target_vector.^2));
        if (dist2target<tolerance.target && any(target~=goal))||idx==0
            idx = dsearchn(path,turtlebot.vrep_pos(1:2));
            target = advanceTarget(path,idx,tolerance.target);
            target_vector = target-turtlebot.vrep_pos(1:2);
        end
        
        if exist('target_handle','var')==1
            delete(target_handle)
        end
        target_handle = plot(target(1),target(2),'bx');
        plot(turtlebot.vrep_pos(1),turtlebot.vrep_pos(2),'ro')
        drawnow
        
        % drive robot towards target
        target_direction = atan2(target_vector(2),target_vector(1));
        heading_diff = target_direction - turtlebot.heading(1);
        if heading_diff<-pi
            heading_diff = heading_diff+2*pi;
        elseif heading_diff>pi
            heading_diff = heading_diff-2*pi;
        end
        vel.rot = sign(heading_diff)*min(abs(heading_diff*0.8),vel.rot_max);
        vel.forw = (vel.forw_max-1) * (1 - abs(vel.rot)/vel.rot_max) + 1;
        
        % compute wheel velocities
        vel.right_wheel = vel.forw - turtlebot.wheel_sep*vel.rot;
        vel.left_wheel = vel.forw + turtlebot.wheel_sep*vel.rot;
        vel.right_motor = vel.right_wheel/turtlebot.wheel_radius;
        vel.left_motor = vel.left_wheel/turtlebot.wheel_radius;
        
        vrep.simxSetJointTargetVelocity(clientID, handles.left_motor,vel.right_motor,vrep.simx_opmode_streaming);
        vrep.simxSetJointTargetVelocity(clientID, handles.right_motor,vel.left_motor,vrep.simx_opmode_streaming);

        % Update pose information
        turtlebot.vrep_pos = vrepGetPose(vrep, clientID, handles.turtlebot, turtlebot.world_frame_translation, 2);
        [~,turtlebot.vrep_euler] = vrep.simxGetObjectOrientation(clientID, handles.turtlebot, handles.tb3_origin, vrep.simx_opmode_buffer);
        vrep.simxGetPingTime(clientID);
        turtlebot.heading = turtlebot.vrep_euler(1);

        % Check if goal reached
        dist2goal = sqrt(sum((goal-turtlebot.vrep_pos(1:2)).^2));
        if dist2goal < tolerance.goal
            goal_reached = 1;
            disp('Goal reached.')
        end
    end
    
    vrep.simxSetJointTargetVelocity(clientID, handles.left_motor,0,vrep.simx_opmode_streaming);
    vrep.simxSetJointTargetVelocity(clientID, handles.right_motor,0,vrep.simx_opmode_streaming);
    
    disp('Grasping composite material...')
    vacuumGraspComposite(vrep, clientID)
    disp('Placement of composite material successful.')
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

disp('Program ended.');