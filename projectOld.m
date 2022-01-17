%% Design for Collaborative Assembly with the ICE Lab Robots
clc; clear all; close all;

%% Best subtasks allocation

% list of points in the workspace
P1 = [0.5, 0, 0.2, 0, pi, 0]; % position of center
P2 = [0.9,0.6, 0.2, 0, pi, 0]; %position of pick 
P3 = [0.5, 0, 0.2, 0, 0, pi/2]; % position of center but from right orientation
P4 = [1, 1, 1, pi, 0, pi/2];

% List of subtasks:
%   startPos
%   endPos
%   taskTime
%   type (move, assembly)
%   Dk
%   Da
%   Pk
%   Pa
%   description
subs = [
    subtask(P1,P2,5,'move',1,0.3,0.1,0.3)
    subtask(P2,P1,5,'move',1,0.3,0.1,0.3)
    subtask(P1,P2,5,'move',1,0.3,0.1,0.3)
    subtask(P2,P1,5,'move',1,0.3,0.1,0.3)
    subtask(P1,P3,5,'move',1,0.3,0.1,0.3)
%     subtask(P1,P2,5,'move',1,0.1,0.4,0.1)
%     subtask(P4,P1,5,'assembly',0.1,0.6,0.7,0.1)
%     subtask(P3,P1,5,'assembly',0.5,0.1,0.7,0.1)
];

% Weight factors
W = [0.8, 0.5, 0.4, 0.6];

% Best allocation algorithm
allocation = bestAllocation(subs, W);

%% Tasks Simulation

abbFrame = [
-1 0 0 1
0 -1 0 0
0 0 1 0
0 0 0 1
];

kukaFrame = eye(4);

% Cobot models
yumiRBT = loadrobot('abbYuMi');
iiwaRBT = loadrobot('kukaIiwa14');

% Inverse kinematics of cobots
iiwaIK = inverseKinematics('RigidBodyTree', iiwaRBT);
yumiIK = inverseKinematics('RigidBodyTree', yumiRBT);

% Sampling time for trajectories
Ts = 0.001;

nTasks = length(subs);
for i=1:nTasks
	task_i = subs(i);
	
    % Executive robot
	if logical(str2num(allocation(i)))
		execCobot = iiwaRBT;
        execIK = iiwaIK;
        execEEName = 'iiwa_link_ee_kuka';
        frame = kukaFrame;
	else
		execCobot = yumiRBT;
        execIK = yumiIK;
        execEEName = 'gripper_r_base';   
        frame = abbFrame;
    end
    execHomeConfig = homeConfiguration(execCobot);
    
    % Inverse Kinematics of Intermediate Configurations
    weights = ones(1,6);

    % IK of picking position
    pickPosT = eul2tform(task_i.startPos(4:6));
    pickPosT(1:3,end) = task_i.startPos(1:3)';
    pickPosT = frame*pickPosT;
    [pickConfig, pickSolInfo] = execIK(execEEName, pickPosT, weights, execHomeConfig);
    
    % IK of placing position
    placePosT = eul2tform(task_i.endPos(1,4:6));
    placePosT(1:3,end) = task_i.endPos(1,1:3)';
    placePosT = frame*placePosT;
    [placeConfig, placeSolInfo] = execIK(execEEName, placePosT, weights, pickConfig);

    % IK of final home position
    placePosT = eul2tform(task_i.endPos(1,4:6));
    placePosT(1:3,end) = task_i.endPos(1,1:3)';
    placePosT = frame*placePosT;
    [finalHConfig, finalHSolInfo] = execIK(execEEName, placePosT, weights, pickConfig);

    % Trajectory Computation
    clear traj;
    
    nJoints = length(execHomeConfig);
    for j=1:nJoints
        
        % Intermediate configurations
        homeq = execHomeConfig(j).JointPosition;
        pickq = pickConfig(j).JointPosition;
        placeq = placeConfig(j).JointPosition;
        positions = [pickq, placeq];

        % Time distribution for the current subtaks
        if j == 1
            tk = getTimeDistrubution(positions, 'eq');
            tk = round(tk/Ts)*Ts*task_i.taskTime;
        end
        
        points = [
            tk          % Time Series
            positions   % Joint Configurations
            zeros(1, length(positions)) % Joint Velocities
        ];

        % Compute trajectory for the j-th joint
        traj(j) = multiPointImpV(points, Ts);

    end % for j=1:nJoints
    
    if i == 1
        %start building up the full trajectory, this is the case for 1st
        %subtask
        if execCobot == iiwaRBT
            iiwaTraj = traj; %first robot is iiwa, we copy in it the generated trajectory
            for k = 1:18
                %for the other robot instead we create an "empty"
                %trajectory for each joint
                points(2,:) = zeros(1,length(positions));
                yumiTraj(k) = multiPointImpV(points, Ts);
            end
        else
            %inverted robots
            yumiTraj = traj;
            for k = 1:7
                points(2,:) = zeros(1,length(positions));
                iiwaTraj(k) = multiPointImpV(points, Ts);
            end
        end
    else
        %for successive substasks
        if execCobot == iiwaRBT
            for k = 1:18 %we add a new piece of trajectory to each joint
                if k <= 7 %iiwa has 7 joints
                    %iiwa is the current cobot so we merge the generated
                    %trajectory
                    iiwaTraj(k) = mergeTrajectories([iiwaTraj(k),traj(k)]);
                end
                %here we create a fixed trajectory with value 1
                points(2,:) = ones(1,length(positions));
                newTraj(k) = multiPointImpV(points, Ts);
                %we set each value of the trajectory equal to the last
                %position of the other cobot 
                newTraj(k).q = newTraj(k).q*yumiTraj(k).q(length(yumiTraj(k).q));
                %so we merge this new piece of trajectory
                yumiTraj(k) = mergeTrajectories([yumiTraj(k), newTraj(k)]);
            end
        else
            %inverted robots
            for k = 1:18
                if k <= 7 %in this case is the iiwa that need a constant trajectory
                    points(2,:) = ones(1,length(positions));
                    newTraj(k) = multiPointImpV(points, Ts);
                    newTraj(k).q = newTraj(k).q*iiwaTraj(k).q(length(iiwaTraj(k).q));
                    iiwaTraj(k) = mergeTrajectories([iiwaTraj(k), newTraj(k)]);
                end
                yumiTraj(k) = mergeTrajectories([yumiTraj(k),traj(k)]);
            end
        end % if execCobot == iiwaRBT
    end % if i == 1
    
end % for i=1:nTasks
%keep away the left arm
yumiTraj(1).q = ones(1, length(yumiTraj(1).q));
%%
% open('projectSimscape.slx');
sim('projectSimscape.slx', 25);