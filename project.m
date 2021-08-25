%% this is used to generate the subsystem in simscape
clc; clear all; close all;

yumi = loadrobot('abbYuMi');
% yumiSM = smimport(yumi);
iiwaRBT = loadrobot('kukaIiwa14');
% iiwaSM = smimport(iiwaRBT);

%% list of subtasks
P1 = [1, 1, 0, 0, 0, 0]; % minipallet
P2 = [2, 3, 0, 0, 0, pi/2]; % vicino al kuka
P3 = [10, 10, 0, pi/4, pi/4, 0]; % vicino all'abb
P4 = [1, 1, 1, pi, 0, pi/2]; % vicino al kuka
%         startPos
%         endPos
%         type %move/assemble
%         Dk %kuka potential incapability coeff
%         Da %abb
%         Pk %kuka potential insufficeincy coeff for precios
%         Pa %abb
%         description
subs = [
    subtask(P1,P2,5,'move',0.1,0.3,0.1,0.3)
    subtask(P1,P3,5,'move',0.5,0.1,0.4,0.1)
    subtask(P4,P1,5,'assembly',0.1,0.6,0.7,0.1)
    subtask(P3,P1,5,'assembly',0.5,0.1,0.7,0.1)
];
W = [1, 1, 1, 1];
allocation = bestAllocation(subs, W);

%% Tasks Simulation
nTasks = length(subs);
    
iiwaIK = inverseKinematics('RigidBodyTree', iiwaRBT);
yumiIK = inverseKinematics('RigidBodyTree', yumi);

Ts = 0.001;

for i=1:length(subs)
	task_i = subs(i);
	
	if logical(str2num(allocation(i)))
		execCobot = iiwaRBT;
        execIK = iiwaIK;
        execEEName = 'iiwa_link_ee_kuka';
	else
		execCobot = yumi;
        execIK = yumiIK;
        execEEName = 'gripper_l_base';        
    end
    execHomeConfig = homeConfiguration(execCobot);
    
    % Subtask execution

    % IK of picking position
    pickPosT = eul2tform(task_i.startPos(4:6));
    pickPosT(1:3,end) = task_i.startPos(1:3)';
    weights = ones(1,6);
    
    [pickConfig, pickSolInfo] = execIK(execEEName, pickPosT, weights, execHomeConfig);
    
    % IK of placing position
    placePosT = eul2tform(task_i.endPos(1,4:6));
    placePosT(1:3,end) = task_i.endPos(1,1:3)';
    
    [placeConfig, placeSolInfo] = execIK(execEEName, placePosT, weights, pickConfig);

    % IK of final home position
    placePosT = eul2tform(task_i.endPos(1,4:6));
    placePosT(1:3,end) = task_i.endPos(1,1:3)';
    
    [finalHConfig, finalHSolInfo] = execIK(execEEName, placePosT, weights, pickConfig);

    % Trajectory
    nJoints = length(execHomeConfig);
    
    
    clear traj;
    clear fakeTraj;
    for j=1:nJoints
        
        % Intermediate configurations
        homeq = execHomeConfig(j).JointPosition;
        pickq = pickConfig(j).JointPosition;
        placeq = placeConfig(j).JointPosition;
        positions = [homeq, pickq, placeq, homeq];
        if j == 1
            tk = getTimeDistrubution(positions, 'eq');
            tk = round(tk/Ts)*Ts*task_i.taskTime;
        end
        points = [
            tk          % Time Series
            positions   % Joint Configurations
            zeros(1, 4) % Joint Velocities
        ];
   
        traj(j) = multiPointImpV(points, Ts);
        
        %plotTrajectories(traj(j),tk);
    end
    
    if i == 1
        if execCobot == iiwaRBT
            iiwaTraj = traj;
            for k = 1:18
                points(2,:) = [0,0,0,0];
                yumiTraj(k) = multiPointImpV(points, Ts);
            end
        else
            yumiTraj = traj;
            for k = 1:7
                points(2,:) = [0,0,0,0];
                iiwaTraj(k) = multiPointImpV(points, Ts);
            end
        end
    else
        if execCobot == iiwaRBT
            for k = 1:18
                if k <= 7 
                    iiwaTraj(k) = mergeTrajectories([iiwaTraj(k),traj(k)]);
                end
                points(2,:) = [1,1,1,1];
                newTraj(k) = multiPointImpV(points, Ts);
                newTraj(k).q = ones(length(traj(1).q),1)*yumiTraj(k).q(length(yumiTraj(k).q));
                yumiTraj(k) = mergeTrajectories([yumiTraj(k), newTraj(k)]);
            end
        else
            for k = 1:18
                if k <= 7
                    points(2,:) = [1,1,1,1];
                    newTraj(k) = multiPointImpV(points, Ts);
                    newTraj(k).q = newTraj(k).q*iiwaTraj(k).q(length(iiwaTraj(k).q));
                    iiwaTraj(k) = mergeTrajectories([iiwaTraj(k), newTraj(k)]);
                end
                yumiTraj(k) = mergeTrajectories([yumiTraj(k),traj(k)]);
            end
            
        end
       
        %mergeTrajectories([iwaaTraj,traj]);
    end
   
    
    
    % Simulation
    % TODO
end
