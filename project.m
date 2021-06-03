%% this is used to generate the subsystem in simscape
clc; clear all; close all;

yumi = loadrobot('abbYuMi');
% yumiSM = smimport(yumi);
iiwaRBT = loadrobot('kukaIiwa14');
% iiwaSM = smimport(iiwaRBT);

%% list of subtasks
P1 = [1, 1, 1, pi, 0, pi/2]; % minipallet
P2 = [2, 3, 5, 0, 0, pi/2]; % vicino al kuka
P3 = [10, 10, 10, pi/4, pi/4, 0]; % vicino all'abb
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
    subtask(P1,P2,'move',0.1,0.3,0.1,0.3)
    subtask(P1,P3,'move',0.5,0.1,0.4,0.1)
    subtask(P4,P1,'assembly',0.1,0.6,0.7,0.1)
    subtask(P3,P1,'assembly',0.5,0.1,0.7,0.1)
];
allocation = bestAllocation(subs);

%% Tasks Simulation
nTasks = length(subs);

iiwaIK = inverseKinematics('RigidBodyTree', iiwaRBT);
yumiIK = inverseKinematics('RigidBodyTree', yumi);

for i=1:1
	task_i = subs(i);
	
	if logical(str2num(allocation(i)))
		execCobot = iiwaRBT;
        execIK = iiwaIK;
	else
		execCobot = yumi;
        execIK = yumiIK;
    end
    execHome = homeConfiguration(execCobot);
    execEEName = string(execCobot.BodyNames(end));
        
    % Subtask execution

    % IK of picking position
    pickPosT = eul2tform(task_i.startPos(4:6));
    pickPosT(1:3,end) = task_i.startPos(1:3)';
    weights = ones(1,6);
    
    [pickConfig, pickSolInfo] = execIK(execEEName, pickPosT, weights, execHome);
    
    % IK of placing position
    placePosT = eul2tform(task_i.endPos(1,4:6));
    placePosT(1:3,end) = task_i.endPos(1,1:3)';
    
    [placeConfig, placeSolInfo] = execIK(execEEName, placePosT, weights, pickConfig);

end