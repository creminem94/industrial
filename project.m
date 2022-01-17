clc; clear all; close all;

yumiFrame = [
-1 0 0 1
0 -1 0 0
0 0 1 0
0 0 0 1
];

iiwaFrame = eye(4);

% Cobot models
yumiRBT = loadrobot('abbYuMi');
iiwaRBT = loadrobot('kukaIiwa14');

% Inverse kinematics of cobots
iiwaIK = inverseKinematics('RigidBodyTree', iiwaRBT);
yumiIK = inverseKinematics('RigidBodyTree', yumiRBT);

iiwaDof = 7;
yumiDof = 18;

pCenter = [0.5, 0, 0.2, 0, pi, 0]; % position of center
pPick = [0.9,0.6, 0.2, 0, pi, 0]; %position of pick 
pCenterHold = [0.5, 0.2, 0.2, 0, 0, pi/2]; % position of center but from right orientation
pTrans = [0.5, 0.3, 0.4, 0, pi, 0];
piiwaHalf = [0.5, 0, 0.7, 0, pi, 0];
piiwaHome = [0 0 1.306 0 0 0];

yumiPoints = [pTrans;pPick;pTrans;pCenter;pTrans;pPick;pTrans;pCenter;pCenterHold;pCenterHold];
iiwaPoints = [piiwaHome;piiwaHome;piiwaHome;piiwaHome;piiwaHome;piiwaHome;piiwaHome;piiwaHome;piiwaHalf;pCenter];

time = 25;
Ts = 0.001;
weights = ones(1,6);
% yumiTraj.q = [];
% yumiTraj.t = [];
% iiwaTraj.q = [];
% iiwaTraj.t = [];
robots = [yumiRBT; iiwaRBT];
for i = 1:length(robots)
    figure(i);
    if robots(i) == iiwaRBT
        execCobot = iiwaRBT;
        execIK = iiwaIK;
        execEEName = 'iiwa_link_ee_kuka';
        frame = iiwaFrame;
        execDof = iiwaDof;
        execPoints = iiwaPoints;
        sgtitle('iiwa joints');
    else
        execCobot = yumiRBT;
        execIK = yumiIK;
        execEEName = 'gripper_r_base';   
        frame = yumiFrame;
        execDof = yumiDof;
        execPoints = yumiPoints;
        sgtitle('Yumi joints');
    end
    execHomeConfig = homeConfiguration(execCobot);
    
    prevConfig = execHomeConfig;
    nPoints = length(execPoints(:,1));
    jointPoints = zeros(execDof,nPoints); 
    for k = 1:nPoints
        x = execPoints(k, :);%x,y,z,phi,theta,psi
        posT = eul2tform(x(4:6));
        posT(1:3,end) = x(1:3)';
        posT = frame*posT;
        [config, solInfo] = execIK(execEEName, posT, weights, prevConfig);
        prevConfig = config;
        jointPoints(:,k) = [config.JointPosition]';
    end
    
    for j = 1:execDof 
        positions = jointPoints(j,:);
        tk = getTimeDistrubution(positions, 'eq');
        tk = round(tk/Ts)*Ts*time;
        points = [
            tk         
            positions   
            zeros(1, length(positions)) 
        ];
        if robots(i) == iiwaRBT
            subplot(3, 3, j);
            iiwaTraj(j) = multiPointImpV(points, Ts);
            plot(iiwaTraj(j).t, iiwaTraj(j).q, 'LineWidth', 2);
            title(strcat('joint ', num2str(j)));
            xlabel('time');
            ylabel('rad');
        else
            yumiTraj(j) = multiPointImpV(points, Ts);
            if j >= 10 && j<=16
                subplot(3, 3, j-9);
                plot(yumiTraj(j).t, yumiTraj(j).q, 'LineWidth', 2);
                title(strcat('joint ', num2str(j)));
                xlabel('time');
                ylabel('rad');
            end
            
        end
    end
        
end
yumiTraj(1).q = ones(1, length(yumiTraj(1).q))*pi;
%%
open('projectSimscape.slx');
sim('projectSimscape.slx', time+5);

