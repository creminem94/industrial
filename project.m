%% this is used to generate the subsystem in simscape
yumi = loadrobot('abbYuMi');
% yumiSM = smimport(yumi);
iiwaRBT = loadrobot('kukaIiwa14');
% iiwaSM = smimport(iiwaRBT);

%% list of subtasks
P1 = [0 0 0]; % minipallet
P2 = [2 3 5]; % vicino al kuka
P3 = [10 10 10]; % vicino all'abb
P4 = [1 1 1]; % vicino al kuka
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
