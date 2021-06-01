%% this is used to generate the subsystem in simscape
yumi = importrobot('abbyumi.urdf');
yumiSM = smimport(yumi);
iiwaRBT = importrobot('iiwa14.urdf');
iiwaSM = smimport(iiwaRBT);