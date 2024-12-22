clear;
clc;% 改进D-H
th(1) = 0; d(1) = 0; a(1) = 0; alp(1) = 0;
th(2) = 0; d(2) = 0; a(2) = 4; alp(2) = 0;
th(3) = 0; d(3) = 0; a(3) = 3; alp(3) = 0;
% DH parameters th d a alpha
L1 = Link([th(1), d(1), a(1), alp(1)], 'modified');
L2 = Link([th(2), d(2), a(2), alp(2)], 'modified');
L3 = Link([th(3), d(3), a(3), alp(3)], 'modified');
L1.m = 20; L2.m = 15; L3.m = 10;% 连杆质量
% 连杆质心位置
L1.r = [2 0 0];
L2.r = [1.5 0 0];
L3.r = [1 0 0];
L1.Jm = 0; L2.Jm = 0; L3.Jm = 0;
% 连杆惯性张量
L1.I = [0 0 0; 0 0 0; 0 0 0.5];
L2.I = [0 0 0; 0 0 0; 0 0 0.2];
L3.I = [0 0 0; 0 0 0; 0 0 0.1];
robot = SerialLink([L1, L2, L3]);
robot.name = 'Plan3R';
view(3)
robot.display()
% Forward Pose Kinematics
theta=[10, 20, 30]*pi/180;
robot.plot(theta);
t0=robot.fkine(theta) %末端执行器位姿
%% 动力学逆解
% TAU = R.rne(Q, QD, QDD, GRAV)
tau = robot.rne(theta, [1, 2, 3], [0.5, 1, 1.5], [0 9.8 0])