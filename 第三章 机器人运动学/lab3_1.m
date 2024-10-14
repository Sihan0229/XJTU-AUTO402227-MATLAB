% 实验3.1 p79-81
%%
% 建立关节机器人模型1
L(1) = Revolute('d', 0, 'a', 0, 'alpha', pi/2);
L(2) = Revolute('d', 0, 'a', 0.43, 'alpha', 0);
L(3) = Revolute('d', 0.15, 'a', 0.02, 'alpha', -pi/2);
L(4) = Revolute('d', 0.43, 'a', 0, 'alpha',pi/2);
L(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2);
L(6) = Revolute('d', 0, 'a', 0, 'alpha', 0);
robot = SerialLink(L, 'name', 'PUMA560');
view(3);
robot.plot([0 0 0 0 0 0]);

%%
% 建立关节机器人模型2
L1 = Link([0 0 0 pi/2 0 0]);
L2 = Link([0 0 0.43 0 0 0]);
L3 = Link([0 0.15 0.12 -pi/2 0 0]);
L4 = Link([0 0.43 0 pi/2 0 0]);
L5 = Link([0 0 0 -pi/2 0 0]);
L6 = Link([0 0 0 0 0 0]);
robot = SerialLink([L1, L2, L3, L4, L5, L6], 'name', 'PUMA560');
view(3);
robot.plot([0 0 0 0 0 0])

%%
% 求解正运动学
theta = [pi/2 0 pi/2 0 0 0];
T = robot.fkine(theta);

%%
% 求解逆运动学
targetPos = [-0.06 -0.2 0.1];
targetform = rpy2tr(90, -180, -180);
TR = transl(targetPos)*targetform;
q = robot.ikine(TR);
