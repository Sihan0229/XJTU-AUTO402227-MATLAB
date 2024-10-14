% 实验3.1 p79-81
%%
% 建立关节机器人模型1
% 使用DH参数来定义每个关节
L(1) = Revolute('d', 0, 'a', 0, 'alpha', pi/2);
L(2) = Revolute('d', 0, 'a', 0.43, 'alpha', 0);
L(3) = Revolute('d', 0.15, 'a', 0.02, 'alpha', -pi/2);
L(4) = Revolute('d', 0.43, 'a', 0, 'alpha',pi/2);
L(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2);
L(6) = Revolute('d', 0, 'a', 0, 'alpha', 0);
robot = SerialLink(L, 'name', 'PUMA560');
view(3);
robot.plot([0 0 0 0 0 0]); % 绘制机器人在6个关节都为0角度时的姿态。

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
theta = [pi/2 0 pi/2 0 0 0]; % 定义6个关节的角度配置，单位是弧度。
T = robot.fkine(theta); % 计算给定关节角度下的末端执行器的齐次变换矩阵T，描述末端执行器的位姿（位置和方向）。

%%
% 求解逆运动学
targetPos = [-0.06 -0.2 0.1]; % 定义目标末端执行器的位置，即[x, y, z]坐标。
targetform = rpy2tr(90, -180, -180); % rpy2tr函数将Roll-Pitch-Yaw角转换为齐次变换矩阵形式
TR = transl(targetPos)*targetform;
q = robot.ikine(TR);
