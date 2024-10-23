% 实验3.1 p79-81
%%
% 建立关节机器人模型1
% Revolute('d', d, 'a', a, 'alpha', alpha)：这是定义旋转关节的函数
% 'd'：关节的平移量，沿着 z 轴。
% 'a'：关节的偏距，沿着 x 轴的距离。
% 'alpha'：关节的扭转角，表示从一个关节到下一个关节沿 x 轴旋转的角度。
% 使用DH参数来定义每个关节
L(1) = Revolute('d', 0, 'a', 0, 'alpha', pi/2);
L(2) = Revolute('d', 0, 'a', 0.43, 'alpha', 0);
L(3) = Revolute('d', 0.15, 'a', 0.02, 'alpha', -pi/2);
L(4) = Revolute('d', 0.43, 'a', 0, 'alpha',pi/2);
L(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2);
L(6) = Revolute('d', 0, 'a', 0, 'alpha', 0);
% SerialLink 用于定义机器人，将多个关节串联起来形成一个连杆机构
% L 是关节数组
robot = SerialLink(L, 'name', 'PUMA560');
view(3);
% 绘制机器人在给定关节角度下的姿态
robot.plot([0 0 0 0 0 0]); % 绘制机器人在6个关节都为0角度时的姿态。

%%
% 建立关节机器人模型2
% Link([theta, d, a, alpha, sigma, offset])：定义机械臂中每个连杆或关节的参数。
% theta：旋转角度（用于旋转关节），这里为 0。
% d：平移距离。
% a：连杆的长度。
% alpha：连杆的扭转角度。
% sigma：0 表示旋转关节，1 表示平移关节。
% offset：用于对角度做补偿偏移。
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
% 输出 T 是一个 齐次变换矩阵，它描述了末端执行器在空间中的姿态。
%%
% 求解逆运动学
targetPos = [-0.06 -0.2 0.1]; % 定义目标末端执行器的位置，即[x, y, z]坐标。
targetform = rpy2tr(90, -180, -180); % rpy2tr函数将Roll-Pitch-Yaw角转换为齐次变换矩阵形式
TR = transl(targetPos)*targetform; % 结合了目标位置和平移变换，表示末端执行器的整体位姿（位置 + 方向）
q = robot.ikine(TR);
