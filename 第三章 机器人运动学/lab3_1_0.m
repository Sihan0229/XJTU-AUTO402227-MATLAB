% 定义DH参数的关节参数
DH_params = [
    0      0      0    pi/2;
    0      0.43   0    0;
    0.15   0.02   0   -pi/2;
    0.43   0      0    pi/2;
    0      0      0   -pi/2;
    0      0      0    0
];

% 输入各关节角度 theta
theta = [pi/2 0 pi/2 0 0 0];

% 计算每个关节的齐次变换矩阵
T = eye(4);  % 初始化为单位矩阵
for i = 1:6
    d = DH_params(i, 1);
    a = DH_params(i, 2);
    alpha = DH_params(i, 3);
    theta_i = theta(i);  % 当前关节的旋转角度

    % 计算DH转换矩阵
    A = [cos(theta_i), -sin(theta_i)*cos(alpha), sin(theta_i)*sin(alpha), a*cos(theta_i);
         sin(theta_i), cos(theta_i)*cos(alpha), -cos(theta_i)*sin(alpha), a*sin(theta_i);
         0,            sin(alpha),               cos(alpha),              d;
         0,            0,                        0,                       1];

    % 累乘得到末端执行器的齐次变换矩阵
    T = T * A;
end

% 输出末端执行器的位姿 (齐次变换矩阵)
disp('End-effector pose:');
disp(T);

%% 逆运动学 (粗略实现)
% 假设我们要通过给定末端执行器的位置和姿态求解关节角度
targetPos = [-0.06 -0.2 0.1];  % 目标末端执行器位置
targetRot = rpy2rot([90, -180, -180]);  % Roll-Pitch-Yaw 转换成旋转矩阵

% 构建目标齐次变换矩阵 TR
TR = [targetRot, targetPos';
      0 0 0 1];

% 逆运动学求解 (这里我们做个简单示例，假设部分关节角度可以通过几何关系推导)
% 对于PUMA 560, 通常我们用解析方法求解逆运动学, 假设已知的几何推导:

% 关节1 (theta1) 由末端的x, y坐标决定:
theta1 = atan2(targetPos(2), targetPos(1));

% 关节2,3 (theta2, theta3) 可以通过解三角形来推导 (略去详细过程)
% 使用关节2和关节3的几何关系推导求解，假设为以下值
theta2 = pi/4;
theta3 = pi/6;

% 其他关节角度可以通过姿态的旋转矩阵反推 (根据具体机器人的参数和配置)
theta4 = 0;  % 这里简化为0
theta5 = 0;
theta6 = 0;

% 输出逆运动学求解的关节角度
disp('Inverse Kinematics Solution:');
disp([theta1, theta2, theta3, theta4, theta5, theta6]);

%% 工具函数: Roll-Pitch-Yaw 转换为旋转矩阵
function R = rpy2rot(rpy)
    % 将Roll, Pitch, Yaw 转换为旋转矩阵
    roll = deg2rad(rpy(1));
    pitch = deg2rad(rpy(2));
    yaw = deg2rad(rpy(3));

    % ZYX顺序旋转矩阵
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw), cos(yaw), 0;
          0, 0, 1];

    Ry = [cos(pitch), 0, sin(pitch);
          0, 1, 0;
          -sin(pitch), 0, cos(pitch)];

    Rx = [1, 0, 0;
          0, cos(roll), -sin(roll);
          0, sin(roll), cos(roll)];

    % 组合得到旋转矩阵
    R = Rz * Ry * Rx;
end
