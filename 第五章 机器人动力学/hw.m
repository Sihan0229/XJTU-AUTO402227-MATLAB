%% 实验 1: PUMA560 正向动力学计算
clc; clear;

% 参数设置
T = [1 1 1 1 1 1];  % 关节扭矩 (Nm, 常量)
q = deg2rad([-60, 90, 30, 0, 0, 0]); % 初始关节角 (弧度)
dq = [0 0 0 0 0 0];   % 初始关节角速度 (弧度/秒)

% 定义 PUMA560 的 DH 参数
L(1) = Link('d', 0, 'a', 0, 'alpha', -pi/2);
L(2) = Link('d', 0, 'a', 0.4318, 'alpha', 0);
L(3) = Link('d', 0.15005, 'a', 0.0203, 'alpha', -pi/2);
L(4) = Link('d', 0.4318, 'a', 0, 'alpha', pi/2);
L(5) = Link('d', 0, 'a', 0, 'alpha', -pi/2);
L(6) = Link('d', 0, 'a', 0, 'alpha', 0);

% 创建机械臂模型
puma = SerialLink(L, 'name', 'PUMA560');

% 初始化关节加速度
ddq = zeros(1, 6); % 假设初始关节加速度为零

% 正向动力学计算
inertia_matrix = puma.inertia(q); % 获取惯性矩阵
coriolis_term = puma.coriolis(q, dq); % 获取科氏力矩阵
gravity_term = puma.gravload(q); % 获取重力项

for i = 1:length(T)
    ddq(i) = (T(i) - gravity_term(i) - coriolis_term(i, :) * dq') / inertia_matrix(i, i);
end

display(ddq);

tau = puma.rne(q, dq, ddq); % 再次计算扭矩验证

display(tau);