mdl_puma560;

%% 定义初始姿态T1 (手动构造)
T1 = [  0, 0, 1, 0;  % Z轴方向是1，表示末端执行器绕X轴旋转90度（trotx(pi/2)）
        1, 0, 0, -0.25;  % 平移部分为[x, y, z] = [0, -0.25, 0]
        0, 1, 0, 0;
        0, 0, 0, 1];

%% 定义目标姿态T2 (手动构造)
T2 = [  0, 0, 1, 0.5;  % 平移到目标位置 [0.5, 0.25, 0.5]
        1, 0, 0, 0.25;
        0, 1, 0, 0.5;
        0, 0, 0, 1];

%% 时间采样
t = [0:0.05:2]';  % 采样时间数组
n = length(t);  % 采样步数

%% 手动进行线性插值来生成轨迹
Ts = zeros(4,4,n);  % 初始化轨迹矩阵
for i = 1:n
    alpha = (i-1) / (n-1);  % 线性插值系数，0到1
    Ts(:,:,i) = (1 - alpha) * T1 + alpha * T2;  % 在T1和T2之间插值
    Ts(4,4,i) = 1;  % 保持齐次变换矩阵的最后一行为[0 0 0 1]
end

%% 手动求解逆运动学
% 这里只针对每个时刻的变换矩阵，手动进行逆运动学解算。为简化，我们可以使用一个逆运动学的迭代方法。

q = zeros(n, 6);  % 关节角度数组初始化
for i = 1:n
    T_current = Ts(:,:,i);  % 获取当前时刻的目标位姿
    q(i,:) = ikine_manual(p560, T_current);  % 使用自定义的逆运动学函数进行求解
end

%% 绘制PUMA560的运动
p560.plot(q);

%% 计算关节角度变化
figure;
for i = 1:6
    subplot(3, 2, i);   % 创建 3x2 的布局，每次在第 i 个子图上绘制
    plot(t, q(:, i));   % 绘制第 i 列的 q 数据相对于时间的变化
    xlabel('Time/s');  % x 轴标签
    ylabel(['q' num2str(i) '/rad']);  % y 轴标签，显示关节编号
end

%% 计算末端执行器的位置（手动计算正运动学）
ee_position = zeros(n, 3);  % 初始化末端执行器位置矩阵
for i = 1:n
    T_current = Ts(:,:,i);  % 获取当前的齐次变换矩阵
    ee_position(i, :) = T_current(1:3, 4)';  % 提取末端执行器的平移部分（位置）
end

%% 绘制末端执行器的运动轨迹
figure;
plot3(ee_position(:, 1), ee_position(:, 2), ee_position(:, 3), 'LineWidth', 2, 'Color', 'b');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('End Effector Trajectory');
grid on;

