% 实验3.2 p81-83
mdl_puma560;

%%
% 计算过程
% 计算两个目标位姿
% 结合了目标位置和平移变换，表示末端执行器的整体位姿（位置 + 方向）
T1 = transl(0, -0.25, 0) * trotx(pi/2);
T2 = transl(0.5, 0.25, 0.5) * trotx(pi/2);
% 逆运动学求解
q1 = p560.ikine(T1);
q2 = p560.ikine(T2);
% 使用关节轨迹生成工具
t = [0:0.05:2]'; % 时间向量
q = jtraj(q1, q2, t); % 生成从 q1 到 q2 的关节轨迹
p560.plot(q);
figure;

%%
% 计算关节角度变化
for i = 1:6
    subplot(3, 2, i);   % 创建 3x2 的布局，每次在第 i 个子图上绘制
    plot(t, q(:, i));   % 绘制第 i 列的 q 数据相对于 
    xlabel('Time/s');  % x 轴标签
    ylabel(['q' num2str(i) '/rad']);  % y 轴标签，显示 q 的列号
end

% 调整图像布局，使子图之间的间距更合理
tightfig;

%%
% 计算末端执行器的位置
ee_position = zeros(length(t), 3);  % 初始化末端执行器位置矩阵
for i = 1:length(t)
    T = p560.fkine(q(i, :));  % 通过正向运动学计算末端执行器的位姿
    ee_position(i, :) = transl(T);  % 提取末端执行器的平移部分（位置）
end

% 绘制末端执行器的运动轨迹
figure;
plot3(ee_position(:, 1), ee_position(:, 2), ee_position(:, 3), 'LineWidth', 2);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('End Effector Trajectory');



%%
% 笛卡尔坐标轨迹规划
T1 = transl(0, -0.25, 0) * trotx(pi/2);
T2 = transl(0.5, 0.25, 0.5) * trotx(pi/2);
t = [0:0.05:2]';
Ts = ctraj(T1,T2,length(t));
q = p560.ikine6s(Ts);
p560.plot(q);

%%
% 计算关节角度变化
for i = 1:6
    subplot(3, 2, i);   % 创建 3x2 的布局，每次在第 i 个子图上绘制
    plot(t, q(:, i));   % 绘制第 i 列的 q 数据相对于 
    xlabel('Time/s');  % x 轴标签
    ylabel(['q' num2str(i) '/rad']);  % y 轴标签，显示 q 的列号
end

% 调整图像布局，使子图之间的间距更合理
tightfig;

%%
% 计算末端执行器的位置
ee_position = zeros(length(t), 3);  % 初始化末端执行器位置矩阵
for i = 1:length(t)
    T1 = p560.fkine(q(i, :));  % 通过正向运动学计算末端执行器的位姿
    ee_position(i, :) = transl(T1);  % 提取末端执行器的平移部分（位置）
end

% 绘制末端执行器的运动轨迹
figure;
plot3(ee_position(:, 1), ee_position(:, 2), ee_position(:, 3), 'LineWidth', 2);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('End Effector Trajectory');