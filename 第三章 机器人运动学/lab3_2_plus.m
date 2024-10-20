mdl_puma560;

%% 定义障碍物：立方体区域
obstacle_center = [0.25, 0, 0.25];  % 障碍物中心点
obstacle_size = 0.2;  % 立方体边长
% 绘制障碍物
figure;
[x, y, z] = meshgrid(obstacle_center(1) + [-obstacle_size/2, obstacle_size/2], ...
                     obstacle_center(2) + [-obstacle_size/2, obstacle_size/2], ...
                     obstacle_center(3) + [-obstacle_size/2, obstacle_size/2]);
fill3(x(:), y(:), z(:), 'r', 'FaceAlpha', 0.3); % 画出立方体障碍物
hold on;

%% 轨迹规划：避开障碍物
T1 = transl(0, -0.25, 0) * trotx(pi/2);
T2 = transl(0.5, 0.25, 0.5) * trotx(pi/2);

% 在障碍物前插入一个新的目标点以避免碰撞
T_avoid = transl(0.25, -0.25, 0.5) * trotx(pi/2); % 添加的避障点

% 生成新的路径，先从T1到T_avoid，再从T_avoid到T2
t = [0:0.05:2]';
Ts1 = ctraj(T1, T_avoid, length(t)/2);  % 第一个段轨迹
Ts2 = ctraj(T_avoid, T2, length(t)/2);  % 第二段轨迹

% 合并两段轨迹
Ts = cat(3, Ts1, Ts2);

% 计算逆运动学
q = p560.ikine6s(Ts);

% 绘制机器人运动
p560.plot(q);
hold on;

%% 绘制轨迹和障碍物
% 计算末端执行器的位姿
ee_position = zeros(size(Ts, 3), 3);  % 初始化末端执行器位置矩阵
for i = 1:size(Ts, 3)
    T = Ts(:, :, i);  % 获取当前的齐次变换矩阵
    ee_position(i, :) = transl(T);  % 提取末端执行器的平移部分（位置）
end

% 绘制末端执行器的运动轨迹
plot3(ee_position(:, 1), ee_position(:, 2), ee_position(:, 3), 'LineWidth', 2, 'Color', 'b');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('End Effector Trajectory with Obstacle Avoidance');
grid on;

% 绘制障碍物（立方体）
fill3([obstacle_center(1)-obstacle_size/2, obstacle_center(1)+obstacle_size/2, ...
       obstacle_center(1)+obstacle_size/2, obstacle_center(1)-obstacle_size/2], ...
      [obstacle_center(2)-obstacle_size/2, obstacle_center(2)-obstacle_size/2, ...
       obstacle_center(2)+obstacle_size/2, obstacle_center(2)+obstacle_size/2], ...
      [obstacle_center(3)-obstacle_size/2, obstacle_center(3)-obstacle_size/2, ...
       obstacle_center(3)-obstacle_size/2, obstacle_center(3)-obstacle_size/2], ...
      'r', 'FaceAlpha', 0.3);

%% 计算关节角度变化
figure;
for i = 1:6
    subplot(3, 2, i);   % 创建 3x2 的布局，每次在第 i 个子图上绘制
    plot(t, q(:, i));   % 绘制第 i 列的 q 数据相对于时间的变化
    xlabel('Time/s');  % x 轴标签
    ylabel(['q' num2str(i) '/rad']);  % y 轴标签，显示关节编号
end
