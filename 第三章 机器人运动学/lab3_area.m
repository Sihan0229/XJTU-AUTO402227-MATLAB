
% 加载PUMA560机器人模型
mdl_puma560;

% 生成关节角度范围，每个关节都有自己的限制
num_samples = 50000;  % 生成的样本数量
q = rand(num_samples, 6);  % 随机生成6个关节的角度配置
q(:, 1) = q(:, 1) * (p560.qlim(1, 2) - p560.qlim(1, 1)) + p560.qlim(1, 1);
q(:, 2) = q(:, 2) * (p560.qlim(2, 2) - p560.qlim(2, 1)) + p560.qlim(2, 1);
q(:, 3) = q(:, 3) * (p560.qlim(3, 2) - p560.qlim(3, 1)) + p560.qlim(3, 1);
q(:, 4) = q(:, 4) * (p560.qlim(4, 2) - p560.qlim(4, 1)) + p560.qlim(4, 1);
q(:, 5) = q(:, 5) * (p560.qlim(5, 2) - p560.qlim(5, 1)) + p560.qlim(5, 1);
q(:, 6) = q(:, 6) * (p560.qlim(6, 2) - p560.qlim(6, 1)) + p560.qlim(6, 1);

% 初始化末端执行器位置矩阵
workspace_points = zeros(num_samples, 3);

% 计算每个随机关节角度配置下的末端执行器位置
for i = 1:num_samples
    T = p560.fkine(q(i, :));  % 使用正运动学计算齐次变换矩阵
    workspace_points(i, :) = T.t';  % 提取末端执行器的位置
end

% 绘制工作空间的三维散点图
figure;
scatter3(workspace_points(:, 1), workspace_points(:, 2), workspace_points(:, 3), 1, 'filled');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('PUMA 560 Robot Workspace');
grid on;
axis equal;
