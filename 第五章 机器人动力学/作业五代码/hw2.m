L = [4, 3, 2]; % 连杆长度 (m)
m = [20, 15, 10]; % 连杆质量 (kg)
Izz = [0.5, 0.2, 0.1]; % 惯性矩 (kg·m^2)
g = 9.81; % 重力加速度 (m/s^2)
theta = deg2rad([10, 20, 30]); % 关节角度 (rad)
theta_dot = [1, 2, 3]; % 关节速度 (rad/s)
theta_ddot = [0.5, 1, 1.5]; % 关节加速度 (rad/s^2)

% 惯性矩阵 M(theta)
M = zeros(3);
for i = 1:3
    for j = 1:3
        if i >= j
            M(i, j) = sum(m(j:end)) * L(i)^2 / 3 + Izz(i);
        end
    end
end

% 科氏力和离心力矩阵 C(theta, theta_dot)
C = zeros(3);
for i = 1:3
    for j = 1:3
        for k = 1:3
            C(i, j) = C(i, j) + m(k) * L(k) * L(j) * sin(theta(k)) * theta_dot(k);
        end
    end
end

% 重力矩阵 G(theta)
G = zeros(3, 1);
for i = 1:3
    G(i) = sum(m(i:end)) * g * (L(i) / 2) * cos(theta(i));
end

% 计算驱动力矩 tau
tau = M * theta_ddot' + C * theta_dot' + G;

% 输出结果
disp('关节驱动力矩 tau (N·m):');
disp(tau);

disp('惯性矩阵 M:');
disp(M);

disp('重力矩阵 G:');
disp(G);

% 验证结果：利用 MATLAB 函数
% 调用惯性矩验证函数
M_calculated = me0(L, m, Izz);

disp('验证惯性矩阵 M:');
disp(M_calculated);

% 调用重力矩验证函数
G_calculated = gravoad0(L, m, theta, g);

disp('验证重力矩阵 G:');
disp(G_calculated);

