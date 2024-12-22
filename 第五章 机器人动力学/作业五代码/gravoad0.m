function G = gravoad0(L, m, theta, g)
    % L: 连杆长度数组 (m)
    % m: 连杆质量数组 (kg)
    % theta: 关节角度数组 (rad)
    % g: 重力加速度 (m/s^2)

    n = length(L); % 连杆数量
    G = zeros(n, 1); % 初始化重力矩阵
    
    for i = 1:n
        % 累加每个连杆的重力影响
        G(i) = sum(m(i:end)) * g * (L(i) / 2) * cos(theta(i));
    end
end
