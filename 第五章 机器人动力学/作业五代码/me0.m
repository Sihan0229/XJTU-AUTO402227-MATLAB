function M = me0(L, m, Izz)
    % L: 连杆长度数组 (m)
    % m: 连杆质量数组 (kg)
    % Izz: 惯性矩数组 (kg·m^2)

    n = length(L); % 连杆数量
    M = zeros(n, n); % 初始化惯性矩阵
    
    for i = 1:n
        for j = 1:n
            if i >= j
                % 累加当前连杆及其后的质量贡献
                M(i, j) = sum(m(j:end)) * (L(i)^2 / 3) + Izz(i);
            end
        end
    end
end
