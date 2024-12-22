mdl_puma560;

%% 定义GIF保存文件路径
gif_filename1 = 'puma_trajectory1_7.gif';  % 第一个GIF文件名
gif_filename2 = 'puma_trajectory2_7.gif';  % 第二个GIF文件名

%% 绘制第一个动画并保存为GIF
T1 = transl(0.5, -0.5, -0.5) * trotx(pi/2);
T2 = transl(0.5, 0.5,  0.5) * trotx(pi/2);
q1 = p560.ikine(T1);
q2 = p560.ikine(T2);
t = [0:0.05:2]';
q = jtraj(q1, q2, t);

figure;  % 创建一个新窗口
for i = 1:length(q)
    p560.plot(q(i, :));
    
    % 捕获当前帧
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);  % 将图像转换为索引图像
    
    % 将帧保存到GIF文件
    if i == 1
        imwrite(imind, cm, gif_filename1, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, gif_filename1, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
end


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

%% 绘制第二个动画并保存为GIF
% 笛卡尔坐标轨迹规划
T1 = transl(0.5, -0.5, -0.5) * trotx(pi/2);
T2 = transl(0.5, 0.5, 0.5) * trotx(pi/2);
t = [0:0.05:2]';
Ts = ctraj(T1, T2, length(t));
q = p560.ikine6s(Ts);

figure;  % 创建一个新窗口
for i = 1:length(q)
    p560.plot(q(i, :));
    
    % 捕获当前帧
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);  % 将图像转换为索引图像
    
    % 将帧保存到GIF文件
    if i == 1
        imwrite(imind, cm, gif_filename2, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, gif_filename2, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
end
