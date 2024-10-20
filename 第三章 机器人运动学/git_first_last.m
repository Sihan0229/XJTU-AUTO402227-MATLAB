% 定义 GIF 文件名和保存图片的文件名
gif_filename = 'puma_trajectory2_0.gif'; % 请替换为你的 GIF 文件名
first_frame_filename = 'puma_trajectory2_0_first_frame.png'; % 第一帧保存为的文件名
last_frame_filename = 'puma_trajectory2_0_last_frame.png'; % 最后一帧保存为的文件名

% 读取 GIF 文件
info = imfinfo(gif_filename); % 获取 GIF 文件的信息
num_frames = numel(info); % 获取帧的数量

% 读取第一帧
[first_frame, cmap1] = imread(gif_filename, 1); % 读取第一帧及其调色板
first_frame_rgb = ind2rgb(first_frame, cmap1); % 转换为 RGB 图像
imwrite(first_frame_rgb, first_frame_filename); % 保存第一帧为 PNG 文件

% 读取最后一帧
[last_frame, cmap2] = imread(gif_filename, num_frames); % 读取最后一帧及其调色板
last_frame_rgb = ind2rgb(last_frame, cmap2); % 转换为 RGB 图像
imwrite(last_frame_rgb, last_frame_filename); % 保存最后一帧为 PNG 文件

disp('第一帧和最后一帧已保存为 RGB 图像。');

