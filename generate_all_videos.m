
links = {'r_leg', 'r_foot_front', 'r_foot_rear'};
% datasets = {'2023_01_18_16_39_51', '2023_01_12_13_16_07'};
datasets = {'2023_01_12_13_16_07'};
for dataset = datasets
dataset_name = dataset{:};
load([dataset_name, '.mat']);
for link = links
    prefix = link{:};
%     correct_yaw = 1;
%     check_frame;
    correct_yaw = 0;
    check_frame_without_video;


   frame = [link{:}, '_ft_eul'];
     suffix_folder = ['_', dataset_name];
 mkdir([prefix, suffix_folder])
% 
fig = figure('Renderer', 'painters', 'Position', [0 0 1200 600]);
time = squeeze(robot_logger_device.orientations.(frame).timestamps);
plot(time -time(1), 180/pi * imu_rpy_log')
hold on;
plot(time -time(1), 180/pi * fk_rpy_log')
title = replace(prefix, '_', ' ');

plot_aesthetic(title, 'time (s)', 'angle (deg)' , '', ...
    '$roll_{imu}$', '$pitch_{imu}$', '$yaw_{imu}$', ...
    '$roll_{fk}$', '$pitch_{fk}$', '$yaw_{fk}$')
saveas(fig, [prefix, suffix_folder, '/rpy_imu_fk.png']);
% close all;
end
end