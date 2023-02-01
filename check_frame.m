robotName='iCubGenova09'; %% Name of the robot

meshFilePrefix = [getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX') '/share']; %% Path to the model meshes
modelPath = [getenv('ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX') '/share/iCub/robots/' robotName '/'];  %% Path to the robot model

fileName='model.urdf'; %% Name of the urdf file
gravityAcceleration = 9.80665;

world_H_base = eye(4);
KinDynModel = iDynTreeWrappers.loadReducedModel(robot_logger_device.description_list, 'root_link', modelPath, fileName, false);
samples = robot_logger_device.orientations.l_foot_rear_ft_eul.dimensions(3);

frame_name = [prefix, '_ft_sensor'];
ft_name = [prefix, '_ft_eul'];
acc_name = [prefix, '_ft_acc'];

display = 1;
is_first=1;

suffix_folder = ['_', dataset_name];
if (correct_yaw)
    suffix_folder = [suffix_folder, '_with_yaw_correction'];
end

error = [];
imu_rpy_log = [];
fk_rpy_log = [];
init = 10;

% figure('Position', [0 0 500 500])
mkdir([prefix, suffix_folder, '/img/'])
frame_index = 0;
frame_rate = 2;
for i = init: (100 * 1 / frame_rate) :samples
    i
    s = robot_logger_device.joints_state.positions.data(:,1,i);
    ds = robot_logger_device.joints_state.velocities.data(:,1,i);
    imu_rpy = robot_logger_device.orientations.(ft_name).data(:,1,i);
    acc = robot_logger_device.accelerometers.(acc_name).data(:,1,i)/9.81;
    iDynTreeWrappers.setRobotState(KinDynModel,s,ds,[0,0,-gravityAcceleration]);

    frameTransform = iDynTreeWrappers.getWorldTransform(KinDynModel, frame_name);
    ft_imu_transform = iDynTreeWrappers.getWorldTransform(KinDynModel, frame_name);
    ft_imu_transform(1:3,1:3) = wbc.rotationFromRollPitchYaw(imu_rpy);

    or = frameTransform(1:3, 4);
    acc = frameTransform * [acc; 1];

    rpy_fk_temp = wbc.rollPitchYawFromRotation(frameTransform(1:3,1:3));

    if (is_first)
        base_R_imu_inertial = frameTransform(1:3,1:3) * ft_imu_transform(1:3,1:3)';
        [visualizer,objects]=iDynTreeWrappers.prepareVisualization(KinDynModel,meshFilePrefix, ...
            'transparency',0.1, ...
            'name', 'Imu Test', ...
            'reuseFigure', 'name');
        forward_kin_frame = iDynTreeWrappers.plotFrame(frameTransform, 0.2, 10);

%         v = [50 20];
%         view(v);

        % reset imu
        ft_imu_transform(1:3,1:3) = [base_R_imu_inertial * ft_imu_transform(1:3,1:3)];
        rpy_imu_temp = wbc.rollPitchYawFromRotation(ft_imu_transform(1:3,1:3));
        rpy_imu_temp(3) = rpy_fk_temp(3);
        ft_imu_transform(1:3,1:3) = wbc.rotationFromRollPitchYaw(rpy_imu_temp);

        imu_frame = iDynTreeWrappers.plotFrame(ft_imu_transform, 0.4, 5);

        set(gcf, 'WindowState', 'maximized');
        xlim([-0.8, 0.8])
        ylim([-0.8, 0.8])
        zlim([-1, 0.8])
        title(ft_name, 'interpreter', 'none');

        %% Plot of accelerometer
%         gravity = plot3([or(1) acc(1)], [or(2) acc(2)], [or(3) acc(3)], 'm', 'linewidth', 7);
    else
        ft_imu_transform(1:3,1:3) = [base_R_imu_inertial * ft_imu_transform(1:3,1:3)];
        if (correct_yaw)
                rpy_imu_temp = wbc.rollPitchYawFromRotation(ft_imu_transform(1:3,1:3));
                rpy_imu_temp(3) = rpy_fk_temp(3);
                ft_imu_transform(1:3,1:3) = wbc.rotationFromRollPitchYaw(rpy_imu_temp);
        end
        iDynTreeWrappers.updateFrame(forward_kin_frame, frameTransform);
        iDynTreeWrappers.updateFrame(imu_frame, ft_imu_transform);
%         set(gravity, 'XData', [or(1) acc(1)], 'YData', [or(2) acc(2)], 'ZData', [or(3) acc(3)]);
        iDynTreeWrappers.updateVisualization(KinDynModel,visualizer);
    end
    imu_rpy_log = [imu_rpy_log, rpy_imu_temp];
    fk_rpy_log = [fk_rpy_log,  wbc.rollPitchYawFromRotation(frameTransform(1:3,1:3))];

    error = [error, norm(wbc.skewVee(ft_imu_transform(1:3,1:3)' * frameTransform(1:3,1:3)))];
    drawnow()
    if (is_first)
        pause(10)
    end

    imwrite(getframe(gca).cdata, fullfile([prefix, suffix_folder, '/img/'], sprintf('%06d.png', frame_index)));
    frame_index = frame_index + 1;

    is_first = 0;
end

cmd = sprintf('ffmpeg -framerate %d -pattern_type glob -i "%s*.png" -c:v libx264 -pix_fmt yuv420p %s/out.mp4', ...
              frame_rate, [prefix, suffix_folder, '/img/'], [prefix, suffix_folder]);
system(cmd);
close all;
