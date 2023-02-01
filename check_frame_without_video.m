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
init = 1;

% figure('Position', [0 0 500 500])
mkdir([prefix, suffix_folder, '/img/'])
frame_index = 0;
frame_rate = 100;
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

    if (is_first)
        base_R_imu_inertial = frameTransform(1:3,1:3) * ft_imu_transform(1:3,1:3)';
    end
    
    ft_imu_transform(1:3,1:3) = base_R_imu_inertial * ft_imu_transform(1:3,1:3);
    rpy_imu_temp = wbc.rollPitchYawFromRotation(ft_imu_transform(1:3,1:3));        
       
    imu_rpy_log = [imu_rpy_log, rpy_imu_temp];
    fk_rpy_log = [fk_rpy_log,  wbc.rollPitchYawFromRotation(frameTransform(1:3,1:3))];

%     error = [error, norm(wbc.skewVee(ft_imu_transform(1:3,1:3)' * frameTransform(1:3,1:3)))];

    is_first = 0;
end

% cmd = sprintf('ffmpeg -framerate %d -pattern_type glob -i "%s*.png" -c:v libx264 -pix_fmt yuv420p %s/out.mp4', ...
%               frame_rate, [prefix, suffix_folder, '/img/'], [prefix, suffix_folder]);
% system(cmd);
% close all;
