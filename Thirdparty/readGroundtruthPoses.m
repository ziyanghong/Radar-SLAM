function [all_poses, all_relative_poses,timestamps,gps] = readGroundtruthPoses(path)
all_poses = [];
all_relative_poses = [];

if contains(path,'Oxford_Radar_RobotCar_Dataset')
    timestamps = readmatrix(strcat(path,'Navtech_Polar.txt'));
    gps = readmatrix(strcat(path,'gps/gps.csv'));
    gt_file = strcat(path,'gt/radar_odometry.csv');
    gt = readmatrix(gt_file);
    translation_xs = gt(1:size(gt,1),3);
    translation_ys = gt(1:size(gt,1),4);
    rotation_yaws = -gt(1:size(gt,1),8);
    RT_gt = eye(3,3);
    last_pose_gt = eye(3,3);
    all_poses = [all_poses;last_pose_gt];
    all_relative_poses = [all_relative_poses; {RT_gt}];
    for i = 1:size(gt,1)
        
        % gt pose
        RT_gt = [cos(rotation_yaws(i)) -sin(rotation_yaws(i)) translation_xs(i); sin(rotation_yaws(i)) cos(rotation_yaws(i)) translation_ys(i); 0 0 1];
        current_pose_gt = last_pose_gt*RT_gt;
        all_poses = [all_poses;{current_pose_gt}];
        all_relative_poses = [all_relative_poses; {RT_gt}];

        last_pose_gt = current_pose_gt;
    end
end


end