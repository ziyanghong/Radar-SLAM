function [poses_gt, radar_timestamps] = readGroudtruthPoses(sequence)
% Radar Groundtruth
timestamp_file = strcat(sequence, 'radar.timestamps');
M = dlmread(timestamp_file, ' ');



if contains(sequence,'MulRan') | contains(sequence,'Oxford')
    gt_file = strcat(sequence ,'gt/radar_odometry.csv');
elseif contains(sequence,'Boreas')
    gt_file = strcat(sequence ,'applanix/radar_poses.csv');
end
gt_odometry = readmatrix(gt_file);
x_initial = gt_odometry(1,2);
y_initial = gt_odometry(1,3);
heading_initial = gt_odometry(1,10);
pose_initial = [cos(heading_initial) -sin(heading_initial) x_initial;
                sin(heading_initial)  cos(heading_initial) y_initial;
                     0               0                1     ];
pose = eye(3);
poses_gt = zeros(3,3,size(gt_odometry,1)+1);
poses_gt(:,:,1) = eye(3);

for i = 1:size(gt_odometry,1)
    if contains(sequence,'MulRan') 
        delta_x = gt_odometry(i,3);
        delta_y = gt_odometry(i,4);
        delta_yaw = gt_odometry(i,8);
        RT = [cos(delta_yaw) -sin(delta_yaw) delta_x;
              sin(delta_yaw)  cos(delta_yaw) delta_y;
                   0               0                1     ];
        pose = pose * RT;
        
    elseif  contains(sequence,'Oxford')
        delta_x = gt_odometry(i,3);
        delta_y = -gt_odometry(i,4);
        delta_yaw = -gt_odometry(i,8);
        RT = [cos(delta_yaw) -sin(delta_yaw) delta_x;
              sin(delta_yaw)  cos(delta_yaw) delta_y;
                   0               0                1     ];
        pose = pose * RT;
        
    elseif contains(sequence,'Boreas')
        curr_utm_x = gt_odometry(i,2);
        curr_utm_y = gt_odometry(i,3);
        curr_utm_yaw = gt_odometry(i,10);
        pose_utm_curr = [cos(heading_initial) -sin(heading_initial) x_initial;
                         sin(heading_initial)  cos(heading_initial) y_initial;
                                      0               0                1     ];
        pose = inverse_pose(pose_initial)*pose_utm_curr;
    end


    poses_gt(:,:,i+1) = pose;
end

radar_timestamps = M(1:size(gt_odometry,1)+1,1);

end
