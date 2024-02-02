function odomParameterStudy(sequence, ro_folder, parameters, xlab_str)

%% Pure odometry
ro_result_path = strcat(sequence, ro_folder);
lengths = [100,200,300,400,500,600,700,800];

% Radar Groundtruth
[poses_gt, radar_timestamps] = readGroudtruthPoses(sequence);
x = reshape(poses_gt(1,3,:),[1 size(poses_gt,3)]);
y = reshape(poses_gt(2,3,:),[1 size(poses_gt,3)]);
gt_x = x;
gt_y = y;


Files = dir(ro_result_path);
vError_t_odom = [];
vError_r_odom = [];
figure;



for f = 3:size(Files,1)

    ro_file = strcat(ro_result_path, '/', Files(f).name ,'/our_result_odometry.csv')
    ro = readmatrix(ro_file);
    ro = sortrows(ro);
    ro_timestamps = radar_timestamps(ro(:,1));
    yaws_ro = ro(:,4)';
    
    poses_ro = zeros(3,3,size(ro,1));
    for i = 1:size(ro,1)
        poses_ro(:,:,i) = [cos(yaws_ro(i)) -sin(yaws_ro(i)) ro(i,2);
                           sin(yaws_ro(i))  cos(yaws_ro(i)) ro(i,3);
                              0                0                   1   ] ;
                          
        
    end
    
    
    
    [sync_ro_poses, gt_sync_poses] = syncPoses(poses_gt, radar_timestamps, poses_ro, ro_timestamps);

    vErrors_odom = calcSequenceErrors(gt_sync_poses, sync_ro_poses,lengths);
    rotation_error = mean(vErrors_odom(:,2));
    translation_error = mean(vErrors_odom(:,3));
    vError_r_odom = [vError_r_odom; rotation_error];
    vError_t_odom = [vError_t_odom; translation_error];
    
    
    %% Plot each trajectory
%     x = reshape(poses_ro(1,3,:),[1 size(ro,1)]);
%     y = reshape(poses_ro(2,3,:),[1 size(ro,1)]);
%     figure;
%     hold on;
%     plot(gt_x, gt_y, 'LineWidth',2);
%     plot(x,y,'LineWidth',2);
%     hold off;
%     lgd = legend('groundtruth', 'odometry');
%     xlabel('x [m]')
%     ylabel('y [m]')
%     lgd.FontSize = 18;

        


end

figure;
plot(parameters, vError_r_odom*100);
xlabel(xlab_str)
ylim([0 1]);
title('Rotation error')

figure;
plot(parameters, vError_t_odom*100);
ylim([0 10]);
xlabel(xlab_str)
title('Translation error')

end