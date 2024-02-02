function [vErrors_kf, vErrors_odom, ...
          vError_t_kf, vError_r_kf,...
          vError_t_odom, vError_r_odom, ... 
          lengths_errors_kf, speed_errors_kf, ...
          lengths_errors_odom, speed_errors_odom] = evaluateOneSequenceRadar(sequence,poses_gt,lengths,ro_folder)


timestamp_file = strcat(sequence, 'radar.timestamps');
M = dlmread(timestamp_file, ' ');
radar_timestamps = M(:,1);

% ro_result_path = strcat(sequence, ro_folder, 'keyframe/');
ro_result_path = strcat(sequence, ro_folder,'keyframe/');
Files = dir(ro_result_path);
vError_t_kf = [];
vError_r_kf = [];

lengths_errors_kf = zeros(numel(lengths),3);
lengths_errors_kf(:,1) = linspace(100,800,numel(lengths)); % [lengths, rotation_error, translation_error]
speed_errors_kf = [];

lengths_errors_odom = zeros(numel(lengths),3);
lengths_errors_odom(:,1) = linspace(100,800,numel(lengths)); % [lengths, rotation_error, translation_error]
speed_errors_odom = [];

%% Keyframe poses
% gt
fig1 = figure;
hold on;
x = reshape(poses_gt(1,3,:),[1 size(poses_gt,3)]);
y = reshape(poses_gt(2,3,:),[1 size(poses_gt,3)]);
plot(x, y, 'k--','LineWidth',1);
gt_x = x;
gt_y = y;
hold on;
% Keyframe
for f = 3:size(Files,1)
    x = 0;
    y = 0;
    translation_error_min = 1.0;
    % If contains g2o file, means loop is detected
    ro_file = strcat(ro_result_path, Files(f).name ,'/keyframe_pose.csv');
    if isfile(ro_file)
        ro = readmatrix(ro_file);
        ro = sortrows(ro);
        ro_timestamps = radar_timestamps(ro(:,1));
        yaws_ro = ro(:,4)';
        poses_ro = zeros(3,3,size(ro,1));
        for i = 1:size(ro,1)
            poses_ro(:,:,i) = [cos(yaws_ro(i)) -sin(yaws_ro(i)) ro(i,2);
                sin(yaws_ro(i))  cos(yaws_ro(i)) ro(i,3);
                0                0                   1   ];
        end
        [sync_ro_poses, sync_gt_poses] = syncPoses(poses_gt, radar_timestamps, poses_ro, ro_timestamps);
        vErrors_kf = calcSequenceErrors(sync_gt_poses, sync_ro_poses,lengths);
        if strcmp(ro_folder, 'iros/')
            algorithm = 'baseline';
        elseif contains(ro_folder, 'ro/')
            algorithm = 'radar';
        end
        convert2Poses4ATE(sync_ro_poses, sync_gt_poses, ro_timestamps, sequence, algorithm);
        
        rotation_error = mean(vErrors_kf(:,2));
        translation_error = mean(vErrors_kf(:,3));
        vError_r_kf = [vError_r_kf; rotation_error];
        vError_t_kf = [vError_t_kf; translation_error];
        
        %% compute error per length
        for i=1:numel(lengths)
            indexs = find(vErrors_kf(:,4)==lengths_errors_kf(i,1));
            rotation_errors = vErrors_kf(indexs,2);
            translation_errors = vErrors_kf(indexs,3);
            lengths_errors_kf(i,2) = mean(rotation_errors);
            lengths_errors_kf(i,3) = mean(translation_errors);
           
        end        
        
        %% compute error per speed
        speed_errors_kf = errorAgainstSpeed(vErrors_kf);
      
        if translation_error < translation_error_min
            translation_error_min = translation_error;
            x = reshape(poses_ro(1,3,:),[1 size(ro,1)]);
            y = reshape(poses_ro(2,3,:),[1 size(ro,1)]);
        end
        
    end
end

plot(x,y,'b','LineWidth',2);
lgd = legend('groundtruth', 'keyframe');
hold off;
[min_error_t, min_seq] = min(vError_t_kf);
[min_error_r, ~] = min(vError_r_kf);
display(sequence);
display(strcat('The sequence has minimum error of keyframe translation: ', num2str(min_error_t), ' and rotation: ', num2str(min_error_r)));




%% Pure odometry
% gt
ro_result_path = strcat(sequence, ro_folder, 'odometry/');

% hold off;

Files = dir(ro_result_path);
vError_t_odom = [];
vError_r_odom = [];
for f = 3:size(Files,1)
    if size(dir(strcat(ro_result_path, Files(f).name, '/our_result_odometry.csv')),1) > 0
        x = 0;
        y = 0;
        translation_error_min = 1.0;
        ro_file = strcat(ro_result_path, Files(f).name ,'/our_result_odometry.csv');
        ro = readmatrix(ro_file);
        ro = sortrows(ro);
        ro_timestamps = radar_timestamps(ro(:,1));
        yaws_ro = ro(:,4)';
        
        poses_ro = zeros(3,3,size(ro,1));
        for i = 1:size(ro,1)
            poses_ro(:,:,i) = [cos(yaws_ro(i)) -sin(yaws_ro(i)) ro(i,2);
                sin(yaws_ro(i))  cos(yaws_ro(i)) ro(i,3);
                0                0                   1   ];
        end
        
        [sync_ro_poses, gt_sync_poses] = syncPoses(poses_gt, radar_timestamps, poses_ro, ro_timestamps);
        if strcmp(ro_folder, 'iros/')
            algorithm = 'baseline_odom';
        elseif contains(ro_folder, 'ro/')
            algorithm = 'radar_odom';
        end
        convert2Poses4ATE(sync_ro_poses, gt_sync_poses, ro_timestamps, sequence, algorithm);     
        
        vErrors_odom = calcSequenceErrors(gt_sync_poses, sync_ro_poses,lengths);
        rotation_error = mean(vErrors_odom(:,2));
        translation_error = mean(vErrors_odom(:,3));
        vError_r_odom = [vError_r_odom; rotation_error];
        vError_t_odom = [vError_t_odom; translation_error];
        
        %% compute error per length
        for i=1:numel(lengths)
            indexs = find(vErrors_odom(:,4)==lengths_errors_odom(i,1));
            rotation_errors = vErrors_odom(indexs,2);
            translation_errors = vErrors_odom(indexs,3);
            lengths_errors_odom(i,2) = mean(rotation_errors);
            lengths_errors_odom(i,3) = mean(translation_errors);
           
        end        
        
        %% compute error per speed
        speed_errors_odom = errorAgainstSpeed(vErrors_odom);
        
        %% update the minimum translation
        if translation_error < translation_error_min
            translation_error_min = translation_error;
            x = reshape(poses_ro(1,3,:),[1 size(ro,1)]);
            y = reshape(poses_ro(2,3,:),[1 size(ro,1)]);
        end

        %% Plot each trajectory
        figure;
        hold on;
        plot(gt_x, gt_y, 'LineWidth',2);
        plot(x,y,'LineWidth',2);        
        hold off;
        lgd = legend('groundtruth', 'odometry');
        xlabel('x [m]')
        ylabel('y [m]')
        lgd.FontSize = 18;


    end
end
% plot(x,y,'g','LineWidth',2);

[min_error_t, min_seq] = min(vError_t_odom);
[min_error_r, ~] = min(vError_r_odom);
display(strcat('The sequence has minimum error of translation for pure odometry: ',num2str(min_error_t), ' and rotation: ', num2str(min_error_r)));
% hold off;
% legend('groundtruth', 'keyframe','odometry');

% saveas(fig2,strcat(ro_result_path,'odometry.png'));

%% Save both keyframe and pure odometry in the same figure
% set(gca,'FontSize',18);
% legend('Ground Truth','Radar SLAM', 'Radar Odometry','Location','southwest', 'Fontsize',14);
% xlabel('x [m]','Fontsize',18);
% ylabel('y [m]','Fontsize',18);
% xlim([-200,1400]);
% newStr = split(sequence,'/');
% saveas(fig1,strcat(sequence, convertCharsToStrings(newStr(end-1)), '_trajectory.eps'), 'epsc');


end