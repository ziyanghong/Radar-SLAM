close all

%% Groundtruth poses
end_index_animation = 1800;
sequence = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/2019-01-17-13-26-39-radar-oxford-10k/';
save_path = '/home/hong/Desktop/IJRR_RadarSLAM_Material/oxford/';

% end_index_animation = 1000;
% sequence = '/media/hong/DiskB/MulRan_Dataset/Riverside01/';
% save_path = '/home/hong/Desktop/IJRR_RadarSLAM_Material/mulran/';

[poses_gt, gt_timestamps] = readGroudtruthPoses(sequence);
x = reshape(poses_gt(1,3,1:end_index_animation),[1 end_index_animation]);
y = reshape(poses_gt(2,3,1:end_index_animation),[1 end_index_animation]);
gt_timestamps = gt_timestamps(1:end_index_animation);
gt_xy = [x;y];
offset = 100;

%% IROS
ro_folder = 'iros/';
[radar_iros_xy, radar_timestamps]= readOdomRadar(sequence, ro_folder);
[radar_iros_sync_xy, ~] = syncTranslation( gt_timestamps, radar_iros_xy, radar_timestamps);

%% SuMa
[suma_xy, suma_timstamps]= readOdomSuma(sequence);
[~, suma_lost_idx] = min(abs(suma_timstamps(end) - radar_timestamps));
[suma_sync_xy, ~] = syncTranslation(gt_timestamps, suma_xy, suma_timstamps);

%% IJRR
ro_folder = 'ro/';
[radar_ijrr_xy, ~]= readOdomRadar(sequence, ro_folder);
[radar_ijrr_sync_xy, ~] = syncTranslation( gt_timestamps, radar_ijrr_xy, radar_timestamps);

%% ORB

[orb_xy, orb_timstamps]= readOdomORB(sequence);
[orb_sync_xy, camera_data_idxs] = syncTranslation(gt_timestamps, orb_xy, orb_timstamps);

%% sync all methods

concat_all_x = [orb_sync_xy(1,:), suma_sync_xy(1,:), radar_iros_sync_xy(1,:), radar_ijrr_sync_xy(1,:)];
concat_all_y = [orb_sync_xy(2,:), suma_sync_xy(2,:), radar_iros_sync_xy(2,:), radar_ijrr_sync_xy(2,:)];
% concat_all_x = [ suma_sync_xy(1,:), radar_iros_sync_xy(1,:), radar_ijrr_sync_xy(1,:)];
% concat_all_y = [ suma_sync_xy(2,:), radar_iros_sync_xy(2,:), radar_ijrr_sync_xy(2,:)];

%% Plot
min_x = min(concat_all_x)-offset;
max_x = max(concat_all_x)+offset;
min_y = min(concat_all_y)-offset;
max_y = max(concat_all_y)+offset;
line_width = 2;
h_gt = animatedline('Color', 'black', 'LineWidth', line_width);
h_orb = animatedline('Color', 'cyan', 'LineWidth', line_width);
h_suma = animatedline('Color', 'red', 'LineWidth', line_width);
h_iros = animatedline('Color', 'green', 'LineWidth', line_width);
h_ijrr = animatedline('Color', 'blue', 'LineWidth', line_width);
daspect([1 1 1])
xlim([min_x, max_x]);
ylim([min_y, max_y]);
ylabel('y [m]');
xlabel('x [m]')
% legend('Groundtruth', 'SuMa','Baseline SLAM', 'Our SLAM','Location', 'southwest')
legend('Groundtruth', 'ORB SLAM', 'SuMa','Baseline SLAM', 'Our SLAM','Location', 'northwest')

for i = 1:size(gt_xy,2)
    
    addpoints(h_gt, gt_xy(1, i), gt_xy(2, i));
    addpoints(h_orb, orb_sync_xy(1, i), orb_sync_xy(2, i));
    addpoints(h_suma, suma_sync_xy(1, i), suma_sync_xy(2, i));
    addpoints(h_iros, radar_iros_sync_xy(1, i), radar_iros_sync_xy(2, i));
    addpoints(h_ijrr, radar_ijrr_sync_xy(1, i), radar_ijrr_sync_xy(2, i));

    drawnow update
    %% write to trajectory image
    ax = gca;
    filename = strcat(save_path, 'odom_traj/', num2str(i,  '%06.f'),'.png');
    exportgraphics(ax, filename, 'Resolution', 300)
    %% write sync camera data
%     img_name = strcat(sequence, 'stereo/image_0/', num2str(camera_data_idxs(i),  '%06.f'),'.png');
%     image_left = imread(img_name);
%     imwrite(image_left, strcat(save_path, 'camera/', num2str(i,  '%06.f'),'.png'));
%     
end


