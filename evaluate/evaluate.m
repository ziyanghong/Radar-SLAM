close all;
%%
% dataset_path = '/home/hong/Documents/VW_RADAR_DATASET/Loops/';
dataset_path = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/';
% dataset_path = '/media/hong/DiskB/MulRan_Dataset/';

Files = dir(dataset_path);


%% Experiments parameters
lengths = [100,200,300,400,500,600,700,800];


%% ---------------------------------------Radar IROS------------------------------------------
display('IROS result.');
ro_folder = 'iros/';
[lengths_errors_kf_iros, speed_errors_kf_iros,...
    lengths_errors_odom_iros, speed_errors_odom_iros, ...
    vErrors_keyframe_all_iros, vErrors_ro_all_iros] = evaluateRadar(Files, lengths, ro_folder);


%% ---------------------------------------Radar Journal------------------------------------------
display('Journal result.');
ro_folder = 'ro/';
[lengths_errors_kf_journal, speed_errors_kf_journal,...
 lengths_errors_odom_journal, speed_errors_odom_journal,...
 vErrors_keyframe_all_journal, vErrors_ro_all_journal] = evaluateRadar(Files, lengths, ro_folder);


%% --------------------------------------------------SuMa------------------------------------------------------
[lengths_errors_suma, speed_errors_suma, vErrors_all_suma] = evaluateSuMa(Files, lengths);


%% ----------------------------------------------ORB STEREO----------------------------------------------------
[lengths_errors_orb, speed_errors_orb, vErrors_all_orb] = evaluateORB(Files, lengths);



%% Box chart plot
close all;
translationError = [vErrors_ro_all_iros(:,3);
                    vErrors_ro_all_journal(:,3);
                    vErrors_keyframe_all_journal(:,3);
                    vErrors_all_suma(:,3);
                    vErrors_all_orb(:,3)];
                
lengthList = [vErrors_ro_all_iros(:,4);
                    vErrors_ro_all_journal(:,4);
                    vErrors_keyframe_all_journal(:,4);
                    vErrors_all_suma(:,4);
                    vErrors_all_orb(:,4)];
      
methods = [repmat({'Baseline Odometry'}, size( vErrors_ro_all_iros,1), 1);
           repmat({'Our Odometry'}, size( vErrors_ro_all_journal,1), 1);
           repmat({'Our SLAM'}, size(vErrors_keyframe_all_journal, 1), 1);
           repmat({'SuMa'},     size( vErrors_all_suma,1), 1);           
           repmat({'ORB-SLAM2'}, size( vErrors_all_orb,1), 1)]; 
           
           
           
T1 = table(lengthList, translationError, methods)  ; 

lengthOrder = lengths;
T1.lengthList = categorical(T1.lengthList,lengthOrder);

figure
b1 = boxchart(T1.lengthList, T1.translationError*100, 'GroupByColor', T1.methods,...
    'MarkerStyle', 'none', 'BoxFaceAlpha',0.6);
b1(1).BoxFaceColor = [0 0.7 0];
b1(2).BoxFaceColor = [0 0.8 0.8];
b1(3).BoxFaceColor = 'k';
b1(4).BoxFaceColor = 'b';
b1(5).BoxFaceColor = 'r';
xlabel('Path Length (m)')
ylabel('Translation Error (%)');
ylim([0 12]);
lgd = legend;
lgd.FontSize = 12;

rotationError = [vErrors_ro_all_iros(:,2);
                    vErrors_ro_all_journal(:,2);
                    vErrors_keyframe_all_journal(:,2);
                    vErrors_all_suma(:,2);
                    vErrors_all_orb(:,2)];
T2 = table(lengthList, rotationError, methods)  ; 

lengthOrder = lengths;
T2.lengthList = categorical(T2.lengthList,lengthOrder);

figure
b2 = boxchart(T2.lengthList, T2.rotationError, 'GroupByColor', T2.methods,...
    'MarkerStyle', 'none', 'BoxFaceAlpha',0.6);
b2(1).BoxFaceColor = [0 0.7 0];
b2(2).BoxFaceColor = [0 0.8 0.8];
b2(3).BoxFaceColor = 'k';
b2(4).BoxFaceColor = 'b';
b2(5).BoxFaceColor = 'r';
xlabel('Path Length (m)')
ylabel('Rotation Error (deg/m)');
ylim([0 0.07]);
lgd = legend;
lgd.FontSize = 12;                

%% Plot error against length 

% Translation error(%)
figure;
p1 = plot(lengths, lengths_errors_odom_iros(:,3)*100, '-o',...
          lengths, lengths_errors_orb(:,3)*100,'-d', ...
          lengths, lengths_errors_odom_journal(:,3)*100, 'k-o',...
          lengths, lengths_errors_kf_journal(:,3)*100, 'b-o',...
          lengths, lengths_errors_suma(:,3)*100, 'r-x',  'LineWidth',1.7 );
p1(1).Color = [0 0.7 0];
p1(2).Color = [0 0.8 0.8];

 
lgd1 = legend('Baseline Odometry', 'ORB-SLAM2','Our Odometry', 'Our SLAM', 'SuMa');
lgd1.FontSize = 12;
xlabel('Path Length (m)');
ylabel('Translation Error (%)');

% Rotation error(deg/m)
figure;
p2 = plot(lengths, lengths_errors_odom_iros(:,2), 'g-o', ...
          lengths, lengths_errors_orb(:,2), 'c-d',...
          lengths, lengths_errors_odom_journal(:,2), 'k-o',...
          lengths, lengths_errors_kf_journal(:,2), 'b-o',...
          lengths, lengths_errors_suma(:,2), 'r-x', 'LineWidth',1.7 );
p2(1).Color = [0 0.7 0];
p2(2).Color = [0 0.8 0.8];
lgd2 = legend('Baseline Odometry', 'ORB-SLAM2','Our Odometry', 'Our SLAM', 'SuMa');
lgd2.FontSize = 12;
xlabel('Path Length (m)');
ylabel('Rotation Error (deg/m)'); 

%% Plot error against speed 

% Translation error(%)
figure;
speeds = speed_errors_odom_iros(:,1);
p3 = plot(speeds, speed_errors_odom_iros(:,3)*100, '-o', ...
          speeds, speed_errors_orb(:,3)*100, '-d', ...
          speeds, speed_errors_odom_journal(:,3)*100, 'k-o',...
          speeds, speed_errors_kf_journal(:,3)*100, 'b-o',...
          speeds, speed_errors_suma(:,3)*100, 'r-x', 'LineWidth',1.7 );
p3(1).Color = [0 0.7 0];
p3(2).Color = [0 0.8 0.8];

 
lgd3 = legend('Baseline Odometry', 'ORB-SLAM2','Our Odometry', 'Our SLAM', 'SuMa');
lgd3.FontSize = 12;
xlabel('Speed (km/h)');
ylabel('Translation Error (%)');


% Rotation error(deg/m)
figure;
p4 = plot(speeds, speed_errors_odom_iros(:,2), 'g-o', ...
          speeds, speed_errors_orb(:,2), 'c-d',...
          speeds, speed_errors_odom_journal(:,2), 'k-o',...
          speeds, speed_errors_kf_journal(:,2), 'b-o',...          
          speeds, speed_errors_suma(:,2), 'r-x',  'LineWidth',1.7 );
p4(1).Color = [0 0.7 0];
p4(2).Color = [0 0.8 0.8];
lgd4 = legend('Baseline Odometry', 'ORB-SLAM2','Our Odometry', 'Our SLAM', 'SuMa');
lgd4.FontSize = 12;
xlabel('Speed (km/h)');
ylabel('Rotation Error (deg/m)'); 










%% ----------------------------------------------LOAM---------------------------------------------------------


% for f = 3:size(Files,1)
%     folderName = strcat( Files(f).folder , '/', Files(f).name, '/');
%     sequence = folderName;
%     lidar_odom_folder = strcat(folderName ,'lo/');
%     lo_file = strcat(lidar_odom_folder, 'loam_odom.csv');
%     
%     if (isfolder(folderName) && isfile(lo_file))
%         %% Radar Groundtruth
%         [poses_gt, radar_timestamps] = readGroudtruthPoses(sequence);
%         
%         % Loam Odometry
%         display(lo_file);
%         timestamp_file = strcat(sequence,'velodyne_left.timestamps');
%         M = dlmread(timestamp_file, ' ');
%         lidar_timstamps = M(:,1);
%         lo = readmatrix(lo_file);
%         lo_timestamps = lidar_timstamps(lo(:,1));
%         
%         quaternions = lo(:,3:6);
%         quaternions(:,[4 1]) = quaternions(:,[1 4]); % swap to [w x y z]
%         vRPY = quat2eul(quaternions);
%         poses_lo = zeros(3,3,size(lo,1));
%         pose = eye(3);
%         
%         previousPose = eye(4);
%         currentPose = eye(4);
%         for i = 1:size(lo,1)
%             
%             currentPose(1:3,1:3) = eul2rotm(vRPY(i,:));
%             currentPose(1:3,4)   = lo(i,7:9)';
%             relativePoseSE3 = computeRelativePose6DOF(previousPose, currentPose);
%             previousPose = currentPose;
%             eulZYX = rotm2eul(relativePoseSE3(1:3,1:3));
%             
%             relatposeSE2 = [cos(eulZYX(1)) -sin(eulZYX(1)) relativePoseSE3(1,4);
%                 sin(eulZYX(1))  cos(eulZYX(1)) relativePoseSE3(2,4);
%                 0               0                        1];
%             
%             pose = pose * relatposeSE2;
%             
%             poses_lo(:,:,i+1) = pose;
%             
%         end
%         %         x = reshape(poses_lo(1,3,:),[1 size(poses_lo,3)]);
%         %         y = reshape(poses_lo(2,3,:),[1 size(poses_lo,3)]);
%         %         figure;
%         %         plot(x,y,'DisplayName','lo');
%         %         xlabel('X')
%         %         ylabel('Y')
%         
%         figure;
%         X = lo(:,7);
%         Y = lo(:,8);
%         Z = lo(:,9);
%         lidar_frame = 500;
%         plot3(X(1:lidar_frame),Y(1:lidar_frame),Z(1:lidar_frame));
%         
%         hold on
%         [~, idx] = min(abs( lo_timestamps(lidar_frame,:) - radar_timestamps  ));
%         gt_X = poses_gt(1,3,1:idx);
%         gt_Y = poses_gt(2,3,1:idx);
%         gt_Z = zeros(1,idx);
%         plot3(gt_X(:), gt_Y(:), gt_Z', '-o','Color','r');
%         hold off;
%         xlabel('X (meter)')
%         ylabel('Y (meter)')
%         zlabel('Z (meter)')
%         
%         
%         % Calculate error
%         % Per sequence
%         [sync_loam_poses, sync_gt_poses] = syncPoses(poses_gt, radar_timestamps, poses_lo, lo_timestamps);
%         vErrors_lo = calcSequenceErrors(sync_gt_poses, sync_loam_poses,lengths);
%         rotation_error = mean(vErrors_lo(:,2));
%         translation_error = mean(vErrors_lo(:,3));
%         
%     end
% end

%%
% %% ----------------------------------------------baseline VO----------------------------------------------------------------
% vo_result_path = strcat(sequence, 'vo/');
% vo_file = strcat(vo_result_path, 'vo.csv');
% vo = readmatrix(vo_file);
% vo_timestamps = vo(:,2);
% yaws_vo = zeros(1,size(vo,1)+1);
% poses_vo = zeros(3,3,size(vo,1)+1);
% pose = eye(3);
% for i = 1:size(vo,1)
%     delta_x = vo(i,3);
%     delta_y = -vo(i,4);
%     delta_yaw = -vo(i,8);
%     yaws_vo(i+1) = yaws_vo(i) + delta_yaw;
%
%     RT = [cos(delta_yaw) -sin(delta_yaw) delta_x;
%           sin(delta_yaw)  cos(delta_yaw) delta_y;
%           0               0                1    ];
%     pose = pose * RT;
%     poses_vo(:,:,i+1) = pose;
% end
%
% sync_vo_poses = syncPoses(radar_timestamps, poses_vo, vo_timestamps);
% vErrors_vo = calcSequenceErrors(poses_gt, sync_vo_poses,lengths);
% mean_t_error_vo = mean(vErrors_vo(:,3));
% x = reshape(sync_vo_poses(1,3,:),[1 size(sync_vo_poses,3)]);
% y = reshape(sync_vo_poses(2,3,:),[1 size(sync_vo_poses,3)]);
% plot(x, y, 'DisplayName','vo');


%
%
% %% Plot yaw to verify the input of pose
% figure;
% subplot(4,1,1);
% plot(1:size(yaws_gt,2), yaws_gt);
% ylim([-10 10])
% pause(1)
% subplot(4,1,2);
% plot(1:size(yaws_lo,2), yaws_lo);
% ylim([-10 10])
% pause(1)
% subplot(4,1,3);
% plot(1:size(yaws_vo,2), yaws_vo);
% ylim([-10 10])
% pause(1)
% subplot(4,1,4);
% plot(1:size(yaws_ro,2), yaws_ro);
% ylim([-10 10])
%
%

