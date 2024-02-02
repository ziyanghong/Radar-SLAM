close all;
clear all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualize the loop closure of correct and wrong loop
% Choose between oxford dataset and ours
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Read files

% paths = ['/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/'...
%     '2019-01-18-14-14-42-radar-oxford-10k/'];
% paths = ['/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/'...
%      '2019-01-17-14-03-00-radar-oxford-10k/'];
paths = ['/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/'...
     '2019-01-16-13-09-37-radar-oxford-10k/'];

% paths = ['/home/hong/Documents/VW_RADAR_DATASET/Loops/2_2020-01-23-21-43-57/'];
% paths = ['/home/hong/Documents/VW_RADAR_DATASET/Loops/0_2020-01-23-15-09-59/'];
% paths = ['/home/hong/Documents/VW_RADAR_DATASET/Loops/3_2020-02-10-15-15-02/'];



%% main
paths = string(paths);
path = paths(1);
image_path = strcat(path, 'radar_zfill_six/*.png');
polarImagefiles = dir(image_path);
cartImagefiles = dir(strcat(path, '701_radar_cart/*.png'));
% descriptors_forward = zeros(size(imagefiles,1),129);
% descriptors_opposite = zeros(size(imagefiles,1),129);
descriptors_forward = zeros(size(polarImagefiles,1),192);
latents = zeros(size(polarImagefiles,1),2);
selectedFrames = [];
selectedPoints = [];
timestamps = [];
gps = [];
%% radar scan parameters
if contains(path,'Oxford_Radar_RobotCar_Dataset')
    max_selected_distance = 87.5; % default
    radar_resolution = 0.0438;
    max_distance = 163;
else
    max_selected_distance = 87.5;
    radar_resolution = 0.1736;
    max_distance = 100;
end
%% parameters
max_selected_distance = 50;
kFrames = 10;
startFrame = 2;
opticFlow = opticalFlowHS;

%% Compute descriptor
threads = 32;
%     for i=1:size(imagefiles,1)
% parfor (i=1:size(imagefiles,1),threads)
% for i=1:size(imagefiles,1)

for i=startFrame:(startFrame+kFrames)
    %% point cloud and M2DP descriptor
    imgName = strcat(polarImagefiles(i).folder, '/', polarImagefiles(i).name);
    img = imread(imgName);
    % figure; imshow(img);
    % Pre-process the polar image
    [point_cloud_forward, point_cloud_opposite] = scan2pointCloud(img, max_selected_distance, radar_resolution, max_distance);
    selectedFrames = [selectedFrames;i];
    selectedPoints = [selectedPoints; {point_cloud_forward}];
%     [coeff,score,latent] = pca(point_cloud_forward(:,1:2));
%     latents(i,:) = latent';
    
    
%             % display the input data
%             figure;grid on;hold on
%             subplot(121)
%             pcshow(point_cloud_forward);title('Input forward point cloud')
%             xlabel('X');
%             ylabel('Y');
%             zlabel('Z');
%             subplot(122)
%             pcshow(point_cloud_opposite);title('Input opposite point cloud')
%             xlabel('X');
%             ylabel('Y');
%             zlabel('Z');

    % Compute forward descriptor
    [desM2DP, ~] = M2DP(point_cloud_forward);
    descriptors_forward(i,:) = desM2DP;
%     
%     % Compute opposite descriptor
% %     tic
%     [desM2DP, ~] = M2DP(point_cloud_opposite);
% %     toc
%     descriptors_opposite(i,:) = desM2DP;

%     %% Optical flow
%     imgName = strcat(cartImagefiles(i).folder, '/', cartImagefiles(i).name);
%     frameGray = imread(imgName);    
% %     frameGray = rgb2gray(frameRGB);  
%     flow = estimateFlow(opticFlow,frameGray);
%     figure;
%     imshow(frameGray)
%     hold on
%     plot(flow,'DecimationFactor',[5 5],'ScaleFactor',60);
%     hold off    
    
end

%% Precision recall parameters
% threshold = 0.3;
% threshold = 0.25;
threshold = 0.2;

true_threshold = 10;

pca_threshold = 3.0;
%% Compute distance matrix
D_forward = pdist2(descriptors_forward,descriptors_forward);
% D_forward = pdist2(descriptors_opposite,descriptors_forward);

%% Visualization
all_poses = [];
all_relative_poses = [];
% close all;
% fig=figure; hold on;
% xlabel('x-axis')
% ylabel('y-axis')
% trajectory = animatedline;
if contains(path,'Oxford_Radar_RobotCar_Dataset')
    timestamps = readmatrix(strcat(path,'Navtech_Polar.txt'));
    gps = readmatrix(strcat(path,'gps/gps.csv'));
    gt_file = strcat(path,'gt/radar_odometry.csv');
    gt = readmatrix(gt_file);
    translation_xs = gt(1:size(gt,1),3);
    translation_ys = gt(1:size(gt,1),4);
    rotation_yaws = -gt(1:size(gt,1),8);
    full_trajectory_gt = zeros(size(gt,1)+1, 3);
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
        
        % Trajectory
        full_trajectory_gt(i+1,1) = current_pose_gt(1,3);
        full_trajectory_gt(i+1,2) = current_pose_gt(2,3);
        full_trajectory_gt(i+1,3) = atan2(current_pose_gt(1,1),current_pose_gt(2,1));
        
        last_pose_gt = current_pose_gt;
        
%         addpoints(trajectory, current_pose_gt(1,3), current_pose_gt(2,3));
%         
%         title(num2str(i));
%         drawnow
%         
%         % start from ith frame
%         if i-600 > 0
%             candidates = D_forward(i+1, 1:i-100);
%             [dist_feat,index] = min(candidates);
%             
%             if latents(i,1) / latents(i,2) > pca_threshold
%                 continue;
%             end
%             
%             % Recall
%             if dist_feat < threshold
%                 %             if normailized_probability > 0.5
%                 dist =  sqrt((current_pose_gt(1,3) - full_trajectory_gt(index,1))^2 ...
%                     + (current_pose_gt(2,3) - full_trajectory_gt(index,2))^2 );
%                 if dist < true_threshold
%                     
%                     plot( full_trajectory_gt(index,1), -full_trajectory_gt(index,2), ...
%                         'Color', 'g', 'Marker', 'o' , 'DisplayName', 'True dectetion');
%                     disp(['Frame: ' num2str(i+1) ' has correct loop in Frame ' num2str(index)]);
%                     plot([full_trajectory_gt(i,1) full_trajectory_gt(index,1)],[-full_trajectory_gt(i,2) -full_trajectory_gt(index,2)],'g');
%                     
%                 else
%                     plot( full_trajectory_gt(index,1), -full_trajectory_gt(index,2), ...
%                         'Color', 'r', 'Marker', 'x', 'DisplayName', 'False detection');
%                     plot([full_trajectory_gt(i,1) full_trajectory_gt(index,1)],[-full_trajectory_gt(i,2) -full_trajectory_gt(index,2)],'r');
%                     
%                     disp(['Frame: ' num2str(i+1) ' has incorrect loop in Frame ' num2str(index)]);
%                 end
%             end
%             
%             %             title(strcat('normailized probability: ', num2str(normailized_probability)));
%         end
%     end
%     filename = strcat(path, 'loopVisualization.png');
%     saveas(fig,filename);
%     
%     
% else
%     % Our dataset
%     gt_file = strcat(path,'groundtruth_converted.txt');
%     full_trajectory_gt = importdata(gt_file)    ;
%     
%     for i=1:size(full_trajectory_gt,1)
%         
%         addpoints(trajectory, full_trajectory_gt(i,1), full_trajectory_gt(i,2));
%         drawnow
%         frameTitle = strcat('Current frame: ', num2str(i));
%         title(frameTitle);
%         
%         if i-300 > 0
%             candidates = D_forward(i, 1:i-400);
%             [dist_feat,index] = min(candidates);
%             
%             % Recall
%             if dist_feat < threshold
%                 dist =  sqrt((full_trajectory_gt(i,1) - full_trajectory_gt(index,1))^2 ...
%                     + (full_trajectory_gt(i,2) - full_trajectory_gt(index,2))^2 );
%                 % True or false
%                 if dist < true_threshold
%                     
%                     plot( full_trajectory_gt(index,1), full_trajectory_gt(index,2), ...
%                         'Color', 'g', 'Marker', 'o' , 'DisplayName', 'True dectetion');
%                     %                     disp(['Frame: ' num2str(i) ' has correct loop in Frame ' num2str(index)]);
%                     plot([full_trajectory_gt(i,1) full_trajectory_gt(index,1)],[full_trajectory_gt(i,2) full_trajectory_gt(index,2)],'g');
%                     
%                 else
%                     plot( full_trajectory_gt(index,1), full_trajectory_gt(index,2), ...
%                         'Color', 'r', 'Marker', 'x', 'DisplayName', 'False detection');
%                     plot([full_trajectory_gt(i,1) full_trajectory_gt(index,1)],[ full_trajectory_gt(i,2) full_trajectory_gt(index,2)],'r');
%                     
%                     disp(['Frame: ' num2str(i) ' has incorrect loop in Frame ' num2str(index)]);
%                 end
%                 
%             end
%             
%         end
    end
end


%% Show accumulated points cloud
selectedPoints = [];
[accumulated_points, selectedPoses, vXY] = accumulatePoints(selectedPoints, selectedFrames, all_poses,all_relative_poses,timestamps);
figure;grid on;hold on
scatter(accumulated_points(:,1),accumulated_points(:,2),'.');
plot(vXY(:,1), vXY(:,2), 'LineWidth',2);
hold off;
%% Show gps and google street view
timestampSeleted = timestamps(selectedFrames(1:end),4);
for k=1:numel(timestampSeleted)

    [val,gps_idx]=min(abs(gps(:,1)/1000000 - timestampSeleted(k)));
    gps_selected_idx(k) = gps_idx;
end
lat_lon = gps(gps_selected_idx,3:4);
initial_gps = gps(1,3:4);
showStreetViewAndGPS(lat_lon, accumulated_points(:,1:2), vXY, initial_gps);



%% Visualize opposite loop
% fid = fopen('oppositeLoop.txt','wt');
% cart_image_path = strcat(path, '/701_radar_cart/');
% save_image_path = '/home/hong/Desktop/PoseError/ReverseLoopImages/';
% 
% D = pdist2(full_trajectory_gt(:,1:2),full_trajectory_gt(:,1:2));
% D_decs = pdist2(descriptors_forward, descriptors_opposite);
% 
% for i=1:size(full_trajectory_gt,1)
%     
%     indices = find(D_decs(i,1:i-50)<0.2);
%     if size(indices,2) == 0
%         continue;
%     end
%     
%     if latents(i,1) / latents(i,2) > pca_threshold
%         continue;
%     end
%     
%     for j=1:size(indices,2)
%         %         c1 = (i - indices(j)) > 100; % try to avoid self matching
%         %         c2 = abs(full_trajectory_gt(i,3) - full_trajectory_gt(indices(j),3)) > 3; % check heading to be opposite
%         %         if  c1 && c2
%         %             forward_id = i;
%         %             reverse_id = indices(j);
% %                     descriptor_forward = descriptors_forward(forward_id,:);
% %                     descriptor_reverse = descriptors_opposite(reverse_id,:);
% %                     dist_decs = pdist2(descriptor_forward,descriptor_reverse);
% 
%         img1 = imread(strcat(cart_image_path, num2str(i,'%06.f'), '.png'   ));
%         img2 = imread(strcat(cart_image_path, num2str(indices(j),'%06.f'), '.png'));
%         
%         if(D(i, indices(j)) < 10)
%             fprintf(fid, ['Frame: ' num2str(i) ' has opposite loop in Frame ' num2str(indices(j)) ' with descriptor distance: '   num2str( D_decs(i, indices(j)))  '\n']);
%             filename = strcat(save_image_path, 'correctLoop/',num2str(i,'%06.f'), '_' , num2str(indices(j),'%06.f'), '.png');
%         else
%             disp(['Frame: ' num2str(i) ' has wrong opposite loop in Frame ' num2str(indices(j))]);
%             [r,c] = size(img2);
%             position =  [r/2, c/2];
%             text_str = 'Wrong loop';
%             img2 = insertText(img2, position, text_str, 'FontSize', 18, 'BoxColor', 'red',...
%                 'BoxOpacity', 0.4, 'TextColor', 'white');
%             img3d = zeros(r,c,3);
%             img3d(:,:,1) = img1;
%             img3d(:,:,2) = img1;
%             img3d(:,:,3) = img1;
%             img1 = img3d;
%             
%             filename = strcat(save_image_path, 'wrongLoop/',num2str(i,'%06.f'), '_' , num2str(indices(j),'%06.f'), '.png');
%             
%         end
%         
%         image_match = [img1, img2];
%         imwrite(image_match,filename)
% %         end
%     end
% end
% fclose(fid);



