close all;
clear all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Visualize the loop closure of correct and wrong loop
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Read files


paths = ['/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/'...
     '2019-01-16-13-09-37-radar-oxford-10k/'];



%% main
paths = string(paths);
path = paths(1);
image_path = strcat(path, 'radar_zfill_six/*.png');
imagefiles = dir(image_path);
descriptors_forward = zeros(size(imagefiles,1),192);
latents = zeros(size(imagefiles,1),2);

%% radar scan parameters
if contains(path,'Oxford_Radar_RobotCar_Dataset')
    max_selected_distance = 87.5;
    radar_resolution = 0.0438;
    max_distance = 163;
else
    max_selected_distance = 87.5;
    radar_resolution = 0.1736;
    max_distance = 100;
end

%% Compute descriptor
%     for i=1:size(imagefiles,1)
threads = 32;
parfor (i=1:size(imagefiles,1),threads)
% for i=1:size(imagefiles,1)
    
% for i=6026
    imgName = strcat(imagefiles(i).folder, '/', imagefiles(i).name);
    img = imread(imgName);
    % figure; imshow(img);
    % Pre-process the polar image
    [point_cloud_forward, point_cloud_opposite] = scan2pointCloud(img, max_selected_distance, radar_resolution, max_distance);
    
   
    % Compute forward descriptor
    [desM2DP, ~] = M2DP(point_cloud_forward);
    descriptors_forward(i,:) = desM2DP;
   
    
end
%% Precision recall parameters
threshold = 0.2;
true_threshold = 10;

%% Compute distance matrix
D_forward = pdist2(descriptors_forward,descriptors_forward);

%% Visualization
close all;
fig=figure; hold on;
xlabel('x-axis')
ylabel('y-axis')
trajectory = animatedline;
if contains(path,'Oxford_Radar_RobotCar_Dataset')
    gt_file = strcat(path,'gt/radar_odometry.csv');
    gt = readmatrix(gt_file);
    translation_xs = gt(1:size(gt,1),3);
    translation_ys = gt(1:size(gt,1),4);
    rotation_yaws = gt(1:size(gt,1),8);
    full_trajectory_gt = zeros(size(gt,1)+1, 3);
    RT_gt = eye(3,3);
    last_pose_gt = eye(3,3);
    for i = 1:size(gt,1)
        
        % gt pose
        RT_gt = [cos(rotation_yaws(i)) -sin(rotation_yaws(i)) translation_xs(i); sin(rotation_yaws(i)) cos(rotation_yaws(i)) translation_ys(i); 0 0 1];
        current_pose_gt = last_pose_gt*RT_gt;
        
        
        full_trajectory_gt(i+1,1) = current_pose_gt(1,3);
        full_trajectory_gt(i+1,2) = current_pose_gt(2,3);
        full_trajectory_gt(i+1,3) = atan2(current_pose_gt(1,1),current_pose_gt(2,1));
        
        last_pose_gt = current_pose_gt;
        
        
        addpoints(trajectory, current_pose_gt(1,3), -current_pose_gt(2,3));
        
        title(num2str(i));
        drawnow
        
        % start from ith frame
        if i-600 > 0
            candidates = D_forward(i+1, 1:i-100);
            [dist_feat,index] = min(candidates);
            
            
            % Recall
            if dist_feat < threshold
                %             if normailized_probability > 0.5
                dist =  sqrt((current_pose_gt(1,3) - full_trajectory_gt(index,1))^2 ...
                    + (current_pose_gt(2,3) - full_trajectory_gt(index,2))^2 );
                if dist < true_threshold
                    
                    plot( full_trajectory_gt(index,1), -full_trajectory_gt(index,2), ...
                        'Color', 'g', 'Marker', 'o' , 'DisplayName', 'True dectetion');
                    disp(['Frame: ' num2str(i+1) ' has correct loop in Frame ' num2str(index)]);
                    plot([full_trajectory_gt(i,1) full_trajectory_gt(index,1)],[-full_trajectory_gt(i,2) -full_trajectory_gt(index,2)],'g');
                    
                else
                    plot( full_trajectory_gt(index,1), -full_trajectory_gt(index,2), ...
                        'Color', 'r', 'Marker', 'x', 'DisplayName', 'False detection');
                    plot([full_trajectory_gt(i,1) full_trajectory_gt(index,1)],[-full_trajectory_gt(i,2) -full_trajectory_gt(index,2)],'r');
                    
                    disp(['Frame: ' num2str(i+1) ' has incorrect loop in Frame ' num2str(index)]);
                end
            end
            
            %             title(strcat('normailized probability: ', num2str(normailized_probability)));
        end
    end
    filename = strcat(path, 'loopVisualization.png');
    saveas(fig,filename);
    
    
else
    % Our dataset
    gt_file = strcat(path,'groundtruth_converted.txt');
    full_trajectory_gt = importdata(gt_file)    ;
    
    for i=1:size(full_trajectory_gt,1)
        
        addpoints(trajectory, full_trajectory_gt(i,1), full_trajectory_gt(i,2));
        drawnow
        frameTitle = strcat('Current frame: ', num2str(i));
        title(frameTitle);
        
        if i-300 > 0
            candidates = D_forward(i, 1:i-400);
            [dist_feat,index] = min(candidates);
            
            % Recall
            if dist_feat < threshold
                dist =  sqrt((full_trajectory_gt(i,1) - full_trajectory_gt(index,1))^2 ...
                    + (full_trajectory_gt(i,2) - full_trajectory_gt(index,2))^2 );
                % True or false
                if dist < true_threshold
                    
                    plot( full_trajectory_gt(index,1), full_trajectory_gt(index,2), ...
                        'Color', 'g', 'Marker', 'o' , 'DisplayName', 'True dectetion');
                    %                     disp(['Frame: ' num2str(i) ' has correct loop in Frame ' num2str(index)]);
                    plot([full_trajectory_gt(i,1) full_trajectory_gt(index,1)],[full_trajectory_gt(i,2) full_trajectory_gt(index,2)],'g');
                    
                else
                    plot( full_trajectory_gt(index,1), full_trajectory_gt(index,2), ...
                        'Color', 'r', 'Marker', 'x', 'DisplayName', 'False detection');
                    plot([full_trajectory_gt(i,1) full_trajectory_gt(index,1)],[ full_trajectory_gt(i,2) full_trajectory_gt(index,2)],'r');
                    
                    disp(['Frame: ' num2str(i) ' has incorrect loop in Frame ' num2str(index)]);
                end
                
            end
        end
    end
end



