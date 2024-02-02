close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Radar localization using OpenStreetMap
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Dataset path
% paths = ['/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/'...
%     '2019-01-18-14-14-42-radar-oxford-10k/'];
% paths = ['/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/'...
%      '2019-01-17-14-03-00-radar-oxford-10k/'];
paths = ['/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/'...
     '2019-01-16-13-09-37-radar-oxford-10k/'];

% paths = ['/home/hong/Documents/VW_RADAR_DATASET/Loops/2_2020-01-23-21-43-57/'];
% paths = ['/home/hong/Documents/VW_RADAR_DATASET/Loops/0_2020-01-23-15-09-59/'];
% paths = ['/home/hong/Documents/VW_RADAR_DATASET/Loops/3_2020-02-10-15-15-02/'];
%% sub images paths
paths = string(paths);
path = paths(1);
image_path = strcat(path, 'radar_zfill_six/*.png');
polarImagefiles = dir(image_path);
cartImagefiles = dir(strcat(path, '701_radar_cart/*.png'));
% descriptors_forward = zeros(size(imagefiles,1),129);
% descriptors_opposite = zeros(size(imagefiles,1),129);
descriptors_forward = zeros(size(polarImagefiles,1),192);
latents = zeros(size(polarImagefiles,1),2);


%% radar scan parameters
if contains(path,'Oxford_Radar_RobotCar_Dataset')
    max_selected_distance = 87.5; % default
    radar_resolution = 0.0438;
    max_distance = 163;
    openstreetmap_filename = 'map_oxford.osm';

else
    max_selected_distance = 87.5;
    radar_resolution = 0.1736;
    max_distance = 100;
end

%% parameters
num_points_per_beam = 5;
max_selected_distance = 60;
kFrames = 3;
startFrame = 1;
topK = 5; % top k cluster of point cloud

%% parse OpenStreetMap
% if ~exist('parsed_osm','var')
%    [parsed_osm, osm_xml] = parse_openstreetmap(openstreetmap_filename);
%    disp('Parsed osm.');
% end

%% plot OpenStreetMap
% fig = figure;
% ax = axes('Parent', fig);
% hold(ax, 'on')
% % plot the network, optionally a raster image can also be provided for the
% % map under the vector graphics of the network
% plot_way(ax, parsed_osm)



%% read poses
[all_poses, all_relative_poses,timestamps,gps] = readGroundtruthPoses(path);
%% plot poses and gps
% heading = -1.1215;
% initial_gps = gps(1,3:4);
% poses_mat = cell2mat(all_poses);
% poses_lats_lons = localCoordinate2gps([poses_mat(1:3:end-2,3), poses_mat(2:3:end-1,3)], heading, initial_gps);
% 
% plot(ax, poses_lats_lons(:,2),  poses_lats_lons(:,1), 'r--')
% plot(ax, gps(:,4), gps(:,3), 'k--')

% hold(ax, 'off')



%% Extract point cloud
selectedFrames = [];
selectedPoints = [];


% for i=startFrame:(startFrame+kFrames)
for i=startFrame

    % point cloud
    imgName = strcat(polarImagefiles(i).folder, '/', polarImagefiles(i).name);
    img = imread(imgName);
    
    % Pre-process the polar image
    tic
    [point_cloud_forward] = extractPointFromPolar_mex(img, max_selected_distance,...
                                      radar_resolution, max_distance,num_points_per_beam);
                                  toc
    selectedFrames = [selectedFrames;i];
    selectedPoints = [selectedPoints; {point_cloud_forward}];
    
    cart_path = strcat( path, '/701_radar_cart/');
    img_cart = imread(strcat(cart_path, polarImagefiles(i).name));
    [rows,cols] =  size(img_cart);
    figure;
    imshow(img_cart);
    hold on;
    for j=1:1:size(point_cloud_forward)
        
        col = cols/2 + point_cloud_forward(j,2)/(max_selected_distance/(cols/2));
        row = rows/2 - point_cloud_forward(j,1)/(max_selected_distance/(cols/2));
        
        plot(row,col,'yo');
    end
%     hold off;
end

%% Show accumulated points cloud
[accumulated_points, selectedPoses, vXY] = accumulatePoints(selectedPoints,...
    selectedFrames, all_poses,all_relative_poses,timestamps);
figure;grid on;
hold on
scatter(accumulated_points(:,1),accumulated_points(:,2),'.');
plot(vXY(:,1), vXY(:,2), 'LineWidth',2);
hold off;

%% Show gps and google street view
% timestampSeleted = timestamps(selectedFrames(1:end),4);
% for k=1:numel(timestampSeleted)
% 
%     [val,gps_idx]=min(abs(gps(:,1)/1000000 - timestampSeleted(k)));
%     gps_selected_idx(k) = gps_idx;
% end
% lat_lon = gps(gps_selected_idx,3:4);
% initial_gps = gps(1,3:4);
% 
% 
% showStreetViewAndGPS(ax, lat_lon, accumulated_points(:,1:2), vXY, initial_gps);
% exportgraphics(ax,'map.png','Resolution',1000)

%% Segment point cloud
filteredPtCloud = filterPointCloud(accumulated_points,topK);






