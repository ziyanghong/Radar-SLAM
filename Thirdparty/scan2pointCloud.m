function [pointCloudForward, pointCloudOpposite]= scan2pointCloud(img, ...
        max_selected_distance,radar_resolution, max_distance )

% input:
% img: polar image
% max_selected_distance: maximum distance chosen by user in meters
% radar_resolution: m / pixel
% max_distance: maximum distance for each azimuth sacn in meters

max_pixel = floor(max_selected_distance/ max_distance * size(img,2));
% selected_scan = [10:180,220:390];


img = img(:,1:max_pixel);
t = 1: size(img,2);
if max_pixel > 600
    car_reflection = 60;
else
    car_reflection = 30;
end

img(:,1:car_reflection) = 0;
img(:,max_pixel:end) = 0;

allocated_memoery = 10000;
% allocated_memoery = 2000; % defalut

pointCloudForward = zeros(allocated_memoery,3);
pointCloudOpposite = zeros(allocated_memoery,3);
% tic
counter = 1;

for i=1:size(img,1)
% for k=1:numel(selected_scan)
%     i = selected_scan(k);
    scan = double(img(i,:));
    [pks,locs]= findpeaks(scan,t,'MinPeakProminence',8,'Annotate','extents','MinPeakDistance',50);
    
    %% show peaks power distribution 
%     if mod(i, 10) == 0
%         
%         figure;
%         histogram(pks,255);
%     end
    
    peaks_mean = mean(pks); 
    peaks_std = std(pks);
    for j=1:size(pks,2)
        pk = pks(j);
        if (pk > peaks_mean + peaks_std)
            theta =  - i* 2 * pi / 400;
            x = locs(j)*radar_resolution*cos(theta);
            y = locs(j)*radar_resolution*sin(theta);
            z = 1;
            % Forward view
            point = [x y z];
            pointCloudForward(counter,:) = point; 
            
            counter = counter + 1;

        end
    end
end
pointCloudForward = pointCloudForward(1:counter,:);
pointCloudOpposite = pointCloudForward;
pointCloudOpposite(:,1:2) = -pointCloudForward(:,1:2);

% toc
end