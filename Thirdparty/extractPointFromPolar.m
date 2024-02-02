function [pointCloudForward]= extractPointFromPolar(img, ...
        max_selected_distance,radar_resolution, max_distance, num_points_per_beam )

% input:
% img: polar image
% max_selected_distance: maximum distance chosen by user in meters
% radar_resolution: m / pixel
% max_distance: maximum distance for each azimuth sacn in meters

max_pixel = floor(max_selected_distance/ max_distance * size(img,2));
selected_scan = [10:180,220:390];


img = img(:,1:max_pixel);
t = 1: size(img,2);
if max_pixel > 600
    car_reflection = 11+60;
else
    car_reflection = 30;
end

img(:,1:car_reflection) = 0;
img(:,max_pixel:end) = 0;

allocated_memoery = 10000;
% allocated_memoery = 2000; % defalut

pointCloudForward = zeros(allocated_memoery,3);
% tic
counter = 1;

% for i=1:size(img,1)
for k=1:numel(selected_scan)
    i = selected_scan(k);
    scan = double(img(i,:));
%     tic
    [pks,locs]= findpeaks(scan,t,'MinPeakProminence',8,'Annotate','extents','MinPeakDistance',50);
%     toc
    peaks_mean = mean(pks); 
    peaks_std = std(pks);
    num_points_per_beam_counter = 0;
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
            num_points_per_beam_counter = num_points_per_beam_counter + 1;
            counter = counter + 1;
            if num_points_per_beam_counter >= num_points_per_beam
                break
            end
        end
    end
end
pointCloudForward = pointCloudForward(1:counter,:);


% toc
end