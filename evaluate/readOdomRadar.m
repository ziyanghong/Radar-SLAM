function [radar_xy,radar_timestamps]= readOdomRadar(sequence, ro_folder)

timestamp_file = strcat(sequence, 'radar.timestamps');
M = dlmread(timestamp_file, ' ');
radar_timestamps = M(:,1);


ro_result_path = strcat(sequence, ro_folder, 'odometry/');

% hold off;

Files = dir(ro_result_path);



ro_file = strcat(ro_result_path, Files(3).name ,'/our_result_odometry.csv');
ro = readmatrix(ro_file);
ro = sortrows(ro);
yaws_ro = ro(:,4)';

poses_ro = zeros(3,3,size(ro,1));
for i = 1:size(ro,1)
    poses_ro(:,:,i) = [cos(yaws_ro(i)) -sin(yaws_ro(i)) ro(i,2);
        sin(yaws_ro(i))  cos(yaws_ro(i)) ro(i,3);
        0                0                   1   ];
end

x = reshape(poses_ro(1,3,:),[1 size(ro,1)]);
y = reshape(poses_ro(2,3,:),[1 size(ro,1)]);
radar_xy = [x;y];


end