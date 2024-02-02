% Running sejong long sequence is problematic so I split it and join the
% odometry files togather.

% odometry_file_1 = '/mnt/gpuServerFolder/diskB/MulRan_Dataset/Sejong01/ro/odometry/1/our_result_odometry_1-10028.csv';
% odometry_file_2 = '/mnt/gpuServerFolder/diskB/MulRan_Dataset/Sejong01/ro/odometry/1/our_result_odometry_10028-11509.csv';
% output_odom_csv = '/mnt/gpuServerFolder/diskB/MulRan_Dataset/Sejong02/ro/odometry/1/our_result_odometry.csv';

odometry_file_1 = '/mnt/gpuServerFolder/diskB/MulRan_Dataset/Sejong02/ro/odometry/1/our_result_odometry_1-10383.csv';
odometry_file_2 = '/mnt/gpuServerFolder/diskB/MulRan_Dataset/Sejong02/ro/odometry/1/our_result_odometry_10383-10978.csv';
output_odom_csv = '/mnt/gpuServerFolder/diskB/MulRan_Dataset/Sejong02/ro/odometry/1/our_result_odometry.csv';

odom_1 = readmatrix(odometry_file_1);
[row_1,~] = size(odom_1);
odom_2 = readmatrix(odometry_file_2);
[row_2,~] = size(odom_2);
odom = [odom_1; odom_2];
odom_1_end_pose = se2_to_SE2(odom_1(row_1, 2:4));
for i = 1:row_2
    new_pose = odom_1_end_pose * se2_to_SE2(odom_2(i, 2:4));
    new_se2 = SE2_to_se2(new_pose);
    odom(row_1 + i,:) = [row_1 + i, new_se2];
end

writematrix(odom, output_odom_csv);
figure;
plot(odom(:,2), odom(:,3));
