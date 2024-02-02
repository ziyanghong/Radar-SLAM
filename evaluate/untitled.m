odom = readmatrix('/media/hong/DiskD/RADIATE_DATASET/Loops/2_2020-01-23-21-43-57/ro/odometry/3/our_result_odometry.csv');
figure;
plot(odom(:,2), odom(:,3));
