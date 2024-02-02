function [suma_xy, suma_timstamps]= readOdomSuma(sequence)

string = strcat(sequence, '*poses.txt');
SuMa_file = dir(string);



% SuMa odometry
pose = eye(3);
if contains(sequence, 'MulRan')
    timestamp_file = strcat(sequence,'ouster_front_stamp.csv');
    M = dlmread(timestamp_file);
    lidar_timstamps = M(:,1);
elseif contains(sequence, 'Oxford')
    timestamp_file = strcat(sequence,'velodyne_left.timestamps');
    M = dlmread(timestamp_file, ' ');
    lidar_timstamps = M(:,1);
end
suma_pose_file = strcat(SuMa_file.folder, '/', SuMa_file.name);
lo = readmatrix(suma_pose_file);
suma_timstamps = lidar_timstamps(1:size(lo,1));
poses_suma = zeros(3,3,size(lo,1));
sumaPoseSE3 = zeros(3,4,size(lo,1));
previousPose = eye(4);


for i = 1:size(lo,1)
    pose6D = reshape(lo(i,:),4,3)';
    sumaPoseSE3(:,:,i) = (pose6D);
    
    currentPose = eye(4);
    currentPose(1:3,1:4) = pose6D;
    relativePoseSE3 = computeRelativePose6DOF(previousPose, currentPose);
    previousPose = currentPose;
    eulZYX = rotm2eul(relativePoseSE3(1:3,1:3));
    if contains(sequence, 'MulRan')
        relatposeSE2 = [cos(eulZYX(1)) -sin(eulZYX(1)) -relativePoseSE3(1,4);
            sin(eulZYX(1))  cos(eulZYX(1)) relativePoseSE3(2,4);
            0               0                        1];
    elseif contains(sequence, 'Oxford')
        relatposeSE2 = [cos(eulZYX(1)) -sin(eulZYX(1)) relativePoseSE3(1,4);
            sin(eulZYX(1))  cos(eulZYX(1)) relativePoseSE3(2,4);
            0               0                        1];
    end
    
    pose = pose * relatposeSE2;
    poses_suma(:,:,i) = pose;
    
end


x = reshape(poses_suma(1,3,:),[1 size(poses_suma,3)]);
y = reshape(poses_suma(2,3,:),[1 size(poses_suma,3)]);
suma_xy = [x;y];

end