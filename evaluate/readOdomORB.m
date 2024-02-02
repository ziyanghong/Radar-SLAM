function [orb_xy, orb_timestamp] = readOdomORB(sequence)

stereo_folder = strcat(sequence ,'stereo/');

% read matrix
CameraTrajectoryORB = strcat(stereo_folder, 'OdomTrajectory.txt');
orb_pose6D = readmatrix(CameraTrajectoryORB);
timestamp_file = strcat(sequence,'stereo.timestamps');
M = dlmread(timestamp_file, ' ');
timestamps = M(:,1);
orb_timestamp = timestamps(1:size(orb_pose6D,1));
poses_stereo = zeros(3,3,size(orb_pose6D,1));
previousPose = eye(4);
pose = eye(3);

orbPoseSE3 = zeros(3,4,size(orb_pose6D,1));
% Loop
for i = 1:size(orb_pose6D,1)
    pose6D = reshape(orb_pose6D(i,:),4,3)';
    
    
    orbPoseSE3(:,:,i) = (pose6D);
    currentPose = eye(4);
    currentPose(1:3,1:4) = pose6D;
    relativePoseSE3 = computeRelativePose6DOF(previousPose, currentPose);
    previousPose = currentPose;
    eulZYX = rotm2eul(relativePoseSE3(1:3,1:3));
    % swap x and y here
    relatposeSE2 = [cos(-eulZYX(2)) -sin(-eulZYX(2)) relativePoseSE3(3,4);
        sin(-eulZYX(2))  cos(-eulZYX(2)) relativePoseSE3(1,4);
        0               0                        1];
    pose = pose * relatposeSE2;
    poses_stereo(:,:,i) = pose;
    
    
end
x = reshape(poses_stereo(1,3,:),[1 size(poses_stereo,3)]);
y = reshape(poses_stereo(2,3,:),[1 size(poses_stereo,3)]);
orb_xy = [x;y];
end