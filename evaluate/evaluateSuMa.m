function [meanLengthEroor, meanSpeedError, vErrors_ro_all ] = evaluateSuMa(Files, lengths)
disp("Evaluate SuMa")

vLengths_errors = [];
vSpeed_errors = [];
vErrors_ro_all = [];
for f = 3:size(Files,1)
    folderName = strcat( Files(f).folder , '/', Files(f).name, '/');
    sequence = folderName;
    string = strcat(sequence, '*poses.txt');
    SuMa_file = dir(string);
    
    if (~isempty(SuMa_file))
        % Radar Groundtruth
        [poses_gt, radar_timestamps] = readGroudtruthPoses(sequence);
        
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
        figure;
        hold on
        x = reshape(poses_gt(1,3,:),[1 size(poses_gt,3)]);
        y = reshape(poses_gt(2,3,:),[1 size(poses_gt,3)]);
        plot(x, y, 'k--','LineWidth',1,'DisplayName','gt');
        
        x = reshape(poses_suma(1,3,:),[1 size(poses_suma,3)]);
        y = reshape(poses_suma(2,3,:),[1 size(poses_suma,3)]);
        plot(x, y, 'DisplayName','SuMa');
        xlabel('X')
        ylabel('Y')
        hold off
%         % Show 3d trajectoryed
%         figure;
%         X = sumaPoseSE3(1,4,:);
%         Y = sumaPoseSE3(2,4,:);
%         Z = sumaPoseSE3(3,4,:);
%         plot3(X(:),Y(:),Z(:));
%         xlabel('X')
%         ylabel('Y')
%         zlabel('Z')
        
        % Calculate error
        % Per sequence
        [sync_suma_poses, sync_gt_poses] = syncPoses(poses_gt, radar_timestamps, poses_suma, suma_timstamps);
        convert2Poses4ATE(sync_suma_poses, sync_gt_poses, suma_timstamps, sequence, 'suma');
        
        display(SuMa_file.name); 
        title(SuMa_file.name);
        vErrors_ro = calcSequenceErrors(sync_gt_poses, sync_suma_poses,lengths);
        rotation_error = mean(vErrors_ro(:,2))
        translation_error = mean(vErrors_ro(:,3))
        
        if translation_error > 0.1
            continue
        end    
        lengths_errors = zeros(numel(lengths),3);
        lengths_errors(:,1) = linspace(100,800,numel(lengths)); % [lengths, rotation_error, translation_error]
        speed_errors = [];
        
        %% compute error per length
        for i=1:numel(lengths)
            indexs = find(vErrors_ro(:,4)==lengths_errors(i,1));
            rotation_errors = vErrors_ro(indexs,2);
            translation_errors = vErrors_ro(indexs,3);
            lengths_errors(i,2) = mean(rotation_errors);
            lengths_errors(i,3) = mean(translation_errors);
           
        end        
        
        %% compute error per speed
        speed_errors = errorAgainstSpeed(vErrors_ro);        
 
        %% Stack for all sequences
        vLengths_errors = [vLengths_errors; lengths_errors];
        vSpeed_errors = [vSpeed_errors; speed_errors];
        vErrors_ro_all = [vErrors_ro_all; vErrors_ro];        
    end
end

[meanLengthEroor, meanSpeedError] = meanErrorSpeedLength(vLengths_errors, vSpeed_errors, lengths);



end