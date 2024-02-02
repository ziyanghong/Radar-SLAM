function [meanLengthEroor, meanSpeedError, vErrors_ro_all ] = evaluateORB(Files, lengths)
disp("Evaluate ORB stereo")

vLengths_errors = [];
vSpeed_errors = [];
vErrors_ro_all = [];
for f = 3:size(Files,1)
    
    folderName = strcat( Files(f).folder , '/', Files(f).name, '/');
    stereo_folder = strcat(folderName ,'stereo/');
    
    if (isfolder(folderName) && exist(stereo_folder, 'dir'))
        
        sequence = folderName
        
        % Radar Groundtruth
        [poses_gt, radar_timestamps] = readGroudtruthPoses(sequence);
        
        % Stereo pose
        % read matrix
        CameraTrajectoryORB = strcat(stereo_folder, 'CameraTrajectoryKITTI.txt');
        if isfile(CameraTrajectoryORB)
            orb_pose6D = readmatrix(CameraTrajectoryORB);
            timestamp_file = strcat(sequence,'stereo.timestamps');
            M = dlmread(timestamp_file, ' ');
            image_timestamps = M(:,1);
            image_timestamps = image_timestamps(1:size(orb_pose6D,1));
            poses_stereo = zeros(3,3,size(orb_pose6D,1));
            previousPose = eye(4);
            pose = eye(3);
            
            orbPoseSE3 = zeros(3,4,size(orb_pose6D,1));
            orbAngles  = zeros(size(orb_pose6D,1), 3);
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
                %
                
                %             eulZYX = rotm2eul(pose6D(1:3,1:3));
                %             orbAngles(i,:) = eulZYX;
                %             % swap x and z here
                %             pose = [cos(-eulZYX(2)) -sin(-eulZYX(2))  pose6D(3,4);
                %                     sin(-eulZYX(2))  cos(-eulZYX(2)) -pose6D(1,4);
                %                     0               0                        1];
                
                
                
                poses_stereo(:,:,i) = pose;
                
                
            end
            figure;
            x = reshape(poses_stereo(1,3,:),[1 size(poses_stereo,3)]);
            y = reshape(poses_stereo(2,3,:),[1 size(poses_stereo,3)]);
            plot(x, y, 'DisplayName','stereo');
            xlabel('X')
            ylabel('Y')
            
            %         figure;
            %         X = orbPoseSE3(1,4,:);
            %         Y = orbPoseSE3(2,4,:);
            %         Z = orbPoseSE3(3,4,:);
            %         plot3(X(:),Y(:),Z(:));
            %         xlabel('X')
            %         ylabel('Y')
            %         zlabel('Z')
            
            % Calculate error
            % Per sequence
            [sync_stereo_poses, sync_gt_poses] = syncPoses(poses_gt, radar_timestamps, poses_stereo, image_timestamps);
            convert2Poses4ATE(sync_stereo_poses, sync_gt_poses, image_timestamps, sequence, 'orb');
            
            vErrors_ro = calcSequenceErrors(sync_gt_poses, sync_stereo_poses,lengths);
            if size(vErrors_ro,1) >=1
                rotation_error = mean(vErrors_ro(:,2))
                translation_error = mean(vErrors_ro(:,3))
                
                
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
    end
end

[meanLengthEroor, meanSpeedError] = meanErrorSpeedLength(vLengths_errors, vSpeed_errors, lengths);


end