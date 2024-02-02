function err = calcSequenceErrors(poses_gt, poses_result,lengths)
% calculate sequence errors
% err = [first_frame, r_err/len, t_err/len, len, speed]
% errors = [err; err; err......]
% lengths = [100,200,300...800]

% 
% % For consistency test
% consistencyTestOdom(poses_gt, poses_result,lengths(2));

distTooshort = true;
counter = 0;

% error vector
err = [];

% parameters
step_size = 4; % every second

% pre-compte distance (from ground truth as reference)
dist = trajectoryDistances(poses_gt);

% for all start positions do
for first_frame = 1:step_size:size(poses_gt,3)
    for i = 1:numel(lengths)
        
        % current length
        len = lengths(i);
        
        % compute last frame
        last_frame = lastFrameFromSegmentLength(dist, first_frame, len);
        
        % continue, if sequence not long enough
        if (last_frame==-1)
            continue
        end

         
        % compute rotational and translational errors
        pose_delta_gt     = inverse_pose(poses_gt(:,:,first_frame))* poses_gt(:,:,last_frame);
        pose_delta_result = inverse_pose(poses_result(:,:,first_frame))*poses_result(:,:,last_frame);
        pose_error        = inverse_pose(pose_delta_result)*pose_delta_gt;
        r_err = rotationError(pose_error);
        t_err = translationError(pose_error);
        
        % compute speed
        num_frames = last_frame-first_frame+1;
        speed = len/(0.25*num_frames);
        
        errors = [first_frame,r_err/len,t_err/len,len,speed];
        
%         if errors(3) > 1.5
% %             first_frame
% %             errors(3)
% %             t_err
% %             len
% % disp("poses_gt(:,:,first_frame)")
% % poses_gt(:,:,first_frame)
% % disp("poses_result(:,:,first_frame)")
% % poses_result(:,:,first_frame)
% % 
% % disp("poses_gt(:,:,last_frame)")
% % poses_gt(:,:,last_frame)
% % disp("poses_result(:,:,last_frame)")
% % poses_result(:,:,last_frame)
% % first_frame
% % last_frame
% % pose_delta_gt
% % pose_delta_result
%         end
        
        
        err = [err; errors];
        
        
        distTooshort = false;
    end
end

if distTooshort
    disp("Distance too short. Fails to init.")
end
end