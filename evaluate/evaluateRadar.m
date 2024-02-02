function [mean_length_error_kf, mean_speed_error_kf, ...
          mean_length_error_odom, mean_speed_error_odom, ... 
          vErrors_keyframe_all, vErrors_odom_all ] = evaluateRadar(Files, lengths, ro_folder)
disp("Evaluate Radar method")
% all_sequences_Error_t_keyframe = [];
% all_sequences_Error_r_keyframe = [];
% all_sequences_Error_t_odom = [];
% all_sequences_Error_r_odom = [];
vErrors_keyframe_all = [];
vErrors_odom_all = [];
vlengths_errors_kf = [];
vspeed_errors_kf = [];
vlengths_errors_odom = [];
vspeed_errors_odom = [];

for f = 3:size(Files,1)
    % for f = 5
    % for f = 3
%  for f= 16
% for f= 10

    folderName = strcat( Files(f).folder , '/', Files(f).name, '/');
    
    if (isfolder(folderName))
        
        sequence = folderName;
        %         sequence = '/home/hong/Documents/Oxford_Radar_RobotCar_Dataset/2019-01-16-13-09-37-radar-oxford-10k/';
        %         sequence = '/media/hong/DiskB/MulRan_Dataset/KAIST02/';
        
        % Radar Groundtruth
        [poses_gt, radar_timestamps] = readGroudtruthPoses(sequence);
        
        % Per sequence
        [vErrors_keyframe, vErrors_ro, ~, ~, ~, ~,...
            lengths_errors_kf, speed_errors_kf,lengths_errors_odom, speed_errors_odom] = evaluateOneSequenceRadar(sequence, poses_gt, lengths, ro_folder);
        vErrors_keyframe_all = [vErrors_keyframe_all; vErrors_keyframe];
        vErrors_odom_all = [vErrors_odom_all; vErrors_ro];
        
        % Stack for all sequences
        vlengths_errors_kf = [vlengths_errors_kf; lengths_errors_kf];
        vspeed_errors_kf = [vspeed_errors_kf; speed_errors_kf];
        vlengths_errors_odom = [vlengths_errors_odom; lengths_errors_odom];
        vspeed_errors_odom = [vspeed_errors_odom; speed_errors_odom];
        
        %% Use this if a sequence is tested multiple times
%         one_sequence_mean_Error_t_keyframe = mean(vError_t_keyframe);
%         one_sequence_mean_Error_r_keyframe = mean(vError_r_keyframe);
%         one_sequence_mean_Error_t_odom     = mean(vError_t_odom);
%         one_sequence_mean_Error_r_odom     = mean(vError_r_odom);
        
%         all_sequences_Error_t_keyframe = [all_sequences_Error_t_keyframe;one_sequence_mean_Error_t_keyframe];
%         all_sequences_Error_r_keyframe = [all_sequences_Error_r_keyframe;one_sequence_mean_Error_r_keyframe];
%         all_sequences_Error_t_odom = [all_sequences_Error_t_odom; one_sequence_mean_Error_t_odom];
%         all_sequences_Error_r_odom = [all_sequences_Error_r_odom; one_sequence_mean_Error_r_odom];
    end
end


[mean_length_error_kf, mean_speed_error_kf] = meanErrorSpeedLength(vlengths_errors_kf, vspeed_errors_kf, lengths);
[mean_length_error_odom, mean_speed_error_odom] = meanErrorSpeedLength(vlengths_errors_odom, vspeed_errors_odom, lengths);


%% Get the final result
% all_sequences_mean_Error_t_keyframe = mean(all_sequences_Error_t_keyframe);
% all_sequences_std_Error_t_keyframe  = std(all_sequences_Error_t_keyframe);
% all_sequences_mean_Error_r_keyframe = mean(all_sequences_Error_r_keyframe);
% all_sequences_std_Error_r_keyframe  = std(all_sequences_Error_r_keyframe);
% all_sequences_mean_Error_t_odom = mean(all_sequences_Error_t_odom);
% all_sequences_std_Error_t_odom  = std(all_sequences_Error_t_odom);
% all_sequences_mean_Error_r_odom = mean(all_sequences_Error_r_odom);
% all_sequences_std_Error_r_odom  = std(all_sequences_Error_r_odom);




end