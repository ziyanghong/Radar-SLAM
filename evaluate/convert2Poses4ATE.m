function convert2Poses4ATE(x_sync_poses, gt_sync_poses, x_timestamps, sequence, algorithm) 
newStr = split(sequence,'/');
ate_result_path = '/home/hong/Documents/rpg_trajectory_evaluation/results/';
if contains(sequence, 'MulRan')
    ate_result_path = strcat(ate_result_path, 'mulran/pc/', algorithm, ...
        '/pc_', algorithm, '_', convertCharsToStrings(newStr(end-1)));
elseif contains(sequence, 'Oxford')
    ate_result_path = strcat(ate_result_path, 'oxford/pc/', algorithm, ....
        '/pc_', algorithm, '_', convertCharsToStrings(newStr(end-1)));
end

% Trajecotry 
ate_result_folder = strcat(ate_result_path, '/');
mkdir(ate_result_folder);
output_pose_file =  strcat(ate_result_folder, 'stamped_traj_estimate.txt');
output_gt_file = strcat(ate_result_folder, 'stamped_groundtruth.txt');
fileID1 = fopen(output_pose_file, 'w');
fileID2 = fopen(output_gt_file, 'w');
formatSpec = '%d %d %d %d %d %d %d %d\n';

for i = 1:size(x_sync_poses,3)
% for i = 277
    
    t = x_timestamps(i);
    x_sync_pose =  x_sync_poses(:,:,i);
    gt_sync_pose = gt_sync_poses(:,:,i);
    
    q_x = eul2quat([real(acos(x_sync_pose(1,1))), 0, 0]);
    q_gt= eul2quat([real(acos(gt_sync_pose(1,1))), 0, 0]);
    fprintf(fileID1, formatSpec,t, x_sync_pose(1,3), x_sync_pose(2,3), 0.0, q_x(2), q_x(3), q_x(4), q_gt(1));
    fprintf(fileID2, formatSpec,t, gt_sync_pose(1,3), gt_sync_pose(2,3), 0.0, q_gt(2), q_gt(3), q_gt(4), q_gt(1));
end

% Config yaml
yaml_file_name = strcat(ate_result_folder, 'eval_cfg.yaml');
fileID = fopen(yaml_file_name, 'w');

if contains(sequence, 'Oxford') && contains(algorithm, 'suma')
    fprintf(fileID, '%s\n', 'align_type: none');
elseif contains(algorithm, 'odom')
    fprintf(fileID, '%s\n', 'align_type: none');
else
    fprintf(fileID, '%s\n', 'align_type: se3');
end
fprintf(fileID, '%s', 'align_num_frames: -1');
end