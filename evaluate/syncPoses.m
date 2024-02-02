function [x_sync_poses, gt_sync_poses] = syncPoses(gt_poses, gt_timestamps, x_poses, x_timestamps)

if numel(x_timestamps) >= numel(gt_timestamps)
    x_sync_poses = zeros(3,3,size(gt_timestamps,1));
    gt_sync_poses = gt_poses;
    for i= 1:size(gt_timestamps)
        [~, idx] = min(abs(gt_timestamps(i) - x_timestamps));
        x_sync_poses(:,:,i) = x_poses(:,:,idx);
        
    end
    
else
    gt_sync_poses = zeros(3,3,size(x_timestamps,1));
    x_sync_poses = x_poses;
    for i= 1:size(x_timestamps,1)
        [~, idx] = min(abs(x_timestamps(i) - gt_timestamps));
        gt_sync_poses(:,:,i) = gt_poses(:,:,idx);
    end
end

end