function [x_sync_xy, sync_data_idxs] = syncTranslation(gt_timestamps, x_xy, x_timestamps)
% if numel(x_timestamps) >= numel(gt_timestamps)
    x_sync_xy = zeros(2,size(gt_timestamps,1));
%     gt_sync_xy = gt_xy;
    sync_data_idxs = zeros(1,size(gt_timestamps,1));
    
    for i= 1:size(gt_timestamps)
        [~, idx] = min(abs(gt_timestamps(i) - x_timestamps));
        x_sync_xy(:,i) = x_xy(:,idx);
        sync_data_idxs(i) = idx;
    end
    
% else
%     gt_sync_xy = zeros(2,size(x_timestamps,1));
%     x_sync_xy = x_xy;
%     for i= 1:size(x_timestamps,1)
%         [~, idx] = min(abs(x_timestamps(i) - gt_timestamps));
%         gt_sync_xy(:,i) = gt_xy(:,idx);
%     end
% end

end