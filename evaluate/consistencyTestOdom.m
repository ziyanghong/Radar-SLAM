function err_list = consistencyTestOdom(poses_gt, poses_result, lenght)

err_list = [];
dist = 0.0;
for i=10:size(poses_gt,3)
    % Distance
    P1 = poses_gt(:,:,i-1);
    P2 = poses_gt(:,:,i);
    dx = P1(1,3) - P2(1,3);
    dy = P1(2,3) - P2(2,3);
    dist = dist + sqrt(dx*dx+dy*dy);
    
    % Pose error
    pose_delta_gt     = inverse_pose(poses_gt(:,:,i-1))* poses_gt(:,:,i);
    pose_delta_result = inverse_pose(poses_result(:,:,i-1))*poses_result(:,:,i);
    pose_error        = inverse_pose(pose_delta_result)*pose_delta_gt;
    
    % return error in degree
    yaw = atan2(pose_error(2,1), pose_error(1,1));
    r_err = rad2deg(yaw);
    
    % Position error in Longitudinal motion
    x_err = pose_error(1,3);
    % Position error in lateral motion
    y_err = pose_error(2,3);    
    
    errors = [r_err, x_err, y_err];
    err_list = [err_list; errors];

    if dist >= lenght
        break
    end
end

n = numel(err_list(:,1));
k_std = 3;



r_std = std(err_list(:,1));
r_mean = mean(err_list(:,1));
fig1 = figure; hold on;
plot(err_list(:,1),'DisplayName','error');
plot(repelem(r_mean,n),'DisplayName','error mean');
plot(repelem(r_mean + k_std*r_std,n),'g');
plot(repelem(r_mean - k_std*r_std,n),'g');
title(strcat('Standard deviation = ', num2str(r_std)));
% saveas(fig1,'iros_rotation_consistency.pdf')
hold off;

x_std = std(err_list(:,2));
x_mean = mean(err_list(:,2));
fig2 = figure; hold on;
plot(err_list(:,2),'DisplayName','error');
plot(repelem(x_mean, n),'DisplayName','error mean');
plot(repelem(x_mean + k_std*x_std,n),'g');
plot(repelem(x_mean - k_std*x_std,n),'g');
title(strcat('Standard deviation = ', num2str(x_std)));
% saveas(fig1,'iros_long_consistency.pdf')

hold off;

y_std = std(err_list(:,3));
y_mean = mean(err_list(:,3));
figure; hold on;
plot(err_list(:,3),'DisplayName','error');
plot(repelem(y_mean, n),'DisplayName','error mean');
plot(repelem(y_mean + k_std*y_std,n),'g');
plot(repelem(y_mean - k_std*y_std,n),'g');
title(strcat('Standard deviation = ', num2str(y_std)));
% saveas(fig1,'iros_lat_consistency.pdf');
hold off;

end