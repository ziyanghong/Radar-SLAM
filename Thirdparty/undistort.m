function point_cloud_undistorted = undistort(point_cloud, relative_pose, time_gap) 

velocity = [relative_pose(1,3)/time_gap, relative_pose(2,3)/time_gap, atan2(relative_pose(2,1),relative_pose(1,1))/time_gap];
factorT0 = 0.5; % The factor of T(t=0)
point_cloud_undistorted = [];
for i=1:length(point_cloud)
    
    point = point_cloud(i,:);
    theta = atan2(point(2), point(1));
    factorT = 0;
    if (theta > 0)
        
        factorT = ((pi * 2 - theta)) / (2 * pi);
        
    else
        
        factorT = (-theta) / (2 * pi);
        
    end
    deltaT = time_gap * (factorT - factorT0);
    c = cos(velocity(3)*deltaT);
    s = sin(velocity(3)*deltaT);
    x = velocity(1)*deltaT;
    y = velocity(2)*deltaT;
    RT = [c, -s, x;
          s,  c, y;
          0,  0, 1];
    point_new = RT * point';
    point_cloud_undistorted = [point_cloud_undistorted; point_new'];
    
    
    
    
end

% point_cloud_undistorted = point_cloud;

% figure;
% grid on;
% scatter(point_cloud(:,1),point_cloud(:,2),'b.');
% title('Input  point cloud')
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% hold on;
% % subplot(122);grid on
% scatter(point_cloud_undistorted(:,1),point_cloud_undistorted(:,2),'r.');
% title('Undistorted point cloud')
% xlabel('X');
% ylabel('Y');
% zlabel('Z');

end