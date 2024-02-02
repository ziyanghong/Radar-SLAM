function [points,selectedPoses, vXY] = accumulatePoints(vPoint_cloud, vIndex, poses, relative_poses, timestamps)
points = [];
selectedPoses = [];
vXY = [];
for i = 1:length(vIndex)
    point_cloud  = vPoint_cloud{i};
    point_cloud_undistort = undistort(point_cloud, relative_poses{vIndex(i)}, timestamps(vIndex(i),4)-  timestamps(vIndex(i)-1,4));
    point_cloud = poses{vIndex(i)}*point_cloud_undistort';
    selectedPoses = [selectedPoses, {poses{vIndex(i)}}];
    points = [points;point_cloud'];
    vXY = [vXY; poses{vIndex(i)}(1,3), poses{vIndex(i)}(2,3)];
end

end 