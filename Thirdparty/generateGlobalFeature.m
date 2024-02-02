%#codegen
function [featureForward, pointCloudForward, featureReverse, pointCloudOpposite] = generateGlobalFeature(img, max_selected_distance, range_resolution, max_distance)

% Pre-process the polar image
[pointCloudForward, pointCloudOpposite] = scan2pointCloud(img, max_selected_distance,range_resolution,max_distance);

% Compute forward descriptor
[featureForward, ~] = M2DP(pointCloudForward);


% Compute forward descriptor
[featureReverse, ~] = M2DP(pointCloudOpposite);
