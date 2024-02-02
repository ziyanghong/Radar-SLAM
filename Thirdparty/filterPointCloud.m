function filteredPtCloud = filterPointCloud(accumulated_points,topK)
ptCloud = pointCloud(accumulated_points);
distThresholdSegmentation = 0.9;
[labels,numClusters] = pcsegdist(ptCloud,distThresholdSegmentation);
labelColorIndex = labels;
[frequency,edges] = histcounts(labels,numClusters);
[B,I] = sort(frequency,'descend');
Locations = [];
labelColorIndexTopK = [];
for i=1:topK
    indexs = find(labels==I(i));
    locs = ptCloud.Location(indexs,:);
    Locations = [Locations; locs];
    indx = repmat(i, size(indexs,1),1);
    labelColorIndexTopK = [labelColorIndexTopK; indx]; 
end


figure;
pcshow(ptCloud.Location,labelColorIndex)
colormap([hsv(numClusters);[0 0 0]])
title('Point Cloud Clusters')
figure;
pcshow(Locations, labelColorIndexTopK);
% colormap([hsv(5);[0 0 0]])
title('Point Cloud K Clusters')
filteredPtCloud = Locations;
end