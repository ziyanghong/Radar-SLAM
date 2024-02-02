function dist =  trajectoryDistances(poses)

dist = zeros(size(poses,3));

for i=2:size(poses,3)
    P1 = poses(:,:,i-1);
    P2 = poses(:,:,i);
    dx = P1(1,3) - P2(1,3);
    dy = P1(2,3) - P2(2,3);
    dist(i) = dist(i-1) + sqrt(dx*dx+dy*dy);
end
end