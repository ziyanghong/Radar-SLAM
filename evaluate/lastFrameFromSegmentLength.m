function idx =  lastFrameFromSegmentLength(dist, first_frame, len)
idx = -1;    
for i = first_frame:size(dist) 
    if(dist(i) > dist(first_frame) + len)
        idx = i;
        break;
    end
end

end


