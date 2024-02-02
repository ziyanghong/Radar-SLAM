function inv_pose = inverse_pose(pose)
    Rcw = pose(1:2,1:2);
    tcw = pose(1:2,3);
    Rwc = Rcw';
    Ow = -Rwc*tcw;
    Twc = eye(3);
    Twc(1:2,1:2) = Rwc;
    Twc(1:2,3) = Ow; 
    inv_pose = Twc;
    
    

end