function error = translationError(pose_error)
    % return error in absolute distance
    dx = pose_error(1,3);
    dy = pose_error(2,3);
    error = sqrt(dx*dx+dy*dy);
end