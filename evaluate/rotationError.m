function error = rotationError(pose_error)
    % return error in degree
    yaw = atan2(pose_error(2,1), pose_error(1,1));
    error = abs(rad2deg(yaw));
end