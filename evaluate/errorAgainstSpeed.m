function speed_errors = errorAgainstSpeed(vErrors)
% speed_err = [r_err, t_err, speed(km/h)]

speed_errors = [vErrors(:,2), vErrors(:,3), vErrors(:,5)*3.6];

end