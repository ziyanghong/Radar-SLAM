function [meanLengthEroor, meanSpeedError] = meanErrorSpeedLength(vLengths_errors, vSpeed_errors, lengths)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% meanLengthEroor = [length1, rotation_error, translation_error; ...]
% meanSpeedError  = [speeds,  rotation_error, translation_error; ...]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

meanLengthEroor = zeros(numel(lengths), 3);
meanLengthEroor(:,1) = lengths;
step = 7;
meanSpeedError = zeros(step, 3);

% Lengths
for i= 1:numel(lengths)
    length = lengths(i);
    
    indexs = find(vLengths_errors(:,1)==length);
    rotation_errors = vLengths_errors(indexs,2);
    translation_errors = vLengths_errors(indexs,3);
    meanLengthEroor(i,2) = mean(rotation_errors);
    meanLengthEroor(i,3) = mean(translation_errors);
    
end

% Speed
min_speed = min(vSpeed_errors(:,3));
max_speed = max(vSpeed_errors(:,3));

speeds = linspace(min_speed,max_speed,step)';
meanSpeedError(:,1) = speeds;
V = vSpeed_errors(:,3);
N = speeds;
[~, closestIndex] = min(abs(N - V.'));
for i= 1:numel(speeds)
    indexs = find(closestIndex(:)==i);
    rotation_errors = vSpeed_errors(indexs,1);
    translation_errors = vSpeed_errors(indexs,2);
    meanSpeedError(i,2) = mean(rotation_errors);
    meanSpeedError(i,3) = mean(translation_errors);    
    
end

end