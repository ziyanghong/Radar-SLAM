close all;
dataset_path = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/';

%% ---------------------------------------Parameters study ------------------------------------------
display('parameters result.');
% para_name = 'clique';
% parameters = 3:1:12;
% xlab_str = 'Clique in pixel';

% para_name = 'hessian';
% parameters = 500:50:950;
% xlab_str = 'Minimum Hessian';

% para_name = 'points';
% parameters = 45:5:90;
% xlab_str = 'Maximum tracked points';

% para_name = 'keydistance';
% parameters = 1.0:0.2:2.8;
% xlab_str = 'Keyframe translation distance';

para_name = 'keyrotation';
parameters = 0.1:0.02:0.28;
xlab_str = 'Keyframe rotation angle';

ro_folder = strcat('ro/', para_name);
sequence = strcat(dataset_path, '2019-01-11-14-02-26-radar-oxford-10k/');
odomParameterStudy(sequence, ro_folder, parameters, xlab_str);