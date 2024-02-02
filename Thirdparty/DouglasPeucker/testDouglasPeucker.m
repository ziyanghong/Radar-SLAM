%% Test Douglas–Peucker algorithm 
% The Ramer–Douglas–Peucker algorithm (RDP) is an algorithm for reducing 
% the number of points in a curve that is approximated by a series of 
% points. The initial form of the algorithm was independently suggested 
% in 1972 by Urs Ramer and 1973 by David Douglas and Thomas Peucker and 
% several others in the following decade. This algorithm is also known 
% under the names Douglas–Peucker algorithm, iterative end-point fit 
% algorithm and split-and-merge algorithm. [Source Wikipedia]
%
%
% -------------------------------------------------------
% Code: Reza Ahmadzadeh (2017) 
% -------------------------------------------------------
clc,clear,close all
Points = [1 2 3 4 5 6 7 8;
    3 2.3 1 2.1 2.2 2 1.5 3.2];

figure;
ii = 1;
subplot(2,3,ii);hold on
plot(Points(1,:),Points(2,:),'-or');
title('initial points');
grid on;box on;axis([1 8 0.5 3.5]);

for epsilon = [0.1:0.1:0.3 1.5 2.5]
    res = DouglasPeucker(Points,epsilon);
    ii = ii + 1;
    subplot(2,3,ii);hold on
    plot(Points(1,:),Points(2,:),'--or');
    plot(res(1,:),res(2,:),'-xb');
    title(['\epsilon = ' num2str(epsilon)]);
    grid on;box on;axis([1 8 0.5 3.5]);
    legend('original','approximated')
end

