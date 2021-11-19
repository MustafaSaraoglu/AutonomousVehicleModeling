function [x,fval,exitflag,output,population,score] = error_d_optimisation()
%% This is an auto generated MATLAB file from Optimization Tool.

%% Start with the default options
options = optimoptions('ga');
%% Specific options

%% With PID
% nvars = 4;
% lb = [0, 0, 0, 1];
% ub = [10, 10, 10, 3.5];
% MaxGenerations_Data = 10;

% paramsToBeOptimized = [Kp, Ki, Kd, forwardMotionGain] [9.9326, 9.5739, 1.7831, 3.4715] % WITHOUT DEG TO RAD Stanley
% [9.7639, 8.0739, 0.1865, 1.5419] % WITH DEG TO RAD Stanley

%% Without PID
nvars = 1;
lb = 1;
ub = 5;
MaxGenerations_Data = 5;

% paramsToBeOptimized = forwardMotionGain = 1

%% For reproducibility: https://www.mathworks.com/help/gads/reproducing-your-results-1.html
rng(1,'twister');

%% Modify options setting
options = optimoptions(options,'MaxGenerations', MaxGenerations_Data);
options = optimoptions(options,'Display', 'iter');
options = optimoptions(options,'PlotFcn', { @gaplotbestf });
[x,fval,exitflag,output,population,score] = ...
ga(@error_d,nvars,[],[],[],[],lb,ub,[],[],options);
