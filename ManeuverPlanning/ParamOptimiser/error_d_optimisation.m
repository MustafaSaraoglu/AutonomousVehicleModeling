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

% paramsToBeOptimized = [Kp, Ki, Kd, forwardMotionGain] [8.7639, 9.3465, 0.1412, 1.6684] fval = 6.5665e-05

%% Without PID
nvars = 1;
lb = 1;
ub = 10;
MaxGenerations_Data = 5;

% paramsToBeOptimized = forwardMotionGain = 1 fval = 0.0019

%% For reproducibility: https://www.mathworks.com/help/gads/reproducing-your-results-1.html
rng(1,'twister');

%% Modify options setting
options = optimoptions(options,'MaxGenerations', MaxGenerations_Data);
options = optimoptions(options,'Display', 'iter');
options = optimoptions(options,'PlotFcn', { @gaplotbestf });
[x,fval,exitflag,output,population,score] = ...
ga(@error_d,nvars,[],[],[],[],lb,ub,[],[],options);
