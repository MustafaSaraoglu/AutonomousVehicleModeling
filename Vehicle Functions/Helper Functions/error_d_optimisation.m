function [x,fval,exitflag,output,population,score] = error_d_optimisation()
%% This is an auto generated MATLAB file from Optimization Tool.

%% Start with the default options
options = optimoptions('ga');
%% Specific options
nvars = 4;
% paramsToBeOptimized = [Kp, Ki, Kd, forwardMotionGain] [9.9326, 9.5739, 1.7831, 3.4715]
lb = [0, 0, 0, 1];
ub = [10, 10, 10, 3.5];
MaxGenerations_Data = 10;

rng(1,'twister'), % for reproducibility: https://www.mathworks.com/help/gads/reproducing-your-results-1.html

%% Modify options setting
options = optimoptions(options,'MaxGenerations', MaxGenerations_Data);
options = optimoptions(options,'Display', 'iter');
options = optimoptions(options,'PlotFcn', { @gaplotbestf });
[x,fval,exitflag,output,population,score] = ...
ga(@error_d,nvars,[],[],[],[],lb,ub,[],[],options);
