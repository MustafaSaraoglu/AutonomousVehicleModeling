function simulate_and_plot(options)
%% default
arguments
    options.position                       (1, :) double = [40, 20];  % e.g. vehicle 1 starts at 40 m, vehicle 2 at 20 m 
    options.speed                          (1, :) double = [20, 25];  % e.g. vehicle 1 starts at 20 m/s, vehicle 2 at 25 m/s 
    options.leader_config                  (1, :) double = [10, 0];   % vehicle 1's reference speed is 10 m/s, it has a extra 0 amplitude sine-wave acceleration by default 
    options.controller                     (1, 1) string = 'MathWorksMPC'; % default controller is MathWorks MPC, the other options are:
                                                                           % MPC: 'MPC_quadprog', 'MPC_quadprog_wKalman', 'MPC_fmincon', 'MPC_fmincon_wKalman';
                                                                           % PID: 'PID'   
    options.controller_behavior_MathWorks  (1, 1) double = 0.5;  % default controller behavior for MathWorks MPC is 0.5, the range is [0, 1]
end
%% load and open system
close all;
load_system('VehicleFollowing');
open_system('VehicleFollowing');
%% parameter setting
leader.position = num2str(options.position(1));
leader.speed = num2str(options.speed(1));
leader.speed_reference = num2str(options.leader_config(1));
leader.speed_disturbance = num2str(options.leader_config(2));
ego.position = num2str(options.position(2));
ego.speed = num2str(options.speed(2));
ego.controller = options.controller;
% set the parameters of Vehicle Model 1; speed disturbance is sine wave and
% its amplitude is set to 0 by default
set_param('VehicleFollowing/Vehicle Model 1 - Leader','Speed', leader.speed);
set_param('VehicleFollowing/Vehicle Model 1 - Leader','Pos', leader.position);
set_param('VehicleFollowing/Vehicle Model 1 - Leader','Disturbance', leader.speed_disturbance);
set_param('VehicleFollowing/Speed Reference','Value', leader.speed_reference)
% set the controller for Vehicle Model 2
blockname = 'VehicleFollowing/Vehicle Model 2 - Following/Variant Vehicle Following Mode';
p = Simulink.Mask.get(blockname);
set_param(blockname,'LabelModeActiveChoice', ego.controller);
controller_behavior_MathWorksMPC = num2str(options.controller_behavior_MathWorks);
controller_set = p.Parameters(1);
controller_set.set('Value', ego.controller);
% set the controller behavior for MathWorks MPC
control_behavior = p.Parameters(3);
control_behavior.set('Value', controller_behavior_MathWorksMPC);
% set the initial speed and position for Vehicle Model 2
set_param('VehicleFollowing/Vehicle Model 2 - Following', 'Speed', ego.speed);
set_param('VehicleFollowing/Vehicle Model 2 - Following', 'Pos', ego.position);
%% run the simulation
sim('VehicleFollowing');
end