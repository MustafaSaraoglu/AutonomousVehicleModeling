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
%% data logging
Simulink.sdi.markSignalForStreaming('VehicleFollowing/Vehicle Model 1 - Leader', 2, 'on');  % leader speed
Simulink.sdi.markSignalForStreaming('VehicleFollowing/Vehicle Model 2 - Following', 3, 'on'); % ego vehicle speed
Simulink.sdi.markSignalForStreaming('VehicleFollowing/Vehicle Model 2 - Following/Multiport Switch', 1, 'on'); % ego vehicle acceleration
Simulink.sdi.markSignalForStreaming('VehicleFollowing/Sum2', 1, 'on'); % relative distance
%% unreal engine
% unreal engine can not be used in mac, comment out the 4 blocks related to
% 3D visulization and uncomment the 2D visulization block 
if ispc
    set_param('VehicleFollowing/Simulation 3D Scene Configuration', 'commented', 'off');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following1', 'commented', 'off');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following2', 'commented', 'off');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following', 'commented', 'off');
    set_param('VehicleFollowing/2D Animation', 'commented', 'on');
else
    set_param('VehicleFollowing/Simulation 3D Scene Configuration','commented','on');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following1','commented','on');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following2','commented','on');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following','commented','on');
    set_param('VehicleFollowing/2D Animation', 'commented', 'off');
end    
%% run the simulation
sim('VehicleFollowing');
%% plot
all_data = Simulink.sdi.Run.getLatest;
% get the data ID from Simulation data inspector to plot, the index should
% be changed according to the corresponding index in Simulation data inspector 
relative_distance_ID = getSignalIDByIndex(all_data, 1);
leader_speed_ID = getSignalIDByIndex(all_data, 2);
ego_speed_ID = getSignalIDByIndex(all_data, 3);
acceleration_ID = getSignalIDByIndex(all_data, 4);
% get the data according to ID
relative_distance = Simulink.sdi.getSignal(relative_distance_ID);
leader_speed = Simulink.sdi.getSignal(leader_speed_ID);
ego_speed = Simulink.sdi.getSignal(ego_speed_ID);
acceleration = Simulink.sdi.getSignal(acceleration_ID);
% calculate min TTC
relative_speed = leader_speed.Values.Data - ego_speed.Values.Data;
idx = relative_speed < 0;
TTC = -relative_distance.Values.Data(idx)./relative_speed(idx);
min_TTC = min(TTC);
disp("min TTC is " + num2str(min_TTC) + 's');
% plot relative distance MathWorks
figure;
plot(relative_distance.Values.Time, 10 + ego_speed.Values.Data * 1.4);
hold on;
ylim([0, 80]);
plot(relative_distance.Values.Time, relative_distance.Values.Data);
hold off;
ylabel(' ');
legend('desired relative distance', 'actual relative distance', 'FontSize', 20);
title('relative distance', 'fontname', 'Times New Roman', 'Color', 'k', 'FontSize', 20);
% plot speed and acceleration
figure;
% speed
subplot(1, 2, 1);
plot(leader_speed.Values.Time, leader_speed.Values.Data);
hold on;
plot(ego_speed.Values.Time, ego_speed.Values.Data);
hold off;
ylabel(' ');
legend('lead vehicle speed', 'ego vehicle speed', 'FontSize', 20);
title('speed', 'fontname', 'Times New Roman', 'Color', 'k', 'FontSize', 20);
% acceleration
subplot(1, 2, 2);
plot(acceleration.Values.Time, acceleration.Values.Data);
hold off;
ylabel(' ');
legend('ego vehicle acceleration', 'FontSize', 20);
ylim([-6, 3]);
title('acceleration', 'fontname', 'Times New Roman', 'Color', 'k', 'FontSize', 20);
end