%% A function script for programmatically simulating and plotting the results
function simulate_and_plot(options)
%% default values for the optional arguments
arguments
    options.position                       (1, :) double = [40, 20];  % e.g. vehicle 1 starts at 40 m, vehicle 2 at 20 m 
    options.speed                          (1, :) double = [20, 25];  % e.g. vehicle 1 starts at 20 m/s, vehicle 2 at 25 m/s 
    options.leader_config                  (1, :) double = [10, 0];   % vehicle 1's reference speed is 10 m/s, it has a extra 0 amplitude sine-wave acceleration by default 
    options.controller                     (1, 1) string = 'MathWorksMPC'; % default controller is 'MathWorks MPC', the other options are:
                                                                           % Custom MPC: 'MPC_quadprog', 'MPC_quadprog_wKalman', 'MPC_fmincon', 'MPC_fmincon_wKalman';
                                                                           % PID: 'PID'   
    options.controller_behavior_MathWorks  (1, 1) double = 0.5;     % default controller behavior for MathWorks MPC is 0.5, the range is [0, 1]
    options.prediction_horizon             (1, 1) double = 10;      % default prediction horizon is 10
    options.time_headway                   (1, 1) double = 1.4;     % default time headway is 1.4 in seconds
    options.acceleration_boundary          (1, :) double = [2, -3]; % default maximum and minimum longitudinal acceleration are 2 m/s^2 and -3 m/s^2
    options.default_spacing                (1, :) double = 10       % default safety distance between leader and ego vehicle is 10 m
    options.simTime                        (1, :) double = 50       % default simulation time in seconds
end

%% Load the Simulink model
close all;
load_system('VehicleFollowing');
open_system('VehicleFollowing');

%% Set Initial Parameters for the Vehicle Models
% The leading Vehicle - Vehicle Model 1 - Leader
leader.position = num2str(options.position(1));
leader.speed = num2str(options.speed(1));
leader.speed_reference = num2str(options.leader_config(1));
leader.speed_disturbance = num2str(options.leader_config(2)); % speed disturbance is sine wave and its amplitude is set to 0 by default
% set the initial speed and position for Vehicle Model 1
set_param('VehicleFollowing/Vehicle Model 1 - Leader','Speed', leader.speed); % set leader speed
set_param('VehicleFollowing/Vehicle Model 1 - Leader','Pos', leader.position); % set leader position
set_param('VehicleFollowing/Vehicle Model 1 - Leader','Disturbance', leader.speed_disturbance); % set leader speed disturbance
set_param('VehicleFollowing/Speed Reference','Value', leader.speed_reference) % set leader speed reference
% Ego Vehicle - Vehicle Model 2 - Following
ego.position = num2str(options.position(2));
ego.speed = num2str(options.speed(2));
ego.controller = options.controller;
% set the initial speed and position for Vehicle Model 2
set_param('VehicleFollowing/Vehicle Model 2 - Following', 'Speed', ego.speed);
set_param('VehicleFollowing/Vehicle Model 2 - Following', 'Pos', ego.position);
% set the vehicle following controller
blockname = 'VehicleFollowing/Vehicle Model 2 - Following/Variant Vehicle Following Mode';
mask_vehicle_2 = Simulink.Mask.get(blockname);
set_param(blockname,'LabelModeActiveChoice', ego.controller);
controller_behavior_MathWorksMPC = num2str(options.controller_behavior_MathWorks);
% set the vehicle following controller
controller_set = mask_vehicle_2.Parameters(1); % MaskParameter: Controller
controller_set.set('Value', ego.controller);
% set the controller behavior for MathWorks MPC
control_behavior = mask_vehicle_2.Parameters(3); % MaskParameter: Controller Behaviour MathWorks MPC
control_behavior.set('Value', controller_behavior_MathWorksMPC);
% set the prediction horizon
predict_horizon = mask_vehicle_2.Parameters(4); % MaskParamter: Prediction Horizon
predict_horizon.set('Value', num2str(options.prediction_horizon));
% set the time headway
t_headway = mask_vehicle_2.Parameters(5); % MaskParameter: Time Headway
t_headway.set('Value', num2str(options.time_headway));
% set the maximum and minimum acceleration
max_acc = mask_vehicle_2.Parameters(6); % MaskParameter: Maximum Longitudinal Acceleration (m/s^2)
max_acc.set('Value', num2str(options.acceleration_boundary(1))); 
min_acc = mask_vehicle_2.Parameters(7); % MaskParameter: Minimum Longitudinal Acceleration (m/s^2)
min_acc.set('Value', num2str(options.acceleration_boundary(2)));
% set the fixed safety distance
fixed_safety_distance = mask_vehicle_2.Parameters(8); % MaskParameter: Default Spacing (m)
fixed_safety_distance.set('Value', num2str(options.default_spacing));

%% Data logging from Simulink signals
Simulink.sdi.markSignalForStreaming('VehicleFollowing/Vehicle Model 1 - Leader', 2, 'on');  % leader speed
Simulink.sdi.markSignalForStreaming('VehicleFollowing/Vehicle Model 2 - Following', 3, 'on'); % ego vehicle speed
Simulink.sdi.markSignalForStreaming('VehicleFollowing/Vehicle Model 2 - Following/Multiport Switch', 1, 'on'); % ego vehicle acceleration
Simulink.sdi.markSignalForStreaming('VehicleFollowing/Sum2', 1, 'on'); % relative distance

%% Visualization using Unreal Engine
% This part of the script can be changed according to preference

if ispc % If Windows PC -> Unreal Engine blocks in Simulink only work on Windows PC if you have the necessary toolboxes
    set_param('VehicleFollowing/Simulation 3D Scene Configuration', 'commented', 'off');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following1', 'commented', 'off');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following2', 'commented', 'off');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following', 'commented', 'off');
    set_param('VehicleFollowing/2D Animation', 'commented', 'on'); % Comment out the basic 2D visualization
else % If Not -> Comment Out Unreal Engine Blocks and enable 2D visualization
    set_param('VehicleFollowing/Simulation 3D Scene Configuration','commented','on');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following1','commented','on');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following2','commented','on');
    set_param('VehicleFollowing/Simulation 3D Vehicle with Ground Following','commented','on');
    set_param('VehicleFollowing/2D Animation', 'commented', 'off');
end

%% Run the Simulation for "options.simTime" seconds
sim('VehicleFollowing','StopTime',num2str(options.simTime));

%% Get logged signals from the Simulink Model
all_data = Simulink.sdi.Run.getLatest;

% Get the data ID from Simulation data inspector to plot, the index should
% be changed according to the corresponding index in Simulation data inspector 
relative_distance_ID = getSignalIDByIndex(all_data, 1);
leader_speed_ID = getSignalIDByIndex(all_data, 2);
ego_speed_ID = getSignalIDByIndex(all_data, 3);
driving_mode_ID = getSignalIDByIndex(all_data, 4);
acceleration_ID = getSignalIDByIndex(all_data, 5);

% Get the data according to ID
relative_distance = Simulink.sdi.getSignal(relative_distance_ID);
leader_speed = Simulink.sdi.getSignal(leader_speed_ID);
ego_speed = Simulink.sdi.getSignal(ego_speed_ID);
driving_mode = Simulink.sdi.getSignal(driving_mode_ID);
acceleration = Simulink.sdi.getSignal(acceleration_ID);

%% Plot the Results and Calculate metrics
% Plot the Relative Distance
figure;
subplot(1, 2, 1);
plot(relative_distance.Values.Time, options.default_spacing + ego_speed.Values.Data * options.time_headway);
grid on;
hold on;
ylim([0, 80]);
plot(relative_distance.Values.Time, relative_distance.Values.Data);
hold off;
ylabel('m');
xlabel('s');
legend('Desired relative distance', 'Actual relative distance', 'FontSize', 20);
title('Relative distance', 'Color', 'k', 'FontSize', 20);

% Plot Speed/Time
subplot(1, 2, 2);
plot(leader_speed.Values.Time, leader_speed.Values.Data);
grid on;
hold on;
plot(ego_speed.Values.Time, ego_speed.Values.Data);
hold off;
ylabel('m/s');
xlabel('s');
legend('Lead vehicle speed', 'Ego vehicle speed', 'FontSize', 20);
title('Speed', 'Color', 'k', 'FontSize', 20);

% Plot Acceleration/Time
figure;
subplot(1, 2, 1);
plot(acceleration.Values.Time, acceleration.Values.Data);
grid on;
ylabel('m/s^2');
xlabel('s');
legend('ego vehicle acceleration', 'FontSize', 20);
ylim([-6, 3]);
title('Acceleration', 'Color', 'k', 'FontSize', 20);

% Plot Driving Mode/Time
subplot(1, 2, 2);
plot(driving_mode.Values.Time, driving_mode.Values.Data);
grid on;
ylabel('driving mode');
xlabel('s');
legend(['Value -> Driving Mode', newline, '1 -> Free Drive', newline, '2 -> Vehicle Following', newline, '3 -> Emergency Brake'], 'FontSize', 20);
ylim([0, 4]);
title('Driving Mode', 'Color', 'k', 'FontSize', 20);

%% Calculate min TTC metric
relative_speed = leader_speed.Values.Data - ego_speed.Values.Data;
idx = relative_speed < 0;
TTC = -relative_distance.Values.Data(idx)./relative_speed(idx);
min_TTC = min(TTC);
disp("Min TTC value is " + num2str(min_TTC) + 's');
end