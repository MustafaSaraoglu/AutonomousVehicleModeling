%% Planner: Manual Design
%========================
%% Scenario 1
% RightLane_FreeDrive -> RightLane_VehicleFollowing -> ToLeftLane_FreeDrive 
% -> LeftLane_FreeDrive -> LeftLane_VehicleFollowing 
% -> ToRightLane_FreeDrive -> RightLane_FreeDrive

prepare_simulation('s_0', [0, 60, 40], 'd_0', [0, 0, 3.7], ...
    'v_0', [20, 10, 13], 'v_ref', [20, 10, 13]);
run_simulation();

%% Scenario 2
% RightLane_FreeDrive -> RightLane_VehicleFollowing -> RightLane_EmergencyBrake 
% -> RightLane_VehicleFollowing -> ToLeftLane_FreeDrive 
% -> LeftLane_FreeDrive -> LeftLane_VehicleFollowing 
% -> LeftLane_EmergencyBrake -> LeftLane_VehicleFollowing -> ...

prepare_simulation('s_0', [0, 40, 20], 'd_0', [0, 0, 3.7], ...
    'v_0', [23.5, 10, 30], 'v_ref', [30, 10, 4.2]);
run_simulation();

%% Scenario 3
% RightLane_FreeDrive -> RightLane_EmergencyBrake
% -> RightLane_VehicleFollowing -> RightLane_FreeDrive

prepare_simulation('s_0', [0, 9, 0], 'd_0', [0, 0, 3.7], ...
    'v_0', [15, 10, 30], 'v_ref', [15, 30, 0]);
run_simulation();

%% Scenario 4
% RightLane_FreeDrive -> RightLane_VehicleFollowing -> ToLeftLane_FreeDrive 
% -> LeftLane_FreeDrive -> LeftLane_EmergencyBrake -> LeftLane_VehicleFollowing
% -> LeftLane_FreeDrive -> ToRightLane_FreeDrive -> ...

prepare_simulation('s_0', [0, 30, 55], 'd_0', [0, 0, 3.7], ...
    'v_0', [16.1, 10, 0.3], 'v_ref', [16.1, 10, 30]);
run_simulation();
