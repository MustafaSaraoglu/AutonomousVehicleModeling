%% ___________________________________________Scenario 1___________________________________________
%% Planner: Rule-based Design
% RightLane_FreeDrive -> RightLane_VehicleFollowing -> ToLeftLane_FreeDrive 
% -> LeftLane_FreeDrive -> LeftLane_VehicleFollowing 
% -> ToRightLane_FreeDrive -> RightLane_FreeDrive

prepare_simulation('s_0', [0, 60, 40], 'd_0', [0, 0, 3.7], ...
    'v_0', [20, 10, 13], 'v_ref', [20, 10, 13], 'planner', 'RuleBased', 'sigmaS', 0, 'sigmaV', 0);
run_simulation();

% Expected Output: 24.01.2022
% @t=0s: Initial state is: 'RightLane_FreeDrive'.
% @t=2.000000s: Switched to State: 'RightLane_VehicleFollowing'.
% @t=15.175000s: Switched to State: 'ToLeftLane_FreeDrive'.
% @t=15.175000s: Start trajectory to left lane, duration=4.000000s.
% @t=18.697500s: Switched to State: 'LeftLane_FreeDrive'.
% @t=20.252500s: Switched to State: 'LeftLane_VehicleFollowing'.
% @t=32.828750s: Switched to State: 'ToRightLane_FreeDrive'.
% @t=32.828750s: Start trajectory to right lane, duration=4.000000s.
% @t=36.367500s: Switched to State: 'RightLane_FreeDrive'.

%% Planner: Minimax Design
prepare_simulation('s_0', [0, 60, 40], 'd_0', [0, 0, 3.7], ...
    'v_0', [20, 10, 13], 'v_ref', [20, 10, 13], 'planner', 'Minimax-Dev'); % With uncertainty
run_simulation();

%% ___________________________________________Scenario 2___________________________________________
%% Planner: Rule-based Design
% RightLane_FreeDrive -> RightLane_VehicleFollowing -> RightLane_EmergencyBrake 
% -> RightLane_VehicleFollowing -> ToLeftLane_FreeDrive 
% -> LeftLane_FreeDrive -> LeftLane_VehicleFollowing 
% -> LeftLane_EmergencyBrake -> LeftLane_VehicleFollowing -> ...

prepare_simulation('s_0', [0, 40, 20], 'd_0', [0, 0, 3.7], ...
    'v_0', [23.5, 10, 30], 'v_ref', [30, 10, 4.2], 'planner', 'Rulebased', 'sigmaS', 0, 'sigmaV', 0);
run_simulation();

% Expected Output: 24.01.2022
% @t=0s: Initial state is: 'RightLane_FreeDrive'.
% @t=0.000000s: Switched to State: 'RightLane_VehicleFollowing'.
% @t=3.837500s: Switched to State: 'RightLane_EmergencyBrake'.
% @t=5.723750s: Switched to State: 'RightLane_VehicleFollowing'.
% @t=9.363750s: Switched to State: 'ToLeftLane_FreeDrive'.
% @t=9.363750s: Start trajectory to left lane, duration=4.000000s.
% @t=12.887500s: Switched to State: 'LeftLane_FreeDrive'.
% @t=12.888750s: Switched to State: 'LeftLane_VehicleFollowing'.
% @t=15.792500s: Switched to State: 'LeftLane_EmergencyBrake'.
% @t=18.598750s: Switched to State: 'LeftLane_VehicleFollowing'.
% @t=21.242500s: Switched to State: 'ToRightLane_FreeDrive'.
% @t=21.242500s: Start trajectory to right lane, duration=4.000000s.
% @t=24.760000s: Switched to State: 'RightLane_FreeDrive'.
% @t=27.962500s: Switched to State: 'RightLane_VehicleFollowing'.
% @t=29.622500s: Switched to State: 'ToLeftLane_FreeDrive'.
% @t=29.622500s: Start trajectory to left lane, duration=4.000000s.
% @t=33.145000s: Switched to State: 'LeftLane_FreeDrive'.
% @t=36.627500s: Switched to State: 'ToRightLane_FreeDrive'.
% @t=36.627500s: Start trajectory to right lane, duration=4.000000s.
% @t=40.183750s: Switched to State: 'RightLane_FreeDrive'.

%% Planner: Minimax Design
prepare_simulation('s_0', [0, 40, 20], 'd_0', [0, 0, 3.7], ...
    'v_0', [23.5, 10, 30], 'v_ref', [30, 10, 4.2], 'planner', 'Minimax-Dev'); % With uncertainty
run_simulation();

%% ___________________________________________Scenario 3___________________________________________
%% Planner: Rule-based Design
% RightLane_FreeDrive -> RightLane_EmergencyBrake
% -> RightLane_VehicleFollowing -> RightLane_FreeDrive

prepare_simulation('s_0', [0, 9, 0], 'd_0', [0, 0, 3.7], ...
    'v_0', [15, 10, 30], 'v_ref', [15, 30, 0], 'planner', 'Rulebased', 'sigmaS', 0, 'sigmaV', 0);
run_simulation();

% Expected Output: 24.01.2022
% @t=0s: Initial state is: 'RightLane_FreeDrive'.
% @t=0.000000s: Switched to State: 'RightLane_EmergencyBrake'.
% @t=2.207500s: Switched to State: 'RightLane_VehicleFollowing'.
% @t=5.557500s: Switched to State: 'RightLane_FreeDrive'.

%% Planner: Minimax Design
prepare_simulation('s_0', [0, 9, 0], 'd_0', [0, 0, 3.7], ...
    'v_0', [15, 10, 30], 'v_ref', [15, 30, 0], 'planner', 'Minimax-Dev'); % With uncertainty
run_simulation();

%% ___________________________________________Scenario 4___________________________________________
%% Planner: Rule-based Design
% RightLane_FreeDrive -> RightLane_VehicleFollowing -> ToLeftLane_FreeDrive 
% -> LeftLane_FreeDrive -> LeftLane_EmergencyBrake -> LeftLane_VehicleFollowing
% -> LeftLane_FreeDrive -> ToRightLane_FreeDrive -> ...

prepare_simulation('s_0', [0, 30, 55], 'd_0', [0, 0, 3.7], ...
    'v_0', [16.1, 10, 0.3], 'v_ref', [16.1, 10, 30], 'planner', 'Rulebased', 'sigmaS', 0, 'sigmaV', 0);
run_simulation();

% Expected Output: 24.01.2022
% @t=0s: Initial state is: 'RightLane_FreeDrive'.
% @t=0.000000s: Switched to State: 'RightLane_VehicleFollowing'.
% @t=0.302500s: Switched to State: 'ToLeftLane_FreeDrive'.
% @t=0.302500s: Start trajectory to left lane, duration=4.000000s.
% @t=3.867500s: Switched to State: 'LeftLane_FreeDrive'.
% @t=3.868750s: Switched to State: 'LeftLane_EmergencyBrake'.
% @t=6.683750s: Switched to State: 'LeftLane_VehicleFollowing'.
% @t=9.685000s: Switched to State: 'LeftLane_FreeDrive'.
% @t=24.853750s: Switched to State: 'ToRightLane_FreeDrive'.
% @t=24.853750s: Start trajectory to right lane, duration=4.000000s.
% @t=28.380000s: Switched to State: 'RightLane_FreeDrive'.

%% Planner: Minimax Design
prepare_simulation('s_0', [0, 30, 55], 'd_0', [0, 0, 3.7], ...
    'v_0', [16.1, 10, 0.3], 'v_ref', [16.1, 10, 30], 'planner', 'Minimax-Dev'); % With uncertainty
run_simulation();