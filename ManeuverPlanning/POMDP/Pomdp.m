%% Test/Tutorial script
%This is a script for building a tree using "dummy" reachability functions 
%for expanding states and calculating next states in some finite time horizon.

%% Traffic situation - 1 Ego Vehicle, 3 Other Vehicles
currentState_Ego = State(150,0,0,2); % Initial state for the Ego Vehicle (which decides for the maneuvers)
currentStates_Other = [State(95,0,0,8) State(150,3.7000,0,2) State(165,0,0,5)]; % Initial state for the Other Vehicles

%% Determine available ego actions
Maneuvers = Maneuver.getallActions();

%% Search Tree Values
maxDepth = 5; % Expand until nth depth if not pruned 
deltaT = 2; % time horizon for each depth: deltaT seconds
cutOffValue_unsafety = 0.02; % UnSafety values greater than this should be pruned

%% Instantiate the tree object
tree =GameTreePlanner(maxDepth,deltaT,cutOffValue_unsafety);

%% Calculate the best decision
tree.calculateBestDecision(currentState_Ego, currentStates_Other,Maneuvers)

