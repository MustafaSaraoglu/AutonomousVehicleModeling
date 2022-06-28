%% Test/Tutorial script
clear all
close all
%This is a script for building a tree using "dummy" reachability functions 
%for expanding states and calculating next states in some finite time horizon.

%% Traffic situation - 1 Ego Vehicle, 3 Other Vehicles
currentState_Ego = StateV(150,0,0,6); % Initial state for the Ego Vehicle (which decides for the maneuvers)
currentStates_Other = [StateV(125,0,0,8) StateV(150,3.7000,0,2) StateV(165,0,0,8)]; % Initial state for the Other Vehicles

%% Determine available ego actions
Maneuvers = Maneuver.getallActions();

%% Search Tree Values
maxDepth = 5; % Expand until nth depth if not pruned 
deltaT = 2; % time horizon for each depth: deltaT seconds
cutOffValue_unsafety = 0.02; % UnSafety values greater than this should be pruned

%% Instantiate the tree object
Planner =GameTreePlanner(maxDepth,deltaT,cutOffValue_unsafety);

%% Calculate the best decision
Planner = Planner.calculateBestDecision(currentState_Ego, currentStates_Other,Maneuvers);

disp(['Best Maneuver: ',Planner.tree.chosenManeuver])


Planner.drawScene();

