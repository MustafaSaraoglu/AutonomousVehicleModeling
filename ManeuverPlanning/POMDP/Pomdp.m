clear all;
load('decisionInputData.mat');

Planner = obj;
clear obj;


% Determine available ego actions
counter(0);
Maneuvers = {'FD','EB','LC'};


rootNodes = [];
leafNodes = [];
newleafNodes = [];
%% POMDP Generation
% Create root node
leafNode = Node(0,0,currentState_Ego,Maneuvers);

for maneuver = Maneuvers
    %expand the root node for each maneuever
    newleafNode = leafNode.expand(maneuver);
    newleafNodes = [newleafNodes newleafNode];

end

% Make leafNode a rootNode
rootNodes = [rootNodes leafNode];
leafNodes = [leafNodes newleafNodes];


% Determine Rewards for children states
% Make children states parent states and expand again
