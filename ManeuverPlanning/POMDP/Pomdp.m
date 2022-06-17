%% Test/Tutorial script
%This is a script for building a tree using "dummy" reachability functions 
%for expanding states and calculating next states in some finite time horizon.

% Clear all variables and load a saved snapshot from a traffic situation/scene
clear all;
load('decisionInputData.mat');
% Just for renaming an object from the saved data Planner <- obj
Planner = obj;
clear obj;

counter(0); % initialize the counter to later count the states in the tree
maxDepth = 4; % Expand until nth depth if not pruned 
deltaT = 2; % time horizon for each depth: deltaT seconds
unsafeBoundaryValue = 0.02;

% Manipulate the original variables to create meaning situation because the
% ego vehicle is too fast and avoids collision by default
currentState_Ego.speed = 4; % Speed reduced from 8 to 4

% Determine available ego actions
Maneuvers = {'FD','EB','LC'}; % TODO Later: create a "maneuver class" to also contain the reachability formulations

%% Tree Generation
% Start by creating the root node
rootNodes = [];
UnSafetyValue=0; % Initial state for decision making shouldn't be a collision state

% Create the root node but since it is the only node, it should be first a
% leaf node so that we expand it.
leafNode = Node([],[],currentState_Ego,Maneuvers,UnSafetyValue);
leafNodes = leafNode;


for depth = 1:maxDepth
    
    
    
    for leafNode = leafNodes
        
        newleafNodes = [];
        %expand the root node for each maneuever if safe
        if leafNode.UnsafetyValue < unsafeBoundaryValue
            for maneuver = Maneuvers
                
                
                
                newleafNode = leafNode.expand(maneuver,deltaT);
                
                leafNode.targetNodeID = [leafNode.targetNodeID newleafNode.id];
                UnSafetyValue = [];
                %Calculate unsafety value
                for otherVehicle =1:length(currentStates_Other)
                    
                    % Calculate PDF of Other Vehicles
                    pdf_other = calculatePDFofOtherVehicles(currentStates_Other(otherVehicle),deltaT,depth);
                    % UnsafetyValue = Area under curve
                    if abs(newleafNode.state.d - currentStates_Other(otherVehicle).d) < 0.03 % Tolerance value for "d"
                        UnSafetyValue_new = abs(pdf_other.cdf(newleafNode.state.s+2)-pdf_other.cdf(newleafNode.state.s-2)); % Size +-2 meters from the center
                        UnSafetyValue = [UnSafetyValue UnSafetyValue_new];
                    else
                        UnSafetyValue_new = 0;
                        UnSafetyValue = [UnSafetyValue UnSafetyValue_new];
                    end
                    
                end
                
                
                newleafNode.UnsafetyValue = max(UnSafetyValue);
                newleafNodes = [newleafNodes newleafNode];
                
                
            end
        else
            disp(strcat(num2str(leafNode.sourceNodeID),'-', leafNode.sourceEdgeName{1},'-','pruned'));
        end
        
        % Make leafNode a rootNode
        rootNodes = [rootNodes leafNode];
        
        [val, idx] = intersect([leafNodes.id], [rootNodes.id]);
        leafNodes(idx)=[]; % Remove the root node from the leaves
        
        leafNodes = [leafNodes newleafNodes];
        
    end
end

allNodes = [rootNodes leafNodes];

GameTree.plot(allNodes,leafNodes,unsafeBoundaryValue);




% Determine Rewards for children states
% Make children states parent states and expand again


function pd = calculatePDFofOtherVehicles(states,deltaT,k)
mean = states.s + states.speed*deltaT*k;
variance = deltaT*k;
range = [mean-(4*variance):0.1:mean+(4*variance)];
pd = makedist('Normal','mu',mean,'sigma',variance);

%plot(range,pdf(pd,range));
end
