clear all;
load('decisionInputData.mat');

Planner = obj;
clear obj;


% Determine available ego actions
counter(0);
Maneuvers = {'FD','EB','LC'};

UnSafetyValue=0;

rootNodes = [];
%% POMDP Generation
% Create root node
leafNode = Node([],0,currentState_Ego,Maneuvers,UnSafetyValue);

leafNodes = [leafNode];

deltaT = 2; % horizon 2 seconds
for k = 1:4
    
    
    
    for leafNode = leafNodes
         
    newleafNodes = [];
    
    for maneuver = Maneuvers
        %expand the root node for each maneuever
        newleafNode = leafNode.expand(maneuver,deltaT);
        leafNode.targetNodeID = [leafNode.targetNodeID newleafNode.id];
        UnSafetyValue = [];
        %Calculate unsafety value
        for otherVehicle =1:length(currentStates_Other)
            
            % Calculate PDF of Other Vehicles
            pdf_other = calculatePDFofOtherVehicles(currentStates_Other(otherVehicle),deltaT,k);
            % UnsafetyValue = Area under curve
            if abs(newleafNode.state.d - currentStates_Other(otherVehicle).d) < 0.02 % Tolerance value for "d"
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
    
    % Make leafNode a rootNode
    rootNodes = [rootNodes leafNode];
    
    [val, idx] = intersect([leafNodes.id], [rootNodes.id]);
    leafNodes(idx)=[]; % Remove the root node from the leaves
    
    leafNodes = [leafNodes newleafNodes];
    
    end
end

allNodes = [rootNodes leafNodes];

Node.plotTree(allNodes);




% Determine Rewards for children states
% Make children states parent states and expand again


function pd = calculatePDFofOtherVehicles(states,deltaT,k)
    mean = states.s + states.speed*deltaT*k;
    variance = deltaT*k;
    range = [mean-(4*variance):0.1:mean+(4*variance)];
    pd = makedist('Normal','mu',mean,'sigma',variance);
    
    %plot(range,pdf(pd,range));
end
