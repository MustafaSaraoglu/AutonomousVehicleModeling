%% Test/Tutorial script
%This is a script for building a tree using "dummy" reachability functions 
%for expanding states and calculating next states in some finite time horizon.

% Traffic situation - 1 Ego Vehicle, 3 Other Vehicles
currentState_Ego = State(150,0,0,4); % Initial state for the Ego Vehicle (which decides for the maneuvers)
currentStates_Other = [State(95,0,0,8) State(150,3.7000,0,2) State(165,0,0,9)]; % Initial state for the Other Vehicles


counter(0); % initialize the counter to later count the states in the tree
maxDepth = 4; % Expand until nth depth if not pruned 
deltaT = 2; % time horizon for each depth: deltaT seconds
unsafeBoundaryValue = 0.02;

% Determine available ego actions
Maneuvers = Maneuver.getallActions();


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
        % Expand the root node if safe
        if leafNode.UnsafetyValue < unsafeBoundaryValue
            
            for maneuver = Maneuvers
                % Expand for each maneuever 
                newleafNode = leafNode.expand(maneuver,deltaT);
                
                leafNode.targetNodeID = [leafNode.targetNodeID newleafNode.id];
                
                % Calculate unsafety value for each other vehicle
                % and take the max unsafety as the unsafety value of the state
                UnSafetyValue = [];
                
                for otherVehicle =1:length(currentStates_Other)
                    
                    % Calculate PDF of Other Vehicles
                    pdf_other = calculatePDFofOtherVehicles(currentStates_Other(otherVehicle),deltaT,depth);
                    
                    % UnsafetyValue = Ego vehicle's area under the normal distribution curve of other vehicles
                    if abs(newleafNode.state.d - currentStates_Other(otherVehicle).d) < 0.03 % Tolerance value for "d"
                        % If on the same lane
                        UnSafetyValue_new = abs(pdf_other.cdf(newleafNode.state.s+2)-pdf_other.cdf(newleafNode.state.s-2)); % Size +-2 meters from the center
                        UnSafetyValue = [UnSafetyValue UnSafetyValue_new]; % Add unsafety value for each other vehicle to array
                    else
                        % If not on the same lane
                        UnSafetyValue_new = 0;
                        UnSafetyValue = [UnSafetyValue UnSafetyValue_new];
                    end
                end
                
                % Take the max unsafety
                newleafNode.UnsafetyValue = max(UnSafetyValue);
                
                % Add each newly discovered leaf Node to the leaf nodes array
                newleafNodes = [newleafNodes newleafNode];
                
                
            end
        else
            % Report pruned states because it is over the unsafety value threshold (for debugging purposes)
            disp(strcat(num2str(leafNode.sourceNodeID),'-', leafNode.sourceEdgeName{1}.name,'-','pruned'));
        end
        
        % Make the leafNode a rootNode and add to the array
        rootNodes = [rootNodes leafNode];
        
        [val, idx] = intersect([leafNodes.id], [rootNodes.id]);
        leafNodes(idx)=[]; % Remove the root node from the leafNodes array to avoid duplicates
        
        % Add all "safe" new leaf nodes to the all leafNodes array
        leafNodes = [leafNodes newleafNodes];
        
    end
end

G = GameTree(rootNodes,leafNodes,unsafeBoundaryValue);

% Calculate the other vehicles' motion as a normal distribution centered around x1 = x0 + v*t
function pd = calculatePDFofOtherVehicles(states,deltaT,k)
mean = states.s + states.speed*deltaT*k;
variance = deltaT*k;
pd = makedist('Normal','mu',mean,'sigma',variance);

%range = [mean-(4*variance):0.1:mean+(4*variance)];
%plot(range,pdf(pd,range));
end
