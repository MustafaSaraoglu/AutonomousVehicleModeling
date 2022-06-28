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
cutOffValue_unsafety = 0.02;

%% Tree Generation
% Start by creating the root node
count = 1; % Id of the first node
rootNodes = [];
UnSafetyValue=0; % Initial state for decision making shouldn't be a collision state

% Create the root node but since it is the only node, it should be first a
% leaf node so that we expand it.
leafNodes = Node([],[],count,currentState_Ego,Maneuvers,UnSafetyValue);
count = count + 1; % Increase the Id for the next nodes

for depth = 1:maxDepth
    
   
    for leafNode = leafNodes
        newleafNodes = [];
        % Expand the root node if safe
        if leafNode.UnsafetyValue < cutOffValue_unsafety
            
            for maneuver = Maneuvers
                % Expand for each maneuever 
                newleafNode = leafNode.expand(count,maneuver,deltaT);
                count = count + 1; % Increase the Id for the next nodes
                
                leafNode.targetNodeID = [leafNode.targetNodeID newleafNode.id];
                
                % Calculate unsafety value for each other vehicle
                % and take the max unsafety as the unsafety value of the state
                UnSafetyValue = [];
                
                for otherVehicle =1:length(currentStates_Other)
                    
                    % Calculate PDF of Other Vehicles
                    pdf_other = Maneuver.calculatePDFofOtherVehicles(currentStates_Other(otherVehicle),deltaT,depth);
                    
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
        
        leafNodes(1)=[]; % Remove the root node from the leafNodes array to avoid duplicates
        
        % Add all "safe" new leaf nodes to the all leafNodes array
        leafNodes = [leafNodes newleafNodes];
        
    end
end

count = count - 1; % Undo the last increment

% Build the tree
G = GameTree(rootNodes,leafNodes,cutOffValue_unsafety);

