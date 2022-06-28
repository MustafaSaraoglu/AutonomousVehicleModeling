classdef GameTreePlanner
    %GAMETREEPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Configurations
        maxDepth
        deltaT
        cutOffValue_unsafety
        
        % Tree Object
        tree
        
        % Scene that is drawn
        currentState_Ego
        currentStates_Other
        scene
        
    end
    
    methods
        function obj = GameTreePlanner(maxDepth,deltaT,cutOffValue_unsafety)
            %GAMETREEPLANNER Construct an instance of this class
            %   Detailed explanation goes here
            obj.maxDepth = maxDepth;
            obj.deltaT = deltaT;
            obj.cutOffValue_unsafety = cutOffValue_unsafety;
        end
        function obj = calculateBestDecision(obj,currentState_Ego,currentStates_Other,Maneuvers)
            % Register all current states for drawing the scene later
            obj.currentState_Ego = currentState_Ego;
            obj.currentStates_Other = currentStates_Other;
            
            %% Tree Generation
            % Start by creating the root node
            count = 1; % Id of the first node
            rootNodes = [];
            UnSafetyValue=0; % Initial state for decision making shouldn't be a collision state
            
            % Create the root node but since it is the only node, it should be first a
            % leaf node so that we expand it.
            leafNodes = Node([],[],count,currentState_Ego,Maneuvers,UnSafetyValue);
            count = count + 1; % Increase the Id for the next nodes
            
            for depth = 1:obj.maxDepth
                
                
                for leafNode = leafNodes
                    newleafNodes = [];
                    % Expand the root node if safe
                    if leafNode.UnsafetyValue < obj.cutOffValue_unsafety
                        
                        for maneuver = Maneuvers
                            % Expand for each maneuever
                            newleafNode = leafNode.expand(count,maneuver,obj.deltaT);
                            count = count + 1; % Increase the Id for the next nodes
                            
                            leafNode.targetNodeID = [leafNode.targetNodeID newleafNode.id];
                            
                            % Calculate unsafety value for each other vehicle
                            % and take the max unsafety as the unsafety value of the state
                            UnSafetyValue = [];
                            
                            for otherVehicle =1:length(currentStates_Other)
                                
                                % Calculate PDF of Other Vehicles
                                pdf_other = Maneuver.calculatePDFofOtherVehicles(currentStates_Other(otherVehicle),obj.deltaT,depth);
                                
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
            obj.tree = GameTree(rootNodes,leafNodes,obj.cutOffValue_unsafety);
        end
        
        function drawScene(obj)
            
            currentState_Ego = obj.currentState_Ego;
            currentStates_Other = obj.currentStates_Other;
            
            %% Clear previous Vehicle
            figure(2);
            hold on;
            %% Vehicle Ego - Plot
            x1 = currentState_Ego.s;
            y1 = currentState_Ego.d;
            yaw1 = currentState_Ego.orientation;
            
            centerP = [x1;y1];
            
            V1_HalfLength = 2; % Length = 4m
            V1_HalfWidth = 0.8; % Width = 0.8 m
            
            % Creating a rectangle
            p1 = [V1_HalfLength; V1_HalfWidth];
            p2 = [V1_HalfLength; -V1_HalfWidth];
            p3 = [-V1_HalfLength; -V1_HalfWidth];
            p4 = [-V1_HalfLength; V1_HalfWidth];
            
            % Rotation Matrix
            Rmatrix = [cos(yaw1) -sin(yaw1); sin(yaw1) cos(yaw1)];
            
            % Rotated Points
            p1r = centerP + Rmatrix*p1;
            p2r = centerP + Rmatrix*p2;
            p3r = centerP + Rmatrix*p3;
            p4r = centerP + Rmatrix*p4;
            
            % Connecting points
            Hitbox_V1 = [p1r p2r p3r p4r p1r];
            
            cornersV1_x = transpose(Hitbox_V1(1,:));
            cornersV1_y = transpose(Hitbox_V1(2,:));
            
            plot(cornersV1_x,cornersV1_y,'b'); %Vehicle 1 rectangle
            plot(x1,y1,'*'); %Vehicle 1 center
            
            %% Plot Other Vehicles
            for k = 1:length(currentStates_Other)
                %% Vehicle 2 - Pose
                x2 = obj.currentStates_Other(k).s;
                y2 = obj.currentStates_Other(k).d;
                yaw2 = obj.currentStates_Other(k).orientation;
                
                centerP = [x2;y2];
                
                V2_HalfLength = 2; % Length = 4m
                V2_HalfWidth = 0.8; % Width = 0.8 m
                
                % Creating a rectangle
                p1 = [V2_HalfLength; V2_HalfWidth];
                p2 = [V2_HalfLength; -V2_HalfWidth];
                p3 = [-V2_HalfLength; -V2_HalfWidth];
                p4 = [-V2_HalfLength; V2_HalfWidth];
                
                % Rotation Matrix
                
                Rmatrix = [cos(yaw2) -sin(yaw2); sin(yaw2) cos(yaw2)];
                
                % Rotated Points
                p1r = centerP + Rmatrix*p1;
                p2r = centerP + Rmatrix*p2;
                p3r = centerP + Rmatrix*p3;
                p4r = centerP + Rmatrix*p4;
                
                % Connecting points
                Hitbox_V2 = [p1r p2r p3r p4r p1r];
                
                cornersV2_x = transpose(Hitbox_V2(1,:));
                cornersV2_y = transpose(Hitbox_V2(2,:));
                
                plot(cornersV2_x,cornersV2_y,'r'); %Vehicle 2 rectangle
                plot(x2,y2,'o'); %Vehicle 2 center
                
            end
            
            
            %% Plot the road
            upperLine = plot([0 1000], [5.55 5.55], 'Color', 'blue');
            midLine = plot([0 1000], [1.85 1.85],'--', 'Color', 'blue');
            lowerLine = plot([0 1000], [-1.85 -1.85], 'Color', 'blue');

            % Adjust Axis
            axis([x1-40 x1+60 y1-30 y1+30]); % Camera following V1 as ego vehicle
        end
        
    end
    
end

