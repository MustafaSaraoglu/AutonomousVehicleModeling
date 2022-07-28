classdef MCTSTreeSearch
    % Search tree to find best decision
    
    properties
        % Configurations
        maxDepth
        deltaT
        cutOffValue_unsafety
        
        Ts % Sample time [s]
        Th % Time horizon for trajectory genereation [s]
        
        discount % Discount future safety levels
        
        NewManeuverPlanner % Generate decisions
        
        % Scene that is drawn
        currentState_Ego
        currentStates_Other
        scene
        
        % The tree
        tree
        digraph
        visualization
        
        % The decision output
        chosenManeuver
        nextState
    end
    
    methods
        function obj = MCTSTreeSearch(EgoInfo, NewManeuverPlanner)
            %TREESEARCH Construct an instance of this class
            obj.Ts = EgoInfo.Ts;
            obj.Th = EgoInfo.timeHorizon;
            
            obj.NewManeuverPlanner = NewManeuverPlanner;
            
            % Search Tree properties
            obj.maxDepth = 4; % Expand until nth depth if not pruned
            obj.deltaT = 2; % time horizon for each depth: deltaT seconds
            obj.cutOffValue_unsafety = 0.001; % UnSafety values greater than this should be pruned
        end
        
        function [bestDecision_total, nextState, obj] = PlanManeuver(obj, state_Ego0, states_Other0, d_destination)
            % Maneuver planning using POMDP + MCTS
            % Register all current states for drawing the scene later
            obj.currentState_Ego = state_Ego0;
            obj.currentStates_Other = states_Other0;
            
            % Assign the maneuvers
            Maneuvers = obj.NewManeuverPlanner.Maneuvers;
            
            %% Tree Generation
            % Start by creating the root node
            count = 1; % Id of the first node
            rootNodes = [];
            UnSafetyValue=0; % Initial state for decision making shouldn't be a collision state
            
            % Create the root node but since it is the only node, it should be first a
            % leaf node so that we expand it.
            leafNodes = Node([],[],count,obj.currentState_Ego,Maneuvers,UnSafetyValue);
            count = count + 1; % Increase the Id for the next nodes
            
            
            for depth = 1:obj.maxDepth
                
                
                for leafNode = leafNodes
                    newleafNodes = [];
                    % Expand the root node if safe
                    if leafNode.UnsafetyValue < obj.cutOffValue_unsafety
                        
                        for maneuver = Maneuvers
                            % Expand for each maneuever
                            newleafNode = leafNode.expand(count,maneuver,obj.deltaT,states_Other0);
                            count = count + 1; % Increase the Id for the next nodes
                            
                            leafNode.targetNodeID = [leafNode.targetNodeID newleafNode.id];
                            
                            % Calculate unsafety value for each other vehicle
                            % and take the max unsafety as the unsafety value of the state
                            UnSafetyValue = [];
                            
                            for otherVehicle =1:length(states_Other0)
                                
                                % Calculate PDF of Other Vehicles
                                pdf_other = NewManeuver.calculatePDFofOtherVehicles(states_Other0(otherVehicle),obj.deltaT,depth);
                                
                                % UnsafetyValue = Ego vehicle's area under the normal distribution curve of other vehicles
                                if abs(newleafNode.state.d - states_Other0(otherVehicle).d) < 0.1 % Tolerance value for "d"
                                    % If on the same lane
                                    width = (4*2)+4; % 4 variance * 2 meters + 4 meters safety distance
                                    UnSafetyValue_new = abs(pdf_other.cdf(newleafNode.state.s+width)-pdf_other.cdf(newleafNode.state.s-width)); % Size +-2 meters from the center
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
                
                % Iterate other vehicles at the end of each depth
                states_Other0 = NewManeuver.moveOtherVehicles(states_Other0,obj.deltaT);
            end
            
            count = count - 1; % Undo the last increment
            
            if isempty(leafNodes)
                %If all the leaf nodes are pruned, take them back from the
                %root nodes and make leaf again otherwise no decision can be taken
                [rootNodes, leafNodes] = MCTSTreeSearch.getLeafNodesfromRootNodes(rootNodes);
            end
            
            % Build the tree
            obj = obj.buildtheTree(rootNodes,leafNodes,obj.cutOffValue_unsafety);
            
            
            bestDecision_total = obj.chosenManeuver;
            nextState = obj.nextState;
            
        end
        
        function obj = buildtheTree(obj,rootNodes,leafNodes,unsafeBoundaryValue)
            % Make an array of all nodes
            allNodes = [rootNodes leafNodes];
            
            % make it "true" for visualizing tree at every loop
            visualize = false;
            
            % Plot the tree using all the calculated nodes and edges
            % creating a digraph: source node -> edge -> target node
            s = [allNodes.sourceNodeID];
            t = [allNodes.targetNodeID];
            G = digraph(s,t);
            
            % Change the labels of the digraph
            G.Edges.Labels = cellfun(@getName,[allNodes.sourceEdgeName],'UniformOutput',false)';
            G.Nodes.UnsafetyValues = [allNodes.UnsafetyValue]'; % Node labels: Unsafety values
            
            idx = [allNodes.id];
            Safest_idx = idx([allNodes.UnsafetyValue]<0.00001); % Mark safest nodes Green
            
            
            
            
            % Optional: Change the Labels from just UnsafetyValues to Unsafety + Liveness values
            NodeLabels = strcat('RLoU:',num2str(G.Nodes.UnsafetyValues,'%.4f'),'/ L:',num2str([vertcat([allNodes.state]).s]','%.1f'));
            
            
            % Highlight unsafe nodes with red
            Pruned_idx = MCTSTreeSearch.findUnsafeNodes(allNodes,idx, unsafeBoundaryValue);
            
            % Highlight edges that lead to unsafe nodes with red
            unsafeSourceTargetPairs = MCTSTreeSearch.findUnsafeEdges(G,Pruned_idx);
            
            % Find the highest liveness value among the safest states
            SafestNode = MCTSTreeSearch.findTheBestState(leafNodes,Safest_idx);
            
            % Get the safe node's id and track back to the initial state
            SafePath_idx = MCTSTreeSearch.findTheSafestPath(allNodes,SafestNode);
            
            
            % Output the first maneuver of the safe path
            obj.chosenManeuver = allNodes(SafePath_idx(end-1)).sourceEdgeName{1};
            obj.nextState =  allNodes(SafePath_idx(end-1));
            
            
            obj.tree = allNodes;
            obj.digraph = G;
            
            if visualize
                f2 = figure(2);
                f2.WindowState = 'maximized';
                % Plot the tree from top down using 'Layout','layered'
                h=plot(G,'NodeLabel',G.Nodes.UnsafetyValues,'EdgeLabel',G.Edges.Labels,'Layout','layered');
                
                % Highlight Safe and Unsafe Nodes (Safe: Green, Unsafe: Red, InBetween: Blue)
                highlight(h,Safest_idx,'NodeColor','g')
                % Optional: Change the Labels from just UnsafetyValues to Unsafety + Liveness values
                h.labelnode(idx,string(NodeLabels));
                % Highlight unsafe nodes with red
                highlight(h,Pruned_idx,'NodeColor','r')
                % Highlight edges that lead to unsafe nodes with red
                highlight(h,unsafeSourceTargetPairs(:,1),unsafeSourceTargetPairs(:,2),'EdgeColor','r');
                % Highlight all the nodes from initial state to the safest state
                highlight(h,SafePath_idx);
                % Highlight all the edges leading to the safe state
                highlight(h,SafePath_idx(2:end),SafePath_idx(1:end-1))
                
                obj.visualization = h;
                input("Click enter to continue!")
                close(f2)
                
            end
            
        end
        
        
        function value = evaluate(obj, safety, state)
            % Evaluate safety and state
            
            liveness = round(state.s, 1) + round(state.speed, 1)*obj.Th - abs(round(state.d, 1));
            value = Values(safety, liveness);
        end
    end
    
    methods (Static)
        
        function [newRootNodes,leafNodes] = getLeafNodesfromRootNodes(rootNodes)
            leafNodes = [];
            newRootNodes = [];
            for rn = rootNodes
            
                if isempty(rn.targetNodeID)
                    leafNodes = [leafNodes rn];
                else
                    newRootNodes = [newRootNodes rn];
                end
            end
        end
        
        function bestDecision = findTheBestState(leafNodes,AllNodes_Safest_idx)
            %Safest leaf nodes
            SafestLeafNodes = leafNodes(ismember([leafNodes.id],AllNodes_Safest_idx));
            
            if length(SafestLeafNodes)>1 % If there are more than one safest node
                % Compare liveness value for the best decision
                [Live_val, idx] = max([vertcat(SafestLeafNodes.state).s]);
                bestDecision = SafestLeafNodes(idx);
                Safe_val = bestDecision.UnsafetyValue;
            else
                % Just choose the safest ~ minimum unsafe
                [Safe_val,idx_best] = min([leafNodes.UnsafetyValue]);
                Live_val = leafNodes(idx_best).state.s;
                bestDecision = leafNodes(idx_best);
            end
            
            %disp(['Safest state value: ',num2str(Safe_val)]);
            %disp(['Liveness state value: ',num2str(Live_val)]);
        end
        
        
        function Pruned_idx = findUnsafeNodes(allNodes,idx, unsafeBoundaryValue)
            
            Pruned_idx= idx([allNodes.UnsafetyValue]>unsafeBoundaryValue);% Mark unsafe nodes Red
            
            
            if isempty(Pruned_idx)
                return
            end
            
            % Propagate pruned nodes if all children are pruned also mark parent node red
            while true
                
                % Previous version (keeping for assurance purposes because it was working fine for ==3 maneuvers)
                %[~,AllChildrenPruned_idx] = find(histcounts([allNodes(Pruned_idx).sourceNodeID])==4); % 4 must be equal to the number of maneuvers
                
                C = histogram([allNodes(Pruned_idx).sourceNodeID],1:max([allNodes(Pruned_idx).sourceNodeID]),"Visible","off");
                AllChildrenPruned_idx = [allNodes(C.Values==4).id];
                
                
                if all(ismember(AllChildrenPruned_idx,Pruned_idx))
                    break;
                else
                    Pruned_idx = unique([Pruned_idx AllChildrenPruned_idx]);
                end
            end
        end
        
        function unsafeSourceTargetPairs = findUnsafeEdges(G,Pruned_idx)
            % Find the edges that lead to unsafe nodes
            unsafeEdges_idx=ismember(G.Edges.EndNodes(:,2), Pruned_idx);
            unsafeSourceTargetPairs =G.Edges.EndNodes(unsafeEdges_idx,:);
        end
        
        function SafePath_idx = findTheSafestPath(allNodes,SafestNode)
            % From the safest leaf node get the path back to the initial state
            
            SafePath_idx = SafestNode.id;
            
            while true
                idx_safe = SafestNode.sourceNodeID;
                SafestNode = allNodes(idx_safe);
                SafePath_idx = [SafePath_idx SafestNode.id];
                
                if isempty(SafestNode.sourceNodeID)
                    break;
                end
            end
            
        end
    end
end

