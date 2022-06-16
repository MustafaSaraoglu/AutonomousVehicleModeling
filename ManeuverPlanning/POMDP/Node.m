classdef Node
    %NODE Every node of a tree contains the state/action information, source/target properties
    
    properties
        sourceNodeID    % Parent Node id
        sourceEdgeName  % Which action from parent node let to this state
        id              % id of this node
        state           % state information of this node
        actions         % available actions (edges out) from this state
        targetNodeID    % next states
        UnsafetyValue   % Unsafety probability value 0<p<1
    end
    
    methods
        function obj = Node(sourceNodeID,sourceEdgeName,state,actions,UnsafetyValue)
            %NODE Construct an instance of this class
            obj.sourceNodeID = sourceNodeID;
            obj.sourceEdgeName = sourceEdgeName;
            obj.id = counter(); % assign id and increase the counter for the next possible nodes
            obj.state = state;
            obj.actions = actions;
            obj.UnsafetyValue = UnsafetyValue;
            obj.targetNodeID = []; % To be assigned if expanded
        end
        
        function childNode = expand(obj,maneuver,deltaT)
            % Expand the node based on available actions (edges out)
            %   Next states' values shall be calculated via reachability
            %   based on the finite horizon
            
            % TODO later: Remove hard-coded maneuvers
            if strcmp(maneuver, 'FD')
                speed_new = obj.state.speed +1;
                s_new = obj.state.s + speed_new*deltaT;
                d_new = obj.state.d;
                orientation_new = obj.state.orientation;
                
            elseif strcmp(maneuver, 'EB')
                speed_new = obj.state.speed -3;
                s_new = obj.state.s + speed_new*deltaT;
                d_new = obj.state.d;
                orientation_new = obj.state.orientation;
                
            elseif strcmp(maneuver, 'LC')
                speed_new = obj.state.speed;
                s_new = obj.state.s + speed_new*deltaT;
                if abs(obj.state.d)<0.05 % tolerance for d value
                    d_new = 3.7;
                elseif abs(obj.state.d)-3.7<0.05
                    d_new = 0;                    
                end
                orientation_new = obj.state.orientation;
            end
            
            
            newState = obj.state; % Copy the object to create a state fast
            % Assign newly calculated values for the time horizon
            newState.s = s_new; 
            newState.d = d_new;
            newState.orientation = orientation_new;
            newState.speed = speed_new;
            newActions= {'FD','EB','LC'};
            
            % Return the newly created child node
            childNode = Node(obj.id, maneuver, newState, newActions,[]);
        end
        

    end
    
    methods(Static)
        
        function plotTree(allNodes,leafNodes,unsafeBoundaryValue)
            % Plot the tree using all the calculated nodes and edges
            % creating a digraph: source node -> edge -> target node
            s = [allNodes.sourceNodeID];
            t = [allNodes.targetNodeID];          
            G = digraph(s,t);
            
            % Change the labels of the digraph
            G.Edges.Labels =string([allNodes.sourceEdgeName])'; % Edge labels: Maneuvers
            G.Nodes.UnsafetyValues = [allNodes.UnsafetyValue]'; % Node labels: Unsafety values
            
            idx = [allNodes.id];
            Safest_idx = idx([allNodes.UnsafetyValue]<0.00001); % Mark safest nodes Green
            

            
            % Plot the tree from top down using 'Layout','layered'
            h=plot(G,'NodeLabel',G.Nodes.UnsafetyValues,'EdgeLabel',G.Edges.Labels,'Layout','layered');
            
            % Highlight Safe and Unsafe Nodes (Safe: Green, Unsafe: Red, InBetween: Blue)
            highlight(h,Safest_idx,'NodeColor','g')
            
            % Optional: Change the Labels from just UnsafetyValues to Unsafety + Liveness values
            NodeLabels = strcat('RLoU:',num2str(G.Nodes.UnsafetyValues,'%.4f'),'/ L:',num2str([vertcat([allNodes.state]).s]','%.1f'));
            h.labelnode(idx,string(NodeLabels));    

            % Highlight unsafe nodes with red
            Pruned_idx = Node.findUnsafeNodes(allNodes,idx, unsafeBoundaryValue);
            highlight(h,Pruned_idx,'NodeColor','r')    
            
            % Highlight edges that lead to unsafe nodes with red
            unsafeSourceTargetPairs = Node.findUnsafeEdges(G,Pruned_idx);
            highlight(h,unsafeSourceTargetPairs(:,1),unsafeSourceTargetPairs(:,2),'EdgeColor','r');
            
            % Find the safest state and track back to the initial state
            [val,idx_safest] = min([leafNodes.UnsafetyValue]);
            disp(['Safest state value: ',num2str(val)]);
            SafestNode = leafNodes(idx_safest);
            
            % Get the safe nodes ids
            SafePath_idx = Node.findTheSafestPath(allNodes,SafestNode);

            % Highlight all the nodes from initial state to the safest state
            highlight(h,SafePath_idx); 
            % Highlight all the edges leading to the safe state
            highlight(h,SafePath_idx(2:end),SafePath_idx(1:end-1)) 
                   
        end
        
        function Pruned_idx = findUnsafeNodes(allNodes,idx, unsafeBoundaryValue)
            
            Pruned_idx= idx([allNodes.UnsafetyValue]>unsafeBoundaryValue);% Mark unsafe nodes Red
            
            % Propagate pruned nodes if all children are pruned also mark parent node red
            while true
                [~,AllChildrenPruned_idx] = find(histcounts([allNodes(Pruned_idx).sourceNodeID])==3);
                
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

