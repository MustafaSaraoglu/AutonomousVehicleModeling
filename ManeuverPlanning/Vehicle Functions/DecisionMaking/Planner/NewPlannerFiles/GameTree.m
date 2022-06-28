classdef GameTree
    %GAMETREE Summary of this class goes here
    %   Detailed explanation goes here
    properties
        chosenManeuver
        
        digraph
        visualization
    end
    
    methods
        function obj = GameTree(rootNodes,leafNodes,unsafeBoundaryValue)
            
            % Make an array of all nodes
            allNodes = [rootNodes leafNodes];
            
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
            
            % Plot the tree from top down using 'Layout','layered'
            h=plot(G,'NodeLabel',G.Nodes.UnsafetyValues,'EdgeLabel',G.Edges.Labels,'Layout','layered');
            
            % Highlight Safe and Unsafe Nodes (Safe: Green, Unsafe: Red, InBetween: Blue)
            highlight(h,Safest_idx,'NodeColor','g')
            
            % Optional: Change the Labels from just UnsafetyValues to Unsafety + Liveness values
            NodeLabels = strcat('RLoU:',num2str(G.Nodes.UnsafetyValues,'%.4f'),'/ L:',num2str([vertcat([allNodes.state]).s]','%.1f'));
            h.labelnode(idx,string(NodeLabels));
            
            % Highlight unsafe nodes with red
            Pruned_idx = GameTree.findUnsafeNodes(allNodes,idx, unsafeBoundaryValue);
            highlight(h,Pruned_idx,'NodeColor','r')
            
            % Highlight edges that lead to unsafe nodes with red
            unsafeSourceTargetPairs = GameTree.findUnsafeEdges(G,Pruned_idx);
            highlight(h,unsafeSourceTargetPairs(:,1),unsafeSourceTargetPairs(:,2),'EdgeColor','r');
            
            % Find the highest liveness value among the safest states
            SafestNode = GameTree.findTheBestState(leafNodes,Safest_idx);

            % Get the safe node's id and track back to the initial state
            SafePath_idx = GameTree.findTheSafestPath(allNodes,SafestNode);
            
            % Highlight all the nodes from initial state to the safest state
            highlight(h,SafePath_idx);
            % Highlight all the edges leading to the safe state
            highlight(h,SafePath_idx(2:end),SafePath_idx(1:end-1))
            % Output the first maneuver of the safe path
            obj.chosenManeuver = allNodes(2).sourceEdgeName{1}.getName;
            
            
            obj.digraph = G;
            obj.visualization = h;
            
        end
    end
    
    methods (Static)
        
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

            disp(['Safest state value: ',num2str(Safe_val)]);
            disp(['Liveness state value: ',num2str(Live_val)]);
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

