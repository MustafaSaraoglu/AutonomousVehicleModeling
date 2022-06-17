classdef GameTree
    %GAMETREE Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        function plot(allNodes,leafNodes,unsafeBoundaryValue)
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
            Pruned_idx = GameTree.findUnsafeNodes(allNodes,idx, unsafeBoundaryValue);
            highlight(h,Pruned_idx,'NodeColor','r')
            
            % Highlight edges that lead to unsafe nodes with red
            unsafeSourceTargetPairs = GameTree.findUnsafeEdges(G,Pruned_idx);
            highlight(h,unsafeSourceTargetPairs(:,1),unsafeSourceTargetPairs(:,2),'EdgeColor','r');
            
            % Find the safest state and track back to the initial state
            [val,idx_safest] = min([leafNodes.UnsafetyValue]);
            disp(['Safest state value: ',num2str(val)]);
            SafestNode = leafNodes(idx_safest);
            
            % Get the safe nodes ids
            SafePath_idx = GameTree.findTheSafestPath(allNodes,SafestNode);
            
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

