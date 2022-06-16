classdef Node
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sourceNodeID
        sourceEdgeName
        id
        state
        actions
        targetNodeID
        UnsafetyValue
    end
    
    methods
        function obj = Node(sourceNodeID,sourceEdgeName,state,actions,UnsafetyValue)
            %NODE Construct an instance of this class
            obj.sourceNodeID = sourceNodeID;
            obj.sourceEdgeName = sourceEdgeName;
            obj.id = counter();
            obj.state = state;
            obj.actions = actions;
            obj.UnsafetyValue = UnsafetyValue;
            obj.targetNodeID = [];
        end
        
        function childNode = expand(obj,maneuver,deltaT)
            
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
            newState = obj.state;
            newState.s = s_new;
            newState.d = d_new;
            newState.orientation = orientation_new;
            newState.speed = speed_new;
            newActions= {'FD','EB','LC'};
            childNode = Node(obj.id, maneuver, newState, newActions,[]);
        end
        

    end
    
    methods(Static)
        
        function plotTree(allNodes)
            
            s = [allNodes.sourceNodeID];
            t = [allNodes.targetNodeID];
            
            
            
            
            G = digraph(s,t);
            G.Edges.Labels =string([allNodes.sourceEdgeName])'; % Edge labels: Maneuvers
            G.Nodes.UnsafetyValues = [allNodes.UnsafetyValue]'; % Node labels: Unsafety values
            
            idx = [allNodes.id];
            Pruned_idx= idx([allNodes.UnsafetyValue]>0.02);% Mark unsafe nodes Red
            Safest_idx = idx([allNodes.UnsafetyValue]<0.00001); % Mark safest nodes Green
            
            h=plot(G,'NodeLabel',G.Nodes.UnsafetyValues,'EdgeLabel',G.Edges.Labels,'Layout','layered');
            
            
            highlight(h,Safest_idx,'NodeColor','g')
            
            % Propagate pruned nodes if all children are pruned also mark parent node red
            AllChildrenPruned_idx = [];
            while true
                [~,AllChildrenPruned_idx] = find(histcounts([allNodes(Pruned_idx).sourceNodeID])==3);
                
                if all(ismember(AllChildrenPruned_idx,Pruned_idx))
                    break;
                else
                    Pruned_idx = unique([Pruned_idx AllChildrenPruned_idx]);
                end
            end
            highlight(h,Pruned_idx,'NodeColor','r')
            
            % Highlight edges that lead to unsafe nodes
            unsafeEdges_idx=ismember(G.Edges.EndNodes(:,2), Pruned_idx);
            unsafeSourceTargetPairs =G.Edges.EndNodes(unsafeEdges_idx,:);
            
            highlight(h,unsafeSourceTargetPairs(:,1),unsafeSourceTargetPairs(:,2),'EdgeColor','r');
            
        end
    end
end

