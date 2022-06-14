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
            plot(G,'Layout','layered');
            
        end
    end
end

