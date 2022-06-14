classdef Node
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sourceNodeID
        sourceEdgeName
        id
        state
        actions
    end
    
    methods
        function obj = Node(sourceNodeID,sourceEdgeName,state,actions)
            %NODE Construct an instance of this class
            obj.sourceNodeID = sourceNodeID;
            obj.sourceEdgeName = sourceEdgeName;
            obj.id = counter();
            obj.state = state;
            obj.actions = actions; 
        end
        
        function childNode = expand(obj,maneuver)
            newState= 404;
            newActions= 405;
            childNode = Node(obj.id, maneuver, newState, newActions);
        end
    end
end

