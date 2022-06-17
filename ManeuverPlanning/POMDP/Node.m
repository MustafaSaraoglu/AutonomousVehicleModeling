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
            obj.actions = cellfun(@getName,actions,'UniformOutput',false);
            obj.UnsafetyValue = UnsafetyValue;
            obj.targetNodeID = []; % To be assigned if expanded
        end
        
        function childNode = expand(obj,maneuver,deltaT)
            % Expand the node based on available actions (edges out)
            %   Next states' values shall be calculated via applying the
            %   action for deltaT seconds
            newState=maneuver{1}.apply(obj.state,deltaT);
            newActions = Maneuver.getallActions();
            
            % Return the newly created child node
            childNode = Node(obj.id, maneuver, newState, newActions,[]);
        end
    end
   
end

