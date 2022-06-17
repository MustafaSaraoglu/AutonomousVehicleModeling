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
            %   Next states' values shall be calculated via reachability
            %   based on the finite horizon
            
            % TODO later: Remove hard-coded maneuvers
%             if strcmp(maneuver, 'FD')
%                 speed_new = obj.state.speed +1;
%                 s_new = obj.state.s + speed_new*deltaT;
%                 d_new = obj.state.d;
%                 orientation_new = obj.state.orientation;
%                 
%             elseif strcmp(maneuver, 'EB')
%                 speed_new = obj.state.speed -3;
%                 s_new = obj.state.s + speed_new*deltaT;
%                 d_new = obj.state.d;
%                 orientation_new = obj.state.orientation;
%                 
%             elseif strcmp(maneuver, 'LC')
%                 speed_new = obj.state.speed;
%                 s_new = obj.state.s + speed_new*deltaT;
%                 if abs(obj.state.d)<0.05 % tolerance for d value
%                     d_new = 3.7;
%                 elseif abs(obj.state.d)-3.7<0.05
%                     d_new = 0;                    
%                 end
%                 orientation_new = obj.state.orientation;
%             end
            
            [speed_new,s_new,d_new,orientation_new]=maneuver{1}.apply(obj.state,deltaT);
            newState = obj.state; % Copy the object to create a state fast
            % Assign newly calculated values for the time horizon
            newState.s = s_new; 
            newState.d = d_new;
            newState.orientation = orientation_new;
            newState.speed = speed_new;
            %newActions= {'FD','EB','LC'};
            newActions = Maneuver.getallActions();
            
            % Return the newly created child node
            childNode = Node(obj.id, maneuver, newState, newActions,[]);
        end
    end
   
end

