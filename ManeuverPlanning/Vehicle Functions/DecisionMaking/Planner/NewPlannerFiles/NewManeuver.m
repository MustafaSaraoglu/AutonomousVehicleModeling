classdef NewManeuver
% Decision for maneuvers
    
    properties
        trajectoryDiscrete % Discrete trajectory for maneuver
        isFeasible % Feasibility of maneuver
        futureState % Future state after executing maneuver
        description % Maneuver description
        trajectoryFrenet_LC % Trajectory for lane change in Frenet coordinates
        trajectoryCartesian_LC % Trajectory for lane change in Cartesian coordinates
        
        
    end
    
    methods
        function obj = NewManeuver(trajectoryDiscrete, isFeasible, futureState, description, ...
                                trajectoryFrenet_LC, trajectoryCartesian_LC)
            %DECISION Construct an instance of this class
            if nargin > 0
                obj.trajectoryDiscrete = trajectoryDiscrete;
                obj.isFeasible = isFeasible; 
                obj.futureState = futureState;
                obj.description = description;
                obj.trajectoryFrenet_LC = trajectoryFrenet_LC;
                obj.trajectoryCartesian_LC = trajectoryCartesian_LC;
            end
        end
        
        function [decisions, id_next] = addDecisionToArray(obj, decisions, id)
        % Add new decisions to Decision array at the given index
            
            decisions(id) = obj;
            id_next = id + 1;
        end
    end
end

