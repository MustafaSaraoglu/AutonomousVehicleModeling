classdef Decision
% Decision for maneuvers
    
    properties
        TS % Discrete transition system for maneuver
        isFeasible % Feasibility of maneuver
        futureState % Future state after executing maneuver
        description % Maneuver description
        trajectoryFrenet_LC % Trajectory for lane change in Frenet coordinates
        trajectoryCartesian_LC % Trajectory for lane change in Cartesian coordinates
    end
    
    methods
        function obj = Decision(TS, isFeasible, futureState, description, trajectoryFrenet_LC, trajectoryCartesian_LC)
            %DECISION Construct an instance of this class
            obj.TS = TS;
            obj.isFeasible = isFeasible; 
            obj.futureState = futureState;
            obj.description = description;
            obj.trajectoryFrenet_LC = trajectoryFrenet_LC;
            obj.trajectoryCartesian_LC = trajectoryCartesian_LC;
        end
        
        function [decisions, id_next] = addDecisionToCell(obj, decisions, id)
        % Add new decisions to cell array at the given index
            
            decisions{id} = obj;
            id_next = id + 1;
        end
    end
end

