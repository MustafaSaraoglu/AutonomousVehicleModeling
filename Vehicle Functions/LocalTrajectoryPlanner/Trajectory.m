classdef Trajectory
% Superclass for Frenet and Cartesian Trajectory
    
    properties
        velocity % Velocity trajectory
        feasibility % Feasibility of trajectory
        cost % Cost of this trajectory
        time % Trajectory time
        length % Trajectory length
    end
    
    methods
        function obj = Trajectory(time, v_trajectory, feasibility, cost)
            %TRAJECTORY Construct an instance of this class
            obj.velocity = v_trajectory;
            obj.feasibility = feasibility;
            obj.cost = cost;
            obj.time = time;
            obj.length = length(time);
        end
        
        function isFeasible = isFeasible(obj)
            % Return if this trajectory is feasible
            
            isFeasible = obj.feasibility;
        end
    end
end

