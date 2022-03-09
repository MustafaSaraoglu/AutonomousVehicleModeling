classdef FrenetTrajectory < Trajectory
% Frenet trajectory
    
    properties
        s % s trajectory
        d % d trajectory
    end
    
    methods
        function obj = FrenetTrajectory(time, s, d, v_trajectory, feasibility, cost)
            %FRENETTRAJECTORY Construct an instance of this class
            obj = obj@Trajectory(time, v_trajectory, feasibility, cost);
            obj.s = s;
            obj.d = d;
        end
    end
end

