classdef CartesianTrajectory < Trajectory
% Cartesian trajectory
    
    properties
        x % x trajectory
        y % y trajectory
        orientation % orientation trajectory
    end
    
    methods
        function obj = CartesianTrajectory(time, x, y, orientation, v_trajectory, feasibility, cost)
            %CARTESIANTRAJECTORY Construct an instance of this class
            obj = obj@Trajectory(time, v_trajectory, feasibility, cost);
            obj.x = x;
            obj.y = y;
            obj.orientation = orientation;
        end
        
        function [closestPoint, idxInTrajectory] = getClosestPointOnTrajectory(obj, point)
        % Calculate which point on a trajectory is closest to a given point
            
            trajectoryPositions = [obj.x, obj.y];
            [~, idxInTrajectory] = min(sum((trajectoryPositions - point).^2, 2));
            closestPoint = trajectoryPositions(idxInTrajectory, :);
        end
    end
end

