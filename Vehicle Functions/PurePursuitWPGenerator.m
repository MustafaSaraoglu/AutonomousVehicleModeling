classdef PurePursuitWPGenerator < LocalTrajectoryPlanner
% Provide reference waypoints for Pure Pursuit
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@LocalTrajectoryPlanner(obj)
        end

        function [nextWPs, d_ref] = stepImpl(obj, pose, changeLaneCmd, currentLane, velocity)
        % Return the reference waypoints necessary for Pure Pursuit and the reference lateral positon

            [s, d] = obj.Cartesian2Frenet(obj.RoadTrajectory, [pose(1) pose(2)]);
            
            trajectoryFrenet = obj.planTrajectory(changeLaneCmd, currentLane, s, d, velocity);
%             plotTraj = trajectoryFrenet(1:75:end, :);
%             plot(plotTraj(:, 1), plotTraj(:, 2), 'Color', 'green');
            
            [s_ref, d_ref, ~] = obj.getNextTrajectoryWaypoint(s);
            
            [refPos, ~] = obj.Frenet2Cartesian(0, [s_ref, d_ref], obj.RoadTrajectory);
            
            nextWPs = refPos;
        end
        
        function [out1, out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [1 2];
            out2 = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
