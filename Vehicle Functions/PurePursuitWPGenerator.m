classdef PurePursuitWPGenerator < LocalTrajectoryPlanner
% Provide reference waypoints for Pure Pursuit
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@LocalTrajectoryPlanner(obj)
        end

        function [nextWPs, d_ref, trajectoryToPlot] = stepImpl(obj, pose, changeLaneCmd, currentLane, velocity)
        % Return the reference waypoints necessary for Pure Pursuit, the reference lateral positon and the reference trajectory to plot

            [s, d] = obj.Cartesian2Frenet(obj.RoadTrajectory, [pose(1) pose(2)]);
            
            trajectoryFrenet = obj.planTrajectory(changeLaneCmd, currentLane, s, d, velocity);
            trajectoryCartesian = obj.Frenet2Cartesian(0, trajectoryFrenet(:, 1:2), obj.RoadTrajectory);
            trajectoryToPlot = getTrajectoryForPlotting(obj, trajectoryCartesian);
            
            [s_ref, d_ref, ~] = obj.getNextTrajectoryWaypoint(s);
            
            [refPos, ~] = obj.Frenet2Cartesian(0, [s_ref, d_ref], obj.RoadTrajectory);
            
            nextWPs = refPos;
        end
        
        function [out1, out2, out3] = getOutputSizeImpl(~)
            % Return size for each output port
            lengthTrajectory = obj.timeHorizon/obj.Ts;
            
            out1 = [1 2];
            out2 = [1 1];
            out3 = [lengthTrajectory 2];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2, out3] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2, out3] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
