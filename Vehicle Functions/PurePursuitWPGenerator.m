classdef PurePursuitWPGenerator < LocalTrajectoryPlanner
% Provide reference waypoints for Pure Pursuit
    
    properties(Nontunable)
        numberWaypoints % Number of waypoints to provide for Pure Pursuit
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@LocalTrajectoryPlanner(obj)
        end

        function [nextWPs, d_ref, trajectoryToPlot] = stepImpl(obj, pose, changeLaneCmd, currentLane, velocity)
        % Return the reference waypoints necessary for Pure Pursuit, the reference lateral positon and the reference trajectory to plot

            [s, d] = Cartesian2Frenet(obj.RoadTrajectory, [pose(1) pose(2)]);
            
            obj.planFrenetTrajectory(changeLaneCmd, currentLane, s, d, velocity);
            trajectoryCartesian = obj.getCurrentTrajectoryCartesian();
            trajectoryToPlot = trajectoryCartesian(:, 1:2);
            
            obj.calculateTrajectoryError(s, d);
            
            [s_ref, d_ref] = obj.getNextFrenetTrajectoryWaypoints(s, obj.numberWaypoints);
            
            [referencePositionCartesian, ~] = Frenet2Cartesian(s_ref, d_ref, obj.RoadTrajectory);
            
            nextWPs = referencePositionCartesian;
            
            d_ref = d_ref(1); % Only use first waypoint as current reference for d
        end
        
        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            lengthTrajectory = obj.timeHorizon/obj.Ts + 1;
            
            out1 = [obj.numberWaypoints 2];
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
