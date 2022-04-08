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

        function [nextWPs, d_ref] = stepImpl(obj, pose, changeLaneCmd, velocity)
        % Return the reference waypoints, the reference lateral positon and the steeringReachability

            [s, d] = Cartesian2Frenet(obj.RoadTrajectory, [pose(1) pose(2)]);
            
            obj.planReferenceTrajectory(changeLaneCmd, s, d, velocity);
            
            [s_ref, d_ref] = obj.getNextFrenetTrajectoryWaypoints(s, velocity, obj.numberWaypoints);
            
            [nextWPs, ~] = Frenet2Cartesian(s_ref, d_ref, obj.RoadTrajectory);

            d_ref = d_ref(1); % Only use first waypoint as current reference for d
        end
        
        function [out1, out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [obj.numberWaypoints, 2];
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
