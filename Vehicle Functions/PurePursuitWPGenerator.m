classdef PurePursuitWPGenerator < LocalTrajectoryPlanner
% Provide reference waypoints for Pure Pursuit
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@LocalTrajectoryPlanner(obj)
        end

        function [nextWPs, d_ref] = stepImpl(obj, pose, changeLaneCmd, velocity)
        % Return the reference waypoints necessary for Pure Pursuit and the reference lateral positon

            [s, d_ref] = obj.Cartesian2Frenet(obj.CurrentTrajectory, [pose(1) pose(2)]);
            
            % Check whether to start or stop lane changing maneuver
            obj.checkForLaneChangingManeuver(changeLaneCmd, s, d_ref, velocity);
            
            if obj.executeManeuver 
                [s_ref, d_ref, ~] = obj.getNextTrajectoryWaypoint(s);
            else
                s_ref = s + 0.01; % Waypoint ahead on the same lane
            end
            
            [refPos, ~] = obj.Frenet2Cartesian(0, [s_ref, d_ref], obj.CurrentTrajectory);
            
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
