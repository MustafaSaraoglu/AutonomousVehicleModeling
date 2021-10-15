classdef PurePursuitWPGenerator < LocalTrajectoryPlanner
    % Provide reference waypoints for Pure Pursuit

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [nextWPs, d_ref] = stepImpl(obj, pose, changeLane, clock)
            % Implement algorithm. 

            % Cartesian to Frenet coordinate transformation
            [s, d_ref] = obj.Cartesian2Frenet(obj.CurrentTrajectory, [pose(1) pose(2)]); % Determine current <s,d>
            
            % Check whether to start or stop lane changing maneuver
            obj.checkForLaneChangingManeuver(changeLane, d_ref, clock);
            
            % Check if ego vehicle should execute maneuver
            if obj.currentManeuver % Add <delta d>
                % Calculate reference lateral position according to reference
                % trajectory
                t = clock - obj.t_start; 
                d_ref = obj.getReferenceLateralPosition(t);
            end
            
            % TODO: NECESSARY TO CONSIDER TIME AND CURRENT VELOCITY?
            s = s + 0.01; % Add <delta s>
            
            nextWPs = [s, d_ref];
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [out1, out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [1 2];
            out2 = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
