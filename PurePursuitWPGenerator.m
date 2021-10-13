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

        function nextWPs = stepImpl(obj, pose, changeLane, clock, velocity)
            % Implement algorithm. 

            % Cartesian to Frenet coordinate transformation
            [s, d] = obj.Cartesian2Frenet(obj.CurrentTrajectory, [pose(1) pose(2)]); % Determine current <s,d>
            
            % Check whether to start or stop lane changing maneuver
            obj.checkForLaneChangingManeuver(changeLane, d, clock);
            
            % Check if ego vehicle should execute maneuver
            if obj.currentManeuver % Add <delta d>
                % Calculate reference lateral position according to reference
                % trajectory
                t = clock - obj.t_start;
                d = obj.a0 + obj.a1*t + obj.a2*t.^2 + obj.a3*t.^3 + obj.a4*t.^4 + obj.a5*t.^5; 
            end
            
            % TODO: NECESSARY TO CONSIDER TIME AND CURRENT VELOCITY?
            s = s + 0.01; % Add <delta s>
            
            nextWPs = [s, d];
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 2];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function out = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function out = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function out = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
