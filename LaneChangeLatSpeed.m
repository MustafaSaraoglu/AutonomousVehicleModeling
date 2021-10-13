classdef LaneChangeLatSpeed < LocalTrajectoryPlanner
    % Change Lane for overtaking maneuver using lateral speed

    % Public, tunable properties
    properties
        
    end
    
    properties(Nontunable)

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.currentManeuver = 0; % Initially do not execute any maneuver
        end

        function SteerCmd = stepImpl(obj, pose, clock, changeLane, velocity)
            % Implement algorithm.
            currentTrajectory = obj.CurrentTrajectory;
            Vpos_C = [pose(1) pose(2)];
            
            % Cartesian to Frenet Coordinate Transformation
            [~, d] = obj.Cartesian2Frenet(currentTrajectory, Vpos_C); % Determine current <s,d>

            % Initialisation maneuver to right lane
            if changeLane == 1
                obj.initialiseManeuver(d, obj.LaneWidth, changeLane, obj.deltaT_LC, clock);
            % Initialisation maneuver to left lane
            elseif changeLane == -1
                obj.initialiseManeuver(d, 0, changeLane, obj.deltaT_OT, clock);
            elseif changeLane == 2
                obj.currentManeuver = 0;
            end
            % Check if ego vehicle should execute maneuver
            if obj.currentManeuver
                % Calculate reference lateral speed according to reference
                % trajectory
                t = clock - obj.t_start;
                latSpeed = obj.a1 + 2*obj.a2*t + 3*obj.a3*t.^2 + 4*obj.a4*t.^3 + 5*obj.a5*t.^4;   
            else
                latSpeed = 0;
            end
            SteerCmd = atan2(latSpeed, velocity);
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];

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
