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
        end

        function [SteerCmd, d_ref] = stepImpl(obj, pose, changeLane, clock, velocity)
            % Calculate steering command and the reference lateral postion
            % by using the lateral speed

            [~, d_ref] = obj.Cartesian2Frenet(obj.CurrentTrajectory, [pose(1) pose(2)]);
            
            % Check whether to start or stop lane changing maneuver
            obj.checkForLaneChangingManeuver(changeLane, d_ref, clock);
            
            if obj.executeManeuver
                % Calculate reference lateral speed according to reference
                % trajectory
                t = clock - obj.t_start;
                latSpeed = obj.a1 + 2*obj.a2*t + 3*obj.a3*t.^2 + 4*obj.a4*t.^3 + 5*obj.a5*t.^4; 
                
                [d_ref, ~] = obj.getLateralReference(t);
            else
                latSpeed = 0;
            end
            SteerCmd = atan2(latSpeed, velocity);
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function [out1, out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [1 1];
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
