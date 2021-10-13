classdef StanleyPoseGenerator < LocalTrajectoryPlanner
    % Provide reference pose for Stanely Lateral Controller

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

        function [referencePose, poseOut] = stepImpl(obj, pose, changeLane, clock)
            % Implement algorithm. 
            currentTrajectory = obj.CurrentTrajectory;
            Vpos_C = [pose(1) pose(2)];
            pose(3) = rad2deg(pose(3)); % Conversion necessary fpr MATLAB Staneley Lateral Controller
            
            % Cartesian to Frenet Coordinate Transformation
            [s, d] = obj.Cartesian2Frenet(currentTrajectory, Vpos_C); % Determine current <s,d>
            
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
            if obj.currentManeuver % Add <delta d>
                % Calculate reference lateral position according to reference
                % trajectory
                t = clock - obj.t_start;
                d = obj.a0 + obj.a1*t + obj.a2*t.^2 + obj.a3*t.^3 + obj.a4*t.^4 + obj.a5*t.^5; 
            end
            s = s + 0.01; % Add <delta s>
            % Generate Reference Pose for Stanley
            [refPos, refOrientation] = obj.Frenet2Cartesian(s, [s, d], currentTrajectory); % Coordinate Conversion function
            
            poseOut = pose';
            referencePose = [refPos(1); refPos(2); refOrientation]';
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [out1, out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [1 3];
            out2 = [1 3];

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
