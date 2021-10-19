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
            setupImpl@LocalTrajectoryPlanner(obj)
        end

        function [d_ref, referencePose, poseOut] = stepImpl(obj, pose, changeLaneCmd, clock, velocity)
            % Return the reference lateral position, the reference pose and 
            % the current pose 
            
            pose(3) = rad2deg(pose(3)); % Conversion necessary for MATLAB Staneley Lateral Controller

            [s, d_ref] = obj.Cartesian2Frenet(obj.CurrentTrajectory, [pose(1) pose(2)]); 
            
            % Check whether to start or stop lane changing maneuver
            obj.checkForLaneChangingManeuver(changeLaneCmd, d_ref, clock);

            if obj.executeManeuver
                % Calculate reference lateral position according to reference
                % trajectory
                t = clock - obj.t_start;
                [d_ref, dDot_ref] = obj.getLateralReference(t);
                refOrientation = atan2(dDot_ref, velocity); % TODO: CHECK:MIGHT ONLY WORK FOR STRAIGHT ROADS
            else
                % Use road geometry as reference orientation
                [~, refOrientation] = obj.Frenet2Cartesian(s, [s, d_ref], obj.CurrentTrajectory);
            end
            
            % TODO: NECESSARY TO CONSIDER TIME AND CURRENT VELOCITY?
            s = s + 0.01;
            
            [refPos, ~] = obj.Frenet2Cartesian(0, [s, d_ref], obj.CurrentTrajectory);

            poseOut = pose'; % MATLAB Staneley Lateral Controller input is [1x3]
            referencePose = [refPos(1); refPos(2); rad2deg(refOrientation)]'; % Degree for MATLAB Stanley Controller
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [1 1];
            out2 = [1 3];
            out3 = [1 3];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2, out3] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2, out3] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2, out3] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
