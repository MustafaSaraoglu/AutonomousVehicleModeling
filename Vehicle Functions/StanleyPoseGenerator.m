classdef StanleyPoseGenerator < LocalTrajectoryPlanner
% Provide reference pose for Stanely Lateral Controller
    
    properties(Nontunable)
        wheelBase % Wheel base vehicle
    end
    
    methods(Static)
        function [closestPoint, idxInTrajectory] = getClosestPointToTrajectory(point, trajectory)
        % Calculate which point on a trajectory is closest to a given point
            
            [~, idxInTrajectory] = min(sum((trajectory - point).^2, 2));
            closestPoint = trajectory(idxInTrajectory, :);
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@LocalTrajectoryPlanner(obj)
        end

        function [d_ref, trajectoryToPlot, referencePose, poseOut] = stepImpl(obj, pose, changeLaneCmd, currentLane, velocity)
        % Return the reference lateral position, the reference trajectory to plot, the reference pose and the current pose  
            
            [s, d] = obj.Cartesian2Frenet(obj.RoadTrajectory, [pose(1) pose(2)]); 
            
            trajectoryFrenet = obj.planTrajectory(changeLaneCmd, currentLane, s, d, velocity);
            trajectoryCartesian = obj.Frenet2Cartesian(0, trajectoryFrenet(:, 1:2), obj.RoadTrajectory);
            trajectoryToPlot = getTrajectoryToPlot(obj, trajectoryCartesian, currentLane);

% TODO: Correct dDot_ref/ orientation... Get orientation directly from trajectory atan2(delta_y,delta_x) for small Ts?         
%             referencePose = obj.getReferencePoseStanley(pose, trajectoryCartesian);
%             [s_ref, ~, ~]obj.Cartesian2Frenet(obj.RoadTrajectory, [referencePose(1) referencePose(2)]);
%             dDot_ref = trajectoryFrenet(trajectoryFrenet == s_ref, 3);
%             refOrientation = atan2(dDot_ref, velocity); % TODO: CHECK:MIGHT ONLY WORK FOR STRAIGHT ROADS
            
            [~, d_ref, ~] = obj.getNextTrajectoryWaypoint(s); % d_ref according to current pose and not according to rear axle
            
            referencePose = obj.getReferencePoseStanley(pose, trajectoryCartesian); 
            
            pose(3) = rad2deg(pose(3)); % Conversion necessary for MATLAB Stanley Lateral Controller
            poseOut = pose'; % MATLAB Stanley Lateral Controller input is [1x3]
        end
        
        function referencePoseCartesian = getReferencePoseStanley(obj, pose, trajectoryCartesian)
        % Get the reference position for Stanley in Cartesian coordinates    
            
            % Reference is the center of the front axle
            centerFrontAxle = getVehicleFrontAxleCenterPoint(pose, obj.wheelBase);
            [referencePositionCartesian, idx] = obj.getClosestPointToTrajectory(centerFrontAxle', trajectoryCartesian);
            delta_position = (trajectoryCartesian(idx+1, :) - trajectoryCartesian(idx-1, :))'; % [delta_x; delta_y]
            refOrientationCartesian = atan2(delta_position(2), delta_position(1));
            
            referencePoseCartesian = [referencePositionCartesian, rad2deg(refOrientationCartesian)]; % Degree for MATLAB Stanley Controller
        end
        
        function [out1, out2, out3, out4] = getOutputSizeImpl(obj)
            % Return size for each output port
            lengthTrajectory = obj.timeHorizon/obj.Ts;
            
            out1 = [1 1];
            out2 = [lengthTrajectory 2];
            out3 = [1 3];
            out4 = [1 3];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2, out3, out4] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";
            out4 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2, out3, out4] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2, out3, out4] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

%         function sts = getSampleTimeImpl(obj)
%             % Define sample time type and parameters
%             %  sts = obj.createSampleTime("Type", "Inherited");
% 
%             % Example: specify discrete sample time
%             sts = obj.createSampleTime("Type", "Discrete", ...
%                 "SampleTime", 'inherited');
%         end      
    end
end
