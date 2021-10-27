classdef StanleyPoseGenerator < LocalTrajectoryPlanner
% Provide reference pose for Stanely Lateral Controller
    
    properties(Nontunable)
        wheelBase % Wheel base vehicle
    end
    
    methods(Static)
        function closestPoint = getClosestPointToTrajectory(point, trajectory)
        % Calculate which point on a trajectory is closest to a given point
            
            [~, idx] = min(sum((trajectory - point).^2, 2));
            closestPoint = trajectory(idx, :);
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@LocalTrajectoryPlanner(obj)
        end

        function [d_ref, referencePose, poseOut] = stepImpl(obj, pose, changeLaneCmd, currentLane, velocity)
        % Return the reference lateral position, the reference pose and the current pose  
            
            [s, d] = obj.Cartesian2Frenet(obj.RoadTrajectory, [pose(1) pose(2)]); 
            
            trajectoryFrenet = obj.planTrajectory(changeLaneCmd, currentLane, s, d, velocity);
%             plotTraj = trajectoryFrenet(1:75:end, :);
%             plot(plotTraj(:, 1), plotTraj(:, 2), 'Color', 'green');
            
            [s_ref, d_ref] = obj.getReferenceStanley(pose, trajectoryFrenet);

            if obj.executeManeuver
                [~, ~, dDot_ref] = obj.getNextTrajectoryWaypoint(s_ref);
                refOrientation = atan2(dDot_ref, velocity); % TODO: CHECK:MIGHT ONLY WORK FOR STRAIGHT ROADS
            else
                [~, refOrientation] = obj.Frenet2Cartesian(0, [s, d_ref], obj.RoadTrajectory);
            end
            
            [refPos, ~] = obj.Frenet2Cartesian(0, [s_ref, d_ref], obj.RoadTrajectory);
            
            [~, d_ref, ~] = obj.getNextTrajectoryWaypoint(s); % d_ref according to current pose and not according to rear axle
            
            pose(3) = rad2deg(pose(3)); % Conversion necessary for MATLAB Staneley Lateral Controller
            poseOut = pose'; % MATLAB Staneley Lateral Controller input is [1x3]
            
            referencePose = [refPos(1); refPos(2); rad2deg(refOrientation)]'; % Degree for MATLAB Stanley Controller
        end
        
        function [s_ref, d_ref] = getReferenceStanley(obj, pose, trajectoryFrenet)
            % Reference is the center of the front axle
            centerFrontAxle = getVehicleFrontAxleCenterPoint(pose, obj.wheelBase);
            trajectoryCartesian = obj.Frenet2Cartesian(0, trajectoryFrenet(:, 1:2), obj.RoadTrajectory);
            referencePositionCartesian = obj.getClosestPointToTrajectory(centerFrontAxle', trajectoryCartesian);
            [s_ref, d_ref] = obj.Cartesian2Frenet(obj.RoadTrajectory, referencePositionCartesian); 
        end
        
        function [out1, out2, out3] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [1 1];
            out2 = [1 3];
            out3 = [1 3];

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
