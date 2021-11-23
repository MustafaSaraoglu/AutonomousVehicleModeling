classdef StanleyPoseGenerator < LocalTrajectoryPlanner
% Provide reference pose for Stanely Lateral Controller
    
    methods(Static)
        function [closestPoint, idxInTrajectory] = getClosestPointOnTrajectory(point, trajectory)
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

        function [d_ref, trajectoryToPlot, steeringReachability, referencePose, poseOut] = stepImpl(obj, pose, changeLaneCmd, currentLane, velocity)
        % Return the reference lateral position, the reference trajectory to plot, the reference pose and the current pose  
            
            [s, d] = Cartesian2Frenet(obj.RoadTrajectory, [pose(1) pose(2)]); 
            
            replan = obj.calculateTrajectoryError(s, d);
            obj.planFrenetTrajectory(changeLaneCmd, false, currentLane, s, d, pose(3), velocity);
            trajectoryCartesian = obj.getCurrentTrajectoryCartesian();
            trajectoryToPlot = trajectoryCartesian(:, 1:2);
            
            steeringReachability = obj.calculateSteeringReachability(pose, s, velocity);
            
            referencePose = obj.getReferencePoseStanley(pose, trajectoryCartesian); 
            
            [~, d_ref] = obj.getNextFrenetTrajectoryWaypoints(s, 1); % d_ref according to current pose and not according to front axle
            
            pose(3) = rad2deg(pose(3)); % Conversion necessary for MATLAB Stanley Lateral Controller
            poseOut = pose'; % MATLAB Stanley Lateral Controller input is [1x3]
        end
        
        function referencePoseCartesian = getReferencePoseStanley(obj, pose, trajectoryCartesian)
        % Get the reference pose for Stanley in Cartesian coordinates    
            
            % Reference is the center of the front axle
            centerFrontAxle = getVehicleFrontAxleCenterPoint(pose, obj.wheelBase);
            [referencePositionCartesian, idx] = obj.getClosestPointOnTrajectory(centerFrontAxle', trajectoryCartesian(:, 1:2));
            
            % Using slope of the curve
            delta_position = (trajectoryCartesian(idx+1, 1:2) - trajectoryCartesian(idx-1, 1:2))'; % [delta_x; delta_y]
            refOrientationCartesian = atan2(delta_position(2), delta_position(1));
            
            referencePoseCartesian = [referencePositionCartesian, rad2deg(refOrientationCartesian)]; % Degree for MATLAB Stanley Controller
        end
        
        function [out1, out2, out3, out4, out5] = getOutputSizeImpl(obj)
            % Return size for each output port
            lengthTrajectory = obj.timeHorizon/obj.Ts + 1;
            numberPointsSteering =  2*ceil(obj.timeHorizon*rad2deg(abs(obj.steerAngle_max)));
            
            out1 = [1 1];
            out2 = [lengthTrajectory, 2];
            out3 = [numberPointsSteering, 8];
            out4 = [1 3];
            out5 = [1 3];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2, out3, out4, out5] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";
            out4 = "double";
            out5 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2, out3, out4, out5] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2, out3, out4, out5] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

%         function sts = getSampleTimeImpl(obj)
%             % Define sample time type and parameters
% 
%             % Example: specify discrete sample time
%             sts = obj.createSampleTime("Type", "Discrete", ...
%                 "SampleTime", obj.Ts);
%         end      
    end
end
