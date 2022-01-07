classdef StanleyPoseGenerator < LocalTrajectoryPlanner
% Provide reference pose for Stanely Lateral Controller

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@LocalTrajectoryPlanner(obj)
        end

        function [d_ref, futurePosition, steeringReachability, referencePose, poseOut] = stepImpl(obj, pose, poseOtherVehicles, poseFutureOtherVehicles, changeLaneCmd, acceleration, velocity)
        % Return the reference lateral position, the reference trajectory to plot, the reference pose and the current pose  
            
            [s, d] = Cartesian2Frenet(obj.RoadTrajectory, [pose(1) pose(2)]); 
            
            replan = obj.calculateTrajectoryError(s, d);
            obj.planTrajectory(changeLaneCmd, replan, s, d, acceleration, velocity, poseOtherVehicles, poseFutureOtherVehicles);
            futurePosition = obj.futurePosition(:, 1:2);
            
            steeringReachability = obj.calculateSteeringReachability(pose, s, velocity);
            
            referencePose = obj.getReferencePoseStanley(pose); 
            
            [~, d_ref] = obj.getNextFrenetTrajectoryWaypoints(s, velocity, 1); % d_ref according to current pose and not according to front axle
            
            pose(3) = rad2deg(pose(3)); % Conversion necessary for MATLAB Stanley Lateral Controller
            poseOut = pose'; % MATLAB Stanley Lateral Controller input is [1x3]
        end
        
        function referencePoseCartesian = getReferencePoseStanley(obj, pose)
        % Get the reference pose for Stanley in Cartesian coordinates    
            
            % Reference is the center of the front axle
            centerFrontAxle = getVehicleFrontAxleCenterPoint(pose, obj.wheelBase);
            
            if ~isempty(obj.laneChangingTrajectoryCartesian)
                [referencePositionCartesian, idxReference] = obj.getClosestPointOnTrajectory(centerFrontAxle, obj.laneChangingTrajectoryCartesian(:, 1:2));
                refOrientation = obj.laneChangingTrajectoryCartesian(idxReference, 3);
                if idxReference >= size(obj.laneChangingTrajectoryCartesian, 1)
                    obj.laneChangingTrajectoryCartesian = [];
                end
            else
                [s, ~] = Cartesian2Frenet(obj.RoadTrajectory, centerFrontAxle); % Projection of font axle positon on current Frenet reference trajectory
                
                [referencePositionCartesian, refOrientation] = Frenet2Cartesian(s, obj.d_destination, obj.RoadTrajectory);
            end
            
            referencePoseCartesian = [referencePositionCartesian, rad2deg(refOrientation)]; % Degree for MATLAB Stanley Controller
        end
        
        function [out1, out2, out3, out4, out5] = getOutputSizeImpl(obj)
            % Return size for each output port
            numberPointsSteering =  2*ceil(obj.timeHorizon*rad2deg(abs(obj.steerAngle_max)));
            
            out1 = [1 1];
            out2 = [1 2];
            out3 = [numberPointsSteering, 10];
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
%             sts = obj.createSampleTime("Type", "Discrete", "SampleTime", obj.Ts);
%         end      
    end
    
    methods(Static)
        function [closestPoint, idxInTrajectory] = getClosestPointOnTrajectory(point, trajectory)
        % Calculate which point on a trajectory is closest to a given point
            
            [~, idxInTrajectory] = min(sum((trajectory - point).^2, 2));
            closestPoint = trajectory(idxInTrajectory, :);
        end
    end
end
