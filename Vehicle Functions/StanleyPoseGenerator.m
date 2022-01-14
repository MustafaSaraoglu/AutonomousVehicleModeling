classdef StanleyPoseGenerator < LocalTrajectoryPlanner
% Provide reference pose for Stanely Lateral Controller

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@LocalTrajectoryPlanner(obj)
        end

        function [d_ref, steeringReachability, referencePose, poseOut] = stepImpl(obj, pose, poseOtherVehicles, speedsOtherVehicles, changeLaneCmd, velocity)
        % Return the reference lateral position, the reference pose and the current pose  
            
            [s, d] = Cartesian2Frenet(obj.RoadTrajectory, [pose(1) pose(2)]); 
            
            if changeLaneCmd 
                % Store lane changing points if valid lane chaning trajectory found
                obj.calculateLaneChangingManeuver(changeLaneCmd, s, d, 0, 0, velocity, poseOtherVehicles, speedsOtherVehicles); 
            end
            
            % Boundary curves for steering reachability
            steeringReachability = obj.calculateSteeringReachability(pose, s, velocity);
            
            referencePose = obj.getReferencePoseStanley(pose); 
            
            [~, d_ref] = obj.getNextFrenetTrajectoryWaypoints(s, velocity, 1); % d_ref according to current pose and not according to front axle
            
            pose(3) = rad2deg(pose(3)); % Conversion necessary for MATLAB Stanley Lateral Controller
            poseOut = pose'; % MATLAB Stanley Lateral Controller input is [1x3]
        end
        
        function referencePoseCartesian = getReferencePoseStanley(obj, pose)
        % Get the reference pose for Stanley in Cartesian coordinates:
        % Reference is the point on the desired trajectory closest to the
        % center of the vehicle's front axle
            
            % Reference is the center of the front axle
            centerFrontAxle = getVehicleFrontAxleCenterPoint(pose, obj.wheelBase);
            
            % Use lane changing points from lane changing trajectory
            if ~isempty(obj.laneChangingTrajectoryCartesian)
                [referencePositionCartesian, idxReference] = obj.getClosestPointOnTrajectory(centerFrontAxle, obj.laneChangingTrajectoryCartesian(:, 1:2));
                refOrientation = obj.laneChangingTrajectoryCartesian(idxReference, 3);
                
                % Reset lane changing trajectory, if passed all lane changing points in trajectory
                if idxReference >= size(obj.laneChangingTrajectoryCartesian, 1)
                    obj.laneChangingTrajectoryCartesian = [];
                end
            else % No lane changing points
                % Projection of font axle positon on current Frenet reference trajectory
                [s, ~] = Cartesian2Frenet(obj.RoadTrajectory, centerFrontAxle); 
                
                [referencePositionCartesian, refOrientation] = Frenet2Cartesian(s, obj.d_destination, obj.RoadTrajectory);
            end
            
            referencePoseCartesian = [referencePositionCartesian, rad2deg(refOrientation)]; % Degree for MATLAB Stanley Controller
        end
        
        function [out1, out2, out3, out4] = getOutputSizeImpl(obj)
            % Return size for each output port
            numberPointsSteering =  2*ceil(obj.timeHorizon*rad2deg(abs(obj.steerAngle_max)));
            
            out1 = [1 1];
            out2 = [numberPointsSteering, 10];
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
    end
    
    methods(Static)
        function [closestPoint, idxInTrajectory] = getClosestPointOnTrajectory(point, trajectory)
        % Calculate which point on a trajectory is closest to a given point
            
            [~, idxInTrajectory] = min(sum((trajectory - point).^2, 2));
            closestPoint = trajectory(idxInTrajectory, :);
        end
    end
end
