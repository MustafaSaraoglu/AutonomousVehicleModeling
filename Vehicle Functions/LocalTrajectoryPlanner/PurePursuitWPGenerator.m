classdef PurePursuitWPGenerator < LocalTrajectoryPlanner
% Provide reference waypoints for Pure Pursuit
    
    properties(Nontunable)
        numberWaypoints % Number of waypoints to provide for Pure Pursuit
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@LocalTrajectoryPlanner(obj)
        end

        function [nextWPs, d_ref, steeringReachability] = stepImpl(obj, pose, poseOtherVehicles, speedsOtherVehicles, changeLaneCmd, plannerMode, velocity)
        % Return the reference waypoints, the reference lateral positon and the steeringReachability

            [s, d] = Cartesian2Frenet(obj.RoadTrajectory, [pose(1) pose(2)]);
            
            obj.planReferenceTrajectory(changeLaneCmd, plannerMode, s, d, velocity, pose(3), poseOtherVehicles, speedsOtherVehicles);
                
            % Boundary curves for steering reachability
            steeringReachability = obj.calculateSteeringReachability(pose, s, velocity);
            
            [s_ref, d_ref] = obj.getNextFrenetTrajectoryWaypoints(s, velocity, obj.numberWaypoints);
            
            [nextWPs, ~] = Frenet2Cartesian(s_ref, d_ref, obj.RoadTrajectory);

            d_ref = d_ref(1); % Only use first waypoint as current reference for d
        end
        
        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            numberPointsSteering =  2*ceil(obj.timeHorizon*rad2deg(abs(obj.steerAngle_max)));
            
            out1 = [obj.numberWaypoints, 2];
            out2 = [1 1];
            out3 = [numberPointsSteering, 10];

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
