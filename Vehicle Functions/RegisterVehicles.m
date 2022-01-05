classdef RegisterVehicles < matlab.System
% Register other vehicles on the road and their distances speeds

    properties(Nontunable)
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
    end

    methods(Access = protected)
        function [speedsSurroundingVehicles, distances2Vehicles] = stepImpl(obj, speedOtherVehicles, poseOtherVehicles, poseEgo)
        % Register s distances of the front and rear vehicles on the same lane and on the other lane
            % speedsSurroundingVehicles(1): Speed front vehicle on the same lane
            % speedsSurroundingVehicles(2): Speed rear vehicle on the same lane
            % speedsSurroundingVehicles(3): Speed front vehicle on the opposite lane
            % speedsSurroundingVehicles(4): Speed rear vehicle on the opposite lane
        
            % distances2Vehicles(1): Distance to front vehicle on the same lane
            % distances2Vehicles(2): Distance to rear vehicle on the same lane
            % distances2Vehicles(3): Distance to front vehicle on the opposite lane
            % distances2Vehicles(4): Distance to rear vehicle on the opposite lane
            
            [sEgo, dEgo] = Cartesian2Frenet(obj.RoadTrajectory, [poseEgo(1), poseEgo(2)]);
            [sOtherVehicles, dOtherVehicles] = Cartesian2Frenet(obj.RoadTrajectory, [poseOtherVehicles(1, :)', poseOtherVehicles(2, :)']);
            
            delta_d = dOtherVehicles - dEgo;
            
            % Vehicles on the same lane
            id_sameLane = abs(delta_d) <= 0.1; % Some tolerance
            [distances2Vehicles(1), distances2Vehicles(2), speedsSurroundingVehicles(1), speedsSurroundingVehicles(2)] = obj.getSurroundingVehicleInformation(id_sameLane, sEgo, sOtherVehicles, speedOtherVehicles);

            % Vehicles on the opposite lane
            id_otherLane = abs(delta_d) >= obj.LaneWidth/2;
            [distances2Vehicles(3), distances2Vehicles(4), speedsSurroundingVehicles(3), speedsSurroundingVehicles(4)] = obj.getSurroundingVehicleInformation(id_otherLane, sEgo, sOtherVehicles, speedOtherVehicles);
        end

        function [out1, out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [1 4];
            out2 = [1 4];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
    
    methods(Static)
        function [distanceFront, distanceRear, speedFront, speedRear] = getSurroundingVehicleInformation(id_lane, sEgo, sOtherVehicles, speedOtherVehicles)
        % Get s distance to front and rear vehicle and their speeds for specified lane
            
            % Very large default distance if there is no detection
            distanceFront = 999;
            distanceRear = 999;
            % Default speed -1 if there is no detection 
            speedFront = -1;
            speedRear = -1;
        
            delta_s = sOtherVehicles(id_lane) - sEgo;
            speeds = speedOtherVehicles(id_lane);
            
            id_front = delta_s >= 0; % Front
            if any(id_front)
                [distanceFront, id_frontMin] = min(delta_s(id_front));
                speedFront = speeds(id_frontMin);
            end
            
            id_rear = delta_s < 0; % Rear
            if any(id_rear)
                [distanceRear, id_rearMin] = min(abs(delta_s(id_rear)));
                distanceRear = -distanceRear;
                speedRear = speeds(id_rearMin);
            end
        end
    end
end
