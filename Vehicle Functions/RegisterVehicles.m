classdef RegisterVehicles < matlab.System
% Register other vehicles on the road and their distances

    properties(Nontunable)
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
    end

    methods(Access = protected)
        function distances2Vehicles = stepImpl(obj, poseOtherVehicles, poseEgo)
        % Register s distances of the front and rear vehicles on the same lane and on the other lane
            % distances2Vehicles(1): Distance to front vehicle on the same lane
            % distances2Vehicles(2): Distance to rear vehicle on the same lane
            % distances2Vehicles(3): Distance to front vehicle on the opposite lane
            % distances2Vehicles(4): Distance to rear vehicle on the opposite lane
            
            [sEgo, dEgo] = Cartesian2Frenet(obj.RoadTrajectory, [poseEgo(1), poseEgo(2)]);
            [sOtherVehicles, dOtherVehicles] = Cartesian2Frenet(obj.RoadTrajectory, [poseOtherVehicles(1, :)', poseOtherVehicles(2, :)']);
            
            delta_d = dOtherVehicles - dEgo;
            
            % Vehicles on the same lane
            id_sameLane = abs(delta_d) <= 0.1; % Some tolerance
            [distances2Vehicles(1), distances2Vehicles(2)] = obj.getVehicleDistances(id_sameLane, sEgo, sOtherVehicles);

            % Vehicles on the opposite lane
            id_otherLane = abs(delta_d) >= obj.LaneWidth/2;
            [distances2Vehicles(3), distances2Vehicles(4)] = obj.getVehicleDistances(id_otherLane, sEgo, sOtherVehicles);
        end

        function out = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 4];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function out = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function out = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function out = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
    
    methods(Static)
        function [distanceFront, distanceRear] = getVehicleDistances(id_lane, sEgo, sOtherVehicles)
        % Get s distance to front and rear vehicle on specified lane
            
            % Very large default distance, if there is no detection
            distanceFront = 999;
            distanceRear = 999;
        
            delta_s = sOtherVehicles(id_lane) - sEgo;
            
            id_front = delta_s >= 0; % Front
            if any(id_front)
                distanceFront = min(delta_s(id_front));
            end
            
            id_rear = delta_s < 0; % Rear
            if any(id_rear)
                distanceRear = -min(abs(delta_s(id_rear)));
            end
        end
    end
end
