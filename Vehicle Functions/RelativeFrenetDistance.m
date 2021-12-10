classdef RelativeFrenetDistance < matlab.System
% Calculate the relative distance according to Frenet delta_s

    properties(Nontunable)
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
    end
    
    methods(Access = protected)
        function delta_s = stepImpl(obj, poseLead, poseEgo)
        % Return relative distance using Frenet coordinate system
        
            [sEgo] = Cartesian2Frenet(obj.RoadTrajectory, [poseEgo(1) poseEgo(2)]); 
            [sLead] = Cartesian2Frenet(obj.RoadTrajectory, [poseLead(1) poseLead(2)]); 
            
            delta_s = sLead - sEgo; 
        end
        
        function out = getOutputSizeImpl(~)
            % Return size for each output port
            out = [1 1];

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
end
