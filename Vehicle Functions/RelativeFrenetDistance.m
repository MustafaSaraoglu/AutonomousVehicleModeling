classdef RelativeFrenetDistance < CoordinateTransformations
% Calculate the relative distance according to Frenet delta_s

    methods(Access = protected)
        function deltaS = stepImpl(obj, poseLead, poseEgo)
        % Return relative distance using Frenet coordinate system
        
            [sEgo, ~] = obj.Cartesian2Frenet(obj.RoadTrajectory, [poseEgo(1) poseEgo(2)]); 
            [sLead, ~] = obj.Cartesian2Frenet(obj.RoadTrajectory, [poseLead(1) poseLead(2)]); 
            
            deltaS = sLead - sEgo;
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
