classdef RelativeFrenetDistance < CoordinateTransformations
    % Calculate the relative distance according to Frenet delta_s

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function deltaS = stepImpl(obj, poseLead, poseEgo)
            % Implement algorithm. 
            
            % Cartesian to Frenet coordinate transformation
            [sEgo, ~] = obj.Cartesian2Frenet(obj.CurrentTrajectory, [poseEgo(1) poseEgo(2)]); % Determine current <s,d>
            [sLead, ~] = obj.Cartesian2Frenet(obj.CurrentTrajectory, [poseLead(1) poseLead(2)]); 
            
            deltaS = sLead - sEgo;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function out = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function out = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function out = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end
    end
end
