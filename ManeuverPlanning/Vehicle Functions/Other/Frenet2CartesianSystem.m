classdef Frenet2CartesianSystem < matlab.System
% Frenet2Cartesian block for different input sizes

    properties(Nontunable)
        RoadTrajectory % Road trajectory according to MOBATSim map format
        lengthOut % Length of output
    end

    methods(Access = protected)
        function [x, y, refOrientation] = stepImpl(obj, s, d)
        % Frenet2Cartesian block for different input sizes
            
            [position, refOrientation] = Frenet2Cartesian(s, d, obj.RoadTrajectory);
            x = position(:, 1);
            y = position(:, 2);
        end
        
        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [obj.lengthOut 1];
            out2 = [obj.lengthOut 1];
            out3 = [obj.lengthOut 1];

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
