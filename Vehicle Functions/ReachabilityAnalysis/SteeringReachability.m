classdef SteeringReachability < ReachabilityAnalysis
% Steering reachability with constant (minimum/maximum) acceleration assumption

    methods(Access = protected)
        function steeringReachability = stepImpl(obj, pose, v)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            steeringReachability = obj.calculateSteeringReachability(pose, v);
        end
        
        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            numberPointsSteering =  2*ceil(obj.timeHorizon*rad2deg(abs(obj.steerAngle_max)));
            
            out = [numberPointsSteering, 10];

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
