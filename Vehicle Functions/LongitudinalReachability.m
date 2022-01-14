classdef LongitudinalReachability < ReachabilityAnalysis
% Longitudinal reachability with constant (minimum/maximum) acceleration assumption

    % Pre-computed constants
    properties(Access = private)
        counter % Counter to stop at correct simulation time
        futureStatePrediction % Store future state predictions for verification
        err_s_v % Error between predicted s and actual s and predicted velocity and actual velocity [t, deltaS, deltaV]
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@ReachabilityAnalysis(obj) 

            obj.counter = 0;
            obj.futureStatePrediction = [];
            time = (obj.timeHorizon:obj.timeHorizon:str2double(get_param('VehicleFollowing', 'StopTime')))';
            obj.err_s_v = [time, zeros(length(time), 2)];
        end

        function [sFuture_min, sFuture_max] = stepImpl(obj, v_0, s_0)
        % Return minimum and maximum predicted future s
            
            [sFuture_min, ~] = obj.predictLongitudinalFutureState(s_0, v_0, obj.maximumVelocity, obj.minimumAcceleration, obj.k_timeHorizon);
            [sFuture_max, ~] = obj.predictLongitudinalFutureState(s_0, v_0, obj.maximumVelocity, obj.maximumAcceleration, obj.k_timeHorizon);
        end
        
        % TODO: Remove unused function
        function verifyReachabilityAnalysis(obj, initialState)
        % Track error between predicted state and actual state every time horizon seconds
           
            if get_param('VehicleFollowing', 'SimulationTime') > obj.counter*obj.timeHorizon
                aLead_ref = 0; % TODO: Other than constant velocity
                if ~isempty(obj.futureStatePrediction)
                    error_s_v = obj.futureStatePrediction - initialState;
                    obj.err_s_v(obj.counter, 2:3) = error_s_v';
                end 
                
                [sFuture, vFuture] = obj.predictLongitudinalFutureState(initialState(1), initialState(2), obj.maximumVelocity, aLead_ref, obj.k_timeHorizon);
                obj.futureStatePrediction = [sFuture; vFuture];
                obj.counter = obj.counter + 1; 
            end
        end
    
        function [out1, out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [1 1];
            out2 = [1 1];

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
end
