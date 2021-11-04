classdef LongitudinalReachability < matlab.System
% Longitudinal reachability with constant (minimum/maximum) acceleration assumption

     properties(Nontunable)
        minimumAcceleration % Minimum longitudinal acceleration
        maximumAcceleration % Maximum longitudinal acceleration
         
        timeHorizon % Time horizon for trajectory genereation [s]
        Ts % Sampling time for trajectory generation [s]
    end

    % Pre-computed constants
    properties(Access = private)
        k % Number of discrete time steps
        
        A_prime % Modified A-Matrix for reachability analysis
        B_prime % Modified B-Matrix for reachability analysis

        counter % Counter to stop at correct simulation time
        futureStatePrediction % Store future state predictions for verification
        err_s_v % Error between predicted s and actual s and predicted velocity and actual velocity [t, deltaS, deltaV]
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            obj.k = obj.timeHorizon/obj.Ts - 1; % -1 because state(k+1) should be equal to time horizon
            
            obj.A_prime = obj.calculateAPrime(obj.k);
            obj.B_prime = obj.calculateBPrime(obj.k);

            obj.counter = 0;
            obj.futureStatePrediction = [];
            time = (obj.timeHorizon:obj.timeHorizon:str2double(get_param('VehicleFollowing', 'StopTime')))';
            obj.err_s_v = [time, zeros(length(time), 2)];
        end

        function [sFuture_min, sFuture_max] = stepImpl(obj, velocity_0, s_0)
        % Return minimum and maximum predicted future s
            
            initialState = [s_0; velocity_0];
                    
            futureState_min = obj.A_prime*initialState + obj.B_prime*obj.minimumAcceleration;
            if futureState_min(2) < 0
                % Account for the -speed error in the prediction and
                % correct the position value and set predicted speed to 0
                
                futureState_min(2) = 0;
                t_stop = -velocity_0/obj.minimumAcceleration; % v(t) = 0 = acc*t + v_0 if acc = const.
                futureState_min(1) = s_0 + velocity_0*t_stop/2;
            end
            sFuture_min = futureState_min(1);
            
            futureState_max = obj.A_prime*initialState + obj.B_prime*obj.maximumAcceleration;
            sFuture_max = futureState_max(1);
            
            obj.verifyReachabilityAnalysis(initialState);
        end

        function verifyReachabilityAnalysis(obj, initialState)
        % Track error between predicted state and actual state every time horizon seconds
           
            if get_param('VehicleFollowing', 'SimulationTime') > obj.counter*obj.timeHorizon
                aLead_ref = 0;
                futreStatePrediction = obj.A_prime*initialState + obj.B_prime*aLead_ref;
                if isempty(obj.futureStatePrediction)
                    obj.futureStatePrediction = futreStatePrediction;
                else
                    error_s_v = obj.futureStatePrediction - initialState;
                    obj.err_s_v(obj.counter, 2:3) = error_s_v';
                    obj.futureStatePrediction = futreStatePrediction;
                end 
                 obj.counter = obj.counter + 1;
            end
        end

        function A_prime = calculateAPrime(obj, k)
        % Calculate modified A-Matrix for k time steps

            A_prime = [1, (k+1)*obj.Ts; 
                       0,       1      ];
        end

        function B_prime = calculateBPrime(obj, k)
        % Calculate modified B-Matrix for k time steps

            B_prime = [obj.Ts^2/2*sum(1:2:(2*k+1)); 
                            obj.Ts*(k+1)           ];
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
