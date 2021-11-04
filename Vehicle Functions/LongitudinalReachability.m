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
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            obj.k = obj.timeHorizon/obj.Ts; 
            
            obj.A_prime = obj.calculateAPrime(obj.k);
             
            obj.B_prime = obj.calculateBPrime(obj.k);
        end

        function [sFuture_min, sFuture_max] = stepImpl(obj, velocity_0, s_0)
        % Return minimum and maximum predicted future s
            
            initialState = [s_0; velocity_0];
                    
            futureState_min = obj.A_prime*initialState + obj.B_prime*obj.minimumAcceleration; % TODO: Need to consider/restrict backward motion?
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
