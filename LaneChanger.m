classdef LaneChanger < matlab.System
    % Untitled3 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        t_i
        t_f
        a0
        a1
        a2
        a3
        a4
        a5
        overtake
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function latSpeed = stepImpl(obj,flag, clock, y)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            if flag
                
                if isempty(obj.a5)
                    obj.t_i = clock;
                    
                    d_i =         [1  obj.t_i   obj.t_i^2   obj.t_i^3    obj.t_i^4    obj.t_i^5]; % -2.5
                    d_dot_i =     [0  1   2*obj.t_i   3*obj.t_i^2  4*obj.t_i^3  5*obj.t_i^4]; %  0
                    d_ddot_i =    [0  0   2     6*obj.t_i    12*obj.t_i^2  20*obj.t_i^3]; %  0
                    
                    
                    obj.t_f = obj.t_i + 10; % delta T = 10 seconds
                    
                    d_f =         [1  obj.t_f   obj.t_f^2   obj.t_f^3    obj.t_f^4    obj.t_f^5]; % 2.5
                    d_dot_f =     [0  1   2*obj.t_f   3*obj.t_f^2  4*obj.t_f^3  5*obj.t_f^4]; % 0
                    d_ddot_f =    [0  0   2     6*obj.t_f    12*obj.t_f^2  20*obj.t_f^3]; % 0
                    
                    A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];
                    
                    B = [y; 0; 0; y+5; 0; 0];
                    
                    X = linsolve(A,B);
                    
                    obj.a0 = X(1);
                    obj.a1 = X(2);
                    obj.a2 = X(3);
                    obj.a3 = X(4);
                    obj.a4 = X(5);
                    obj.a5 = X(6);
                end
                
                if (clock>34) && (isempty(obj.overtake))
                    obj.overtake= true;
                    obj.t_i = clock;
                    
                    d_i =         [1  obj.t_i   obj.t_i^2   obj.t_i^3    obj.t_i^4    obj.t_i^5]; % -2.5
                    d_dot_i =     [0  1   2*obj.t_i   3*obj.t_i^2  4*obj.t_i^3  5*obj.t_i^4]; %  0
                    d_ddot_i =    [0  0   2     6*obj.t_i    12*obj.t_i^2  20*obj.t_i^3]; %  0
                    
                    
                    obj.t_f = obj.t_i + 10; % delta T = 10 seconds
                    
                    d_f =         [1  obj.t_f   obj.t_f^2   obj.t_f^3    obj.t_f^4    obj.t_f^5]; % 2.5
                    d_dot_f =     [0  1   2*obj.t_f   3*obj.t_f^2  4*obj.t_f^3  5*obj.t_f^4]; % 0
                    d_ddot_f =    [0  0   2     6*obj.t_f    12*obj.t_f^2  20*obj.t_f^3]; % 0
                    
                    A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];
                    
                    B = [y; 0; 0; y-5; 0; 0];
                    
                    X = linsolve(A,B);
                    
                    obj.a0 = X(1);
                    obj.a1 = X(2);
                    obj.a2 = X(3);
                    obj.a3 = X(4);
                    obj.a4 = X(5);
                    obj.a5 = X(6);
                    
                    
                    
                end
                
                if clock> obj.t_f
                    latSpeed=0;
                else
                    t = clock;
                    
                    latSpeed =  obj.a1  + 2*obj.a2*t +  3*obj.a3*t.^2  +  4*obj.a4*t.^3  +  5*obj.a5*t.^4;

                    
                end
                
            else
                latSpeed = 0;
            end
            
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
