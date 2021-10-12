classdef LaneChangeLatSpeed < matlab.System
    % Change Lane for overtaking maneuver using lateral speed

    % Public, tunable properties
    properties
        
    end
    
    properties(Nontunable)
        LaneWidth = 3.7;
        deltaT_LC = 5;
        deltaT_OT = 5;
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
        % Coefficents for lane changing trajectory
        a0
        a1
        a2
        a3
        a4
        a5
        % State if any maneuver should be executed
        %   0 = Stay in same lane
        %   1 = Change to left lane
        %  -1 = Change to right lane
        currentManeuver
        % Lateral destionation after executing maneuver
        y_f
        % Time to start maneuver
        t_start
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.currentManeuver = 0; % Initially do not execute any maneuver
        end

        function latSpeed = stepImpl(obj, y_ego, clock, changeLane)
            % Implement algorithm. 

            % Initialisation maneuver to right lane
            if changeLane == 1
                obj.initialiseManeuver(y_ego, obj.LaneWidth, changeLane, obj.deltaT_LC, clock);
            % Initialisation maneuver to left lane
            elseif changeLane == -1
                obj.initialiseManeuver(y_ego, 0, changeLane, obj.deltaT_OT, clock);
            end
            % Check if ego vehicle should execute maneuver
            if obj.currentManeuver
                % d = obj.a0 + obj.a1*t + obj.a2*t.^2 + obj.a3*t.^3 + obj.a4*t.^4 + obj.a5*t.^5;
                % End maneuver if lateral destionation is reached
                if (obj.currentManeuver > 0 && y_ego >= obj.y_f) || (obj.currentManeuver < 0 && y_ego <= obj.y_f)
                    obj.currentManeuver = 0;
                    latSpeed = 0;
                    return
                end
                % Calculate reference lateral speed according to reference
                % trajectory
                t = clock - obj.t_start;
                latSpeed = obj.a1 + 2*obj.a2*t + 3*obj.a3*t.^2 + 4*obj.a4*t.^3 + 5*obj.a5*t.^4;   
            else
                latSpeed = 0;
            end
        end
        
        function initialiseManeuver(obj, y_ego, y_f, maneuver, deltaManeuver, clock)
            % Initialise lane changing maneuver and calculate reference
            % trajectory
            obj.y_f = y_f;
            obj.calculateLaneChangingTrajectoryCoefficients(y_ego, y_f, deltaManeuver);
            obj.currentManeuver = maneuver;
            obj.t_start = clock;
        end
        
        function calculateLaneChangingTrajectoryCoefficients(obj, y_ego, y_f, deltaManeuver)
            % Calculate coefficients for minimum jerk trajectory
            t_i = 0; % Start at 0 (relative time frame)
                    
            d_i =         [1  t_i   t_i^2   t_i^3    t_i^4      t_i^5]; % y_i before lane change
            d_dot_i =     [0  1     2*t_i   3*t_i^2  4*t_i^3    5*t_i^4]; %  0
            d_ddot_i =    [0  0     2       6*t_i    12*t_i^2   20*t_i^3]; %  0


            t_f = deltaManeuver; % deltaT: time to finish maneuver

            d_f =         [1  t_f   t_f^2   t_f^3    t_f^4      t_f^5]; % y_f
            d_dot_f =     [0  1     2*t_f   3*t_f^2  4*t_f^3    5*t_f^4]; % 0
            d_ddot_f =    [0  0     2       6*t_f    12*t_f^2   20*t_f^3]; % 0

            A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];

            B = [y_ego; 0; 0; y_f; 0; 0];

            X = linsolve(A,B);

            obj.a0 = X(1);
            obj.a1 = X(2); 
            obj.a2 = X(3);
            obj.a3 = X(4);
            obj.a4 = X(5);
            obj.a5 = X(6);
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
