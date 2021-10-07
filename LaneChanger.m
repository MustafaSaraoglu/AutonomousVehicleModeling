classdef LaneChanger < matlab.System
    % Change Lane for overtaking maneuver

    % Public, tunable properties
    properties
        
    end
    
    properties(Nontunable)
        LaneWidth = 5;
        deltaT_LC = 5;
        deltaT_OT = 5;
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
        s_thresh
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.s_thresh = 40;
        end

        function latSpeed = stepImpl(obj, clock, vLead, poseLead, vEgo, poseEgo)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            % Pose ego vehicle
            x_ego = poseEgo(1);
            y_ego = poseEgo(2);
            % yaw_ego = poseEgo(3);
            
            % Pose lead vehicle
            x_lead = poseLead(1);
            % y_lead = poseLead(2);
            % yaw_lead = poseLead(3);
            
            
            newWP_all = [];
            % Only start lane changing maneuver if necessary
            if (x_lead - x_ego >= obj.s_thresh) || vLead >= vEgo 
                latSpeed = 0;
                return
            else
                % Lane change
                if isempty(obj.a5)
                    obj.t_i = clock;
                    
                    d_i =         [1  obj.t_i   obj.t_i^2   obj.t_i^3    obj.t_i^4    obj.t_i^5]; % y_i before lane change
                    d_dot_i =     [0  1         2*obj.t_i   3*obj.t_i^2  4*obj.t_i^3  5*obj.t_i^4]; %  0
                    d_ddot_i =    [0  0         2           6*obj.t_i    12*obj.t_i^2  20*obj.t_i^3]; %  0
                    
                    
                    obj.t_f = obj.t_i + obj.deltaT_LC; % deltaT: time to finish maneuver (lane change)
                    
                    d_f =         [1  obj.t_f   obj.t_f^2   obj.t_f^3    obj.t_f^4    obj.t_f^5]; % y_i + w_lane
                    d_dot_f =     [0  1         2*obj.t_f   3*obj.t_f^2  4*obj.t_f^3  5*obj.t_f^4]; % 0
                    d_ddot_f =    [0  0         2           6*obj.t_f    12*obj.t_f^2  20*obj.t_f^3]; % 0
                    
                    A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];
                    
                    B = [y_ego; 0; 0; y_ego+obj.LaneWidth; 0; 0];
                    
                    X = linsolve(A,B);
                    
                    obj.a0 = X(1);
                    obj.a1 = X(2); 
                    obj.a2 = X(3);
                    obj.a3 = X(4);
                    obj.a4 = X(5);
                    obj.a5 = X(6);
                    
                    % Sollte zeitunabhängig sein !!!
                    % Mit Frenet arbeiten !!!
                    
                    tP = obj.t_i:0.2:obj.t_f;
                    tL = 0:0.2:obj.t_f-obj.t_i;
                    newWP_s = x_ego + vEgo*tL; % Longitudinal motion
                    newWP_d = obj.a0 + obj.a1*tP + obj.a2*tP.^2 + obj.a3*tP.^3 + obj.a4*tP.^4 + obj.a5*tP.^5; % Lateral motion
                    newWP_all =  [newWP_s' newWP_d'];
                end
                % Cut in
                if (x_ego - x_lead >= obj.s_thresh/2) && (isempty(obj.overtake))
                    obj.overtake = true;
                    obj.t_i = clock;
                    
                    d_i =         [1  obj.t_i   obj.t_i^2   obj.t_i^3    obj.t_i^4    obj.t_i^5]; % y_i before overtake
                    d_dot_i =     [0  1         2*obj.t_i   3*obj.t_i^2  4*obj.t_i^3  5*obj.t_i^4]; %  0
                    d_ddot_i =    [0  0         2           6*obj.t_i    12*obj.t_i^2  20*obj.t_i^3]; %  0
                    
                    
                    obj.t_f = obj.t_i + obj.deltaT_OT; % deltaT: time to finish maneuver (cut in)
                    
                    d_f =         [1  obj.t_f   obj.t_f^2   obj.t_f^3    obj.t_f^4    obj.t_f^5]; % 2.5
                    d_dot_f =     [0  1         2*obj.t_f   3*obj.t_f^2  4*obj.t_f^3  5*obj.t_f^4]; % 0
                    d_ddot_f =    [0  0         2           6*obj.t_f    12*obj.t_f^2  20*obj.t_f^3]; % 0
                    
                    A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];
                    
                    B = [y_ego; 0; 0; y_ego-obj.LaneWidth; 0; 0];
                    
                    X = linsolve(A,B);
                    
                    obj.a0 = X(1);
                    obj.a1 = X(2);
                    obj.a2 = X(3);
                    obj.a3 = X(4);
                    obj.a4 = X(5);
                    obj.a5 = X(6);
                    
                    tP = obj.t_i:0.2:obj.t_f;
                    tL = 0:0.2:obj.t_f-obj.t_i;
                    newWP_s = x_ego + vEgo*tL; % Longitudinal motion
                    newWP_d = obj.a0 + obj.a1*tP + obj.a2*tP.^2 + obj.a3*tP.^3 + obj.a4*tP.^4 + obj.a5*tP.^5; % Lateral motion
                    newWP_all =  [newWP_s' newWP_d'];
                end
                % Stay on lane
                if clock > obj.t_f
                    latSpeed = 0;
                % Change lane
                else
                    t = clock;
                    
                    latSpeed =  obj.a1  + 2*obj.a2*t +  3*obj.a3*t.^2  +  4*obj.a4*t.^3  +  5*obj.a5*t.^4;    
                end
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
