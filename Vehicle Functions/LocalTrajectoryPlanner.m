classdef LocalTrajectoryPlanner < CoordinateTransformations
    % LocalTrajectoryPlanner Superclass for generating necessary inputs for 
    % the lateral controllers.
    
    properties
        
    end
    
    properties(Nontunable)
        durationToLeftLane = evalin('base', 'durationToLeftLane'); % Time for lane changing
        durationToRightLane = evalin('base', 'durationToRightLane'); % Time for overtaking
    end
    
    % Pre-computed constants
    properties(Access = protected)
        % Coefficents for lane changing trajectory
        a0
        a1
        a2
        a3
        a4
        a5
        
        maneuvers % Possible maneuvers
        
        % State if any maneuver should be executed
        %   0 = Stay on same lane
        %   1 = Change to left lane
        %  -1 = Change to right lane
        executeManeuver
        
        laneChangeCmds % Possible commands for lane changing
        
        t_start % Time to start maneuver
    end
    
    methods
        function obj = LocalTrajectoryPlanner(varargin)
            %WAYPOINTGENERATOR Construct an instance of this class
            %   Detailed explanation goes here
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.maneuvers = containers.Map({'StayOnLane', 'ChangeToLeftLane', 'ChangeToRightLane'}, [0, 1, -1]);
            obj.executeManeuver = obj.maneuvers('StayOnLane'); 
            
            obj.laneChangeCmds = ...
                containers.Map({'CmdFollow', 'CmdStartToLeft', 'CmdStartToRight', 'CmdStopLaneChange'}, [0, 1, -1, 2]);
        end
        
        function checkForLaneChangingManeuver(obj, changeLaneCmd, d, clock)
            % Check whether to start or stop a lane changing maneuver
            
            % Initialisation maneuver to left lane
            if changeLaneCmd == obj.laneChangeCmds('CmdStartToLeft')
                obj.initialiseManeuver(d, obj.LaneWidth, obj.maneuvers('ChangeToLeftLane'), obj.durationToLeftLane, clock);
            % Initialisation maneuver to right lane
            elseif changeLaneCmd == obj.laneChangeCmds('CmdStartToRight')
                obj.initialiseManeuver(d, 0, obj.maneuvers('ChangeToRightLane'), obj.durationToRightLane, clock);
            % Stop maneuver
            elseif changeLaneCmd == obj.laneChangeCmds('CmdStopLaneChange')
                obj.executeManeuver = obj.maneuvers('StayOnLane');
            end
        end
        
        function initialiseManeuver(obj, d_currnet, d_destination, maneuver, durationManeuver, clock)
            % Initialise lane changing maneuver and calculate reference
            % trajectory
            
            obj.calculateLaneChangingTrajectoryCoefficients(d_currnet, d_destination, durationManeuver);
            obj.executeManeuver = maneuver; % Set maneuver to indicate lane change to the left or right
            obj.t_start = clock; % Store global time when starting the maneuver
        end
        
        function calculateLaneChangingTrajectoryCoefficients(obj, d_currnet, d_destination, durationManeuver)
            % Calculate coefficients for minimum jerk trajectory
            
            % Initial conditions
            t_i = 0; % Start at 0 (relative time frame)
                    
            d_i =         [1  t_i   t_i^2   t_i^3    t_i^4      t_i^5]; % d_initial = d_current before lane change
            d_dot_i =     [0  1     2*t_i   3*t_i^2  4*t_i^3    5*t_i^4]; %  0
            d_ddot_i =    [0  0     2       6*t_i    12*t_i^2   20*t_i^3]; %  0

            % Final conditions
            t_f = durationManeuver; % deltaT: time to finish maneuver

            d_f =         [1  t_f   t_f^2   t_f^3    t_f^4      t_f^5]; % d_destination
            d_dot_f =     [0  1     2*t_f   3*t_f^2  4*t_f^3    5*t_f^4]; % 0
            d_ddot_f =    [0  0     2       6*t_f    12*t_f^2   20*t_f^3]; % 0

            A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];

            B = [d_currnet; 0; 0; d_destination; 0; 0];

            X = linsolve(A,B);

            obj.a0 = X(1);
            obj.a1 = X(2); 
            obj.a2 = X(3);
            obj.a3 = X(4);
            obj.a4 = X(5);
            obj.a5 = X(6);
        end   
        
        function [d_ref, dDot_ref] = getLateralReference(obj, t)
            % Get the reference lateral position and speed for maneuver
            
            % Calculate d and dDot according to minimum jerk trajectory
            d_ref = obj.a0 + obj.a1*t + obj.a2*t.^2 + obj.a3*t.^3 + obj.a4*t.^4 + obj.a5*t.^5;
            dDot_ref = obj.a1 + 2*obj.a2*t + 3*obj.a3*t.^2 + 4*obj.a4*t.^3 + 5*obj.a5*t.^4; 

            % Check whether t exceeds duration planned for the maneuver,
            % because calculated minimum jerk trajectory is only valid for 
            % t in [0, durationManeuver]
            switch obj.executeManeuver
                case obj.maneuvers('ChangeToLeftLane')
                    if t >= obj.durationToLeftLane
                        d_ref = obj.LaneWidth;
                        dDot_ref = 0;
                    end
                case obj.maneuvers('ChangeToRightLane')
                    if t >= obj.durationToRightLane
                        d_ref = 0;
                        dDot_ref = 0;
                    end
            end
        end
    end
end
