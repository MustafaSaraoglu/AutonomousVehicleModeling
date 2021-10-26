classdef LocalTrajectoryPlanner < CoordinateTransformations
    % LocalTrajectoryPlanner Superclass for generating necessary inputs for 
    % the lateral controllers.
    
    properties(Nontunable)
        durationToLeftLane % Time for lane changing
        durationToRightLane % Time for overtaking
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
        
        maneuverTrajectory % Planned trajectory for maneuver (x, y, lateral velociy)
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
        
        function checkForLaneChangingManeuver(obj, changeLaneCmd, s, d, velocity)
            % Check whether to start or stop a lane changing maneuver
            
            % Initialisation maneuver to left lane
            if changeLaneCmd == obj.laneChangeCmds('CmdStartToLeft')
                obj.initialiseManeuver(s, d, obj.LaneWidth, obj.maneuvers('ChangeToLeftLane'), obj.durationToLeftLane, velocity);
            % Initialisation maneuver to right lane
            elseif changeLaneCmd == obj.laneChangeCmds('CmdStartToRight')
                obj.initialiseManeuver(s, d, 0, obj.maneuvers('ChangeToRightLane'), obj.durationToRightLane, velocity);
            % Stop maneuver
            elseif changeLaneCmd == obj.laneChangeCmds('CmdStopLaneChange')
                obj.executeManeuver = obj.maneuvers('StayOnLane');
            end
        end
        
        function initialiseManeuver(obj, s_current, d_currnet, d_destination, maneuver, durationManeuver, velocity)
            % Initialise lane changing maneuver and calculate reference
            % trajectory
            
            obj.calculateLaneChangingTrajectory(s_current, d_currnet, d_destination, durationManeuver, velocity);
            obj.executeManeuver = maneuver; % Set maneuver to indicate lane change to the left or right
        end
        
        function calculateLaneChangingTrajectory(obj, s_current, d_currnet, d_destination, durationManeuver, velocity)
            % Calculate minimum jerk trajectory for lane changing maneuver
            
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
            
            % Calculate trajectory for complete maneuver
            t_discrete = 0:0.01:durationManeuver;
            
            s_trajectory = s_current + velocity*t_discrete;
            d_trajectory = obj.a0 + obj.a1*t_discrete + obj.a2*t_discrete.^2 + obj.a3*t_discrete.^3 + obj.a4*t_discrete.^4 + obj.a5*t_discrete.^5;
            dDot_trajectory = obj.a1 + 2*obj.a2*t_discrete + 3*obj.a3*t_discrete.^2 + 4*obj.a4*t_discrete.^3 + 5*obj.a5*t_discrete.^4;
%             positionCartesianTrajectory = obj.Frenet2Cartesian(0, [s_trajectory' d_trajectory'], obj.CurrentTrajectory);
            
            obj.maneuverTrajectory = [s_trajectory', d_trajectory',  dDot_trajectory'];
        end   
        
        function [s_ref, d_ref, dDot_ref] = getNextTrajectoryWaypoint(obj, s)
            % Get the next waypoint for trajectory according to current s
            
            IDs_passedWPs = s >= obj.maneuverTrajectory(:, 1);
            ID_nextWP = sum(IDs_passedWPs) + 1;
            
            if ID_nextWP > length(obj.maneuverTrajectory)
                s_ref = s + 0.01; % No more waypoints in list: waypoint ahead on the same lane
                ID_nextWP = length(obj.maneuverTrajectory);
            else
                s_ref = obj.maneuverTrajectory(ID_nextWP, 1);
            end

            d_ref = obj.maneuverTrajectory(ID_nextWP, 2);
            dDot_ref = obj.maneuverTrajectory(ID_nextWP, 3);
        end
    end
end

