classdef LocalTrajectoryPlanner < matlab.System
% LocalTrajectoryPlanner Superclass for generating necessary inputs for 
% the lateral controllers.
    
    properties(Nontunable)
        maximumAcceleration % Maximum longitudinal acceleration [m/s^2]
        
        vEgo_ref % Reference speed for ego vehicle [m/s]

        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        wheelBase % Wheel base vehicle [m]
        steerAngle_max % Maximum steering angle [rad]
         
        timeHorizon % Time horizon for trajectory genereation [s]
        Ts % Sampling time for trajectory generation [s]
    end
    
    properties(Access = protected)        
        d_destination % Reference lateral destination (right or left lane)
        
        laneChangingTrajectoryFrenet % Planned trajectory for lane changing in Frenet coordinates 
                                     % [s, d] 
        laneChangingTrajectoryCartesian % Planned trajectory for lane changing in Cartesian 
                                        % coordinates [x, y, orientation]
        
        TrajectoryGenerator % Generate trajectories
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
            obj.d_destination = 0; % Start on right lane
            
            curvature_max = tan(obj.steerAngle_max)/obj.wheelBase; % Maximum allowed curvature
            a_lateral_max = 30; % Maximum allowed lateral acceleration
            obj.TrajectoryGenerator = TrajectoryGeneration(obj.Ts, obj.timeHorizon, ...
                                                           obj.RoadTrajectory, curvature_max, ...
                                                           a_lateral_max);
        end
        
        function planReferenceTrajectory(obj, changeLaneCmd, s, d, v)
        % Plan the reference trajectory
        % Longitudinal: The current lane is the reference
        % Change Lane: Calculate Frenet lane changing trajectory points
            
            if changeLaneCmd 
                durationManeuver = changeLaneCmd;
                
                [d_oppositeLane, destinationLane] = obj.getOppositeLane(d, obj.LaneWidth);
                obj.d_destination = d_oppositeLane;
               
                [trajectoryFrenet, trajectoryCartesian] = ...
                    obj.TrajectoryGenerator.calculateLaneChangingTrajectory...
                        (s, d, 0, 0, d_oppositeLane, v, obj.vEgo_ref, obj.maximumAcceleration, ...
                         durationManeuver);

                obj.laneChangingTrajectoryFrenet = trajectoryFrenet;
                obj.laneChangingTrajectoryCartesian = trajectoryCartesian;

                x_trajectory = obj.laneChangingTrajectoryCartesian.x;
                y_trajectory = obj.laneChangingTrajectoryCartesian.y;

                plot(x_trajectory, y_trajectory, 'Color', 'green');

                t = get_param('ManeuverPlanning', 'SimulationTime');

                fprintf('@t=%fs: Start trajectory to %s, duration=%fs.\n', t, destinationLane, ...
                        durationManeuver);
            end
        end
        
        function [s_ref, d_ref] = getNextFrenetTrajectoryWaypoints(obj, s, v, numberWPs)
        % Get the next waypoint(s) for current trajectory according to current s in Frenet 
        % coordinates
            
            if ~isempty(obj.laneChangingTrajectoryFrenet) 
                ID_nextWP = sum(s >= obj.laneChangingTrajectoryFrenet.s) + 1;
                
                if ID_nextWP > obj.laneChangingTrajectoryFrenet.length% No more lane changing 
                                                                          % points left
                    % Reset lane changing trajectory, if passed all lane changing points in 
                    % trajectory
                    obj.laneChangingTrajectoryFrenet = [];
                    
                    % Get Waypoints ahead staying on the same lane
                    [s_ref, d_ref] = obj.getNextRoadTrajectoryWaypoints(s, v, numberWPs);
                    return
                end
                
                % Add points from lane changing trajectory
                numberResidualLaneChangingPoints = ...
                    obj.laneChangingTrajectoryFrenet.length - (ID_nextWP-1);
                numberPointsFromLaneChanging = min(numberResidualLaneChangingPoints, numberWPs);
                s_ref = ...
                    obj.laneChangingTrajectoryFrenet.s(ID_nextWP:ID_nextWP+(numberPointsFromLaneChanging-1));
                d_ref = ...
                    obj.laneChangingTrajectoryFrenet.d(ID_nextWP:ID_nextWP+(numberPointsFromLaneChanging-1));
                
                % Add points for straight movement staying on the same lane if no more 
                % lane changing points left
                numberPointsFromRoadTrajectory = numberWPs - numberPointsFromLaneChanging;
                if numberPointsFromRoadTrajectory > 0
                    [s_add, d_add] = obj.getNextRoadTrajectoryWaypoints(s_ref(end), v, ...
                                                                        numberPointsFromRoadTrajectory);
                    s_ref = [s_ref; s_add];
                    d_ref = [d_ref; d_add];
                end
            else
                [s_ref, d_ref] = obj.getNextRoadTrajectoryWaypoints(s, v, numberWPs);
            end
        end
        
        function [s_ref, d_ref] = getNextRoadTrajectoryWaypoints(obj, s, v, numberPoints)
        % Get next waypoints for staying on the same lane and following the road trajectory
            
            % Linear spacing according to current velocity
            s_ref = s + linspace(v*obj.Ts, numberPoints*v*obj.Ts, numberPoints)';
            d_ref = obj.d_destination*ones(numberPoints, 1);
        end
    end
    
    methods(Static)
        function [d_oppositeLane, destinationLane] = getOppositeLane(d_currentLane, laneWidth)
        % Get lane opposite to current lane
            
            d_oppositeLane = 0;
            destinationLane = 'right lane';
        
            if abs(d_currentLane) < 0.1 % Some threshold value in case the nominal d value is very small
                d_oppositeLane = laneWidth;
                destinationLane = 'left lane';
            end
        end
    end
end

