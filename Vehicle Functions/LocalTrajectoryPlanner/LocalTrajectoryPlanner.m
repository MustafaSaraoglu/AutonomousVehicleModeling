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
        
        laneChangingTrajectoryFrenet % Planned trajectory for lane changing in Frenet coordinates [s, d] 
        laneChangingTrajectoryCartesian % Planned trajectory for lane changing in Cartesian coordinates [x, y, orientation]
        
        curvature_max % Maximum allowed curvature
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
            
            obj.curvature_max = tan(obj.steerAngle_max)/obj.wheelBase;
        end
        
        function planReferenceTrajectory(obj, changeLaneCmd, s, d, v)
        % Plan the reference trajectory
        % Longitudinal: The current lane is the reference
        % Change Lane: Calculate Frenet lane changing trajectory points
            
            if changeLaneCmd 
                durationManeuver = changeLaneCmd;
                
                [d_oppositeLane, destinationLane] = obj.getOppositeLane(d, obj.LaneWidth);
                obj.d_destination = d_oppositeLane;
               
                [trajectoryFrenet, trajectoryCartesian, ~, ~, ~] = ...
                    obj.calculateLaneChangingTrajectory(s, d, 0, 0, d_oppositeLane, v, ...
                                                        obj.vEgo_ref, obj.maximumAcceleration, ...
                                                        durationManeuver, obj.curvature_max, ...
                                                        obj.RoadTrajectory, obj.timeHorizon, obj.Ts);

                obj.laneChangingTrajectoryFrenet = trajectoryFrenet;
                obj.laneChangingTrajectoryCartesian = trajectoryCartesian;

                x_trajectory = obj.laneChangingTrajectoryCartesian(:, 1);
                y_trajectory = obj.laneChangingTrajectoryCartesian(:, 2);

                plot(x_trajectory, y_trajectory, 'Color', 'green');

                t = get_param('VehicleFollowing', 'SimulationTime');

                fprintf('@t=%fs: Start trajectory to %s, duration=%fs.\n', t, destinationLane, durationManeuver);
            end
        end
        
        function [s_ref, d_ref] = getNextFrenetTrajectoryWaypoints(obj, s, v, numberWPs)
        % Get the next waypoint(s) for current trajectory according to current s in Frenet coordinates
            
            if ~isempty(obj.laneChangingTrajectoryFrenet)
                s_trajectory =  obj.laneChangingTrajectoryFrenet(:, 1); 
                ID_nextWP = sum(s >= s_trajectory) + 1;
                
                if ID_nextWP > size(obj.laneChangingTrajectoryFrenet, 1) % No more lane changing points left
                    % Reset lane changing trajectory, if passed all lane changing points in trajectory
                    obj.laneChangingTrajectoryFrenet = [];
                    
                    % Get Waypoints ahead staying on the same lane
                    [s_ref, d_ref] = obj.getNextRoadTrajectoryWaypoints(s, v, numberWPs);
                    return
                end
                
                % Add points from lane changing trajectory
                numberResidualLaneChangingPoints = size(obj.laneChangingTrajectoryFrenet, 1) - (ID_nextWP-1);
                numberPointsFromLaneChanging = min(numberResidualLaneChangingPoints, numberWPs);
                s_ref = obj.laneChangingTrajectoryFrenet(ID_nextWP:ID_nextWP+(numberPointsFromLaneChanging-1), 1);
                d_ref = obj.laneChangingTrajectoryFrenet(ID_nextWP:ID_nextWP+(numberPointsFromLaneChanging-1), 2);
                
                % Add points for straight movement staying on the same lane if no more lane changing points left
                numberPointsFromRoadTrajectory = numberWPs - numberPointsFromLaneChanging;
                if numberPointsFromRoadTrajectory > 0
                    [s_add, d_add] = obj.getNextRoadTrajectoryWaypoints(s_ref(end), v, numberPointsFromRoadTrajectory);
                    s_ref = [s_ref; s_add];
                    d_ref = [d_ref; d_add];
                end
            else
                [s_ref, d_ref] = obj.getNextRoadTrajectoryWaypoints(s, v, numberWPs);
            end
        end
        
        function [s_ref, d_ref] = getNextRoadTrajectoryWaypoints(obj, s, v, numberPoints)
        % Get next waypoints for staying on the same lane and following the road trajectory
            
            s_ref = s + linspace(v*obj.Ts, numberPoints*v*obj.Ts, numberPoints)'; % Linear spacing according to current velocity
            d_ref = obj.d_destination*ones(numberPoints, 1);
        end
    end
    
    methods(Static)
        function [trajectoryFrenet, trajectoryCartesian, v_trajectory, cost, isFeasibleTrajectory] = ...
                    calculateLaneChangingTrajectory(s_current, d_current, d_dot_current, ...
                                                    d_ddot_current, d_destination, v_current, ...
                                                    v_ref, a_ref, durationManeuver, curvature_max, ...
                                                    roadTrajectory, Th, Ts)
        % Calculate minimum jerk trajectory for lane changing maneuver
            
            % Initial conditions
            t_i = 0; % Start at 0 (relative time frame)
                    
            d_i =         [1  t_i   t_i^2   t_i^3    t_i^4      t_i^5]; % d_initial = d_current 
            d_dot_i =     [0  1     2*t_i   3*t_i^2  4*t_i^3    5*t_i^4]; %  0
            d_ddot_i =    [0  0     2       6*t_i    12*t_i^2   20*t_i^3]; %  0

            % Final conditions
            t_f = durationManeuver; % Time to finish maneuver

            d_f =         [1  t_f   t_f^2   t_f^3    t_f^4      t_f^5]; % d_destination
            d_dot_f =     [0  1     2*t_f   3*t_f^2  4*t_f^3    5*t_f^4]; % 0
            d_ddot_f =    [0  0     2       6*t_f    12*t_f^2   20*t_f^3]; % 0

            A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];

            B = [d_current; d_dot_current; d_ddot_current; d_destination; 0; 0];

            X = linsolve(A,B);

            a0 = X(1);
            a1 = X(2); 
            a2 = X(3);
            a3 = X(4);
            a4 = X(5);
            a5 = X(6);
            
            % Calculate trajectory for complete maneuver
            t_maneuver = 0:Ts:durationManeuver; 
            
            d_trajectory = a0 + a1*t_maneuver + a2*t_maneuver.^2 + a3*t_maneuver.^3 + a4*t_maneuver.^4 + a5*t_maneuver.^5;
            d_dot_trajectory = a1 + 2*a2*t_maneuver + 3*a3*t_maneuver.^2 + 4*a4*t_maneuver.^3 + 5*a5*t_maneuver.^4;
            d_ddot_trajectory = 2*a2 + 6*a3*t_maneuver + 12*a4*t_maneuver.^2 + 20*a5*t_maneuver.^3;
            d_dddot_trajectory = 6*a3 + 24*a4*t_maneuver + 60*a5*t_maneuver.^2;
            
            % Calculate velocity profile according to reference acceleration 
            [~, v_trajectory] = LocalTrajectoryPlanner.calculateLongitudinalTrajectory(s_current, ...
                v_current, v_ref, a_ref, length(t_maneuver), Ts);
            
            
            if any(v_trajectory.^2 < d_dot_trajectory.^2)
                % Trajectory not feasible
                isFeasibleTrajectory = false;
                
                trajectoryFrenet = [s_current, d_current];
                
                [position_current, roadOrientation_current] = Frenet2Cartesian(s_current, d_current, roadTrajectory);
                trajectoryCartesian = [position_current(1), position_current(2), roadOrientation_current];
                
                cost = Inf;
                return
            end
            
            % Calculate velocity in s-direction 
            s_dot_trajectory = sqrt(v_trajectory.^2 - d_dot_trajectory.^2); 
            
            % Calculate uture prediction for s-trajectory along the road
            s_trajectory = zeros(1, length(t_maneuver)); 
            s = s_current;
            for k = 1:length(t_maneuver) % Numerical integration
                s_trajectory(k) = s;
                
                % Sum up s-values knowing the velocity in s-direction and the discrete sample time
                s = s + s_dot_trajectory(k)*Ts; 
            end
            
            % If the maneuver duration is shorter than the specified time horizon, 
            % a longitudinal trajectory following the destination lane needs to be added
            if durationManeuver < Th
                t_longitudinal = durationManeuver+Ts:Ts:Th; 
                
                % Calculate initial displacement and velocity for t_longitudinal_0 = durationManeuver+obj.Ts
                [s_0, v_0] = ReachabilityAnalysis.predictLongitudinalFutureState(s_trajectory(end), ...
                    v_trajectory(end), v_ref, a_ref, 0, Ts);
                
                [s_trajectory_straight, v_trajectory_straight] = ...
                    LocalTrajectoryPlanner.calculateLongitudinalTrajectory(s_0, v_0, v_ref, ...
                                                                a_ref, length(t_longitudinal), Ts);
                
                % Update whole lane changing trajectory by adding a longitudinal trajectory 
                % to the calculated lane changing trajectory
                s_trajectory = [s_trajectory, s_trajectory_straight];
                v_trajectory = [v_trajectory, v_trajectory_straight];
                s_dot_trajectory = [s_dot_trajectory, v_trajectory_straight]; % For straight trajectory s_dot = v
                
                d_trajectory = [d_trajectory, d_destination*ones(1, length(t_longitudinal))];
                d_dot_trajectory = [d_dot_trajectory, zeros(1, length(t_longitudinal))];
                d_ddot_trajectory = [d_ddot_trajectory, zeros(1, length(t_longitudinal))];
                d_dddot_trajectory = [d_dddot_trajectory, zeros(1, length(t_longitudinal))];
            end
            
            [laneChangingPositionCartesian, roadOrientation] = Frenet2Cartesian(s_trajectory', d_trajectory', roadTrajectory);
            % The road orientation needs to be added (See documentation: "Calculate Trajectory for Lane Changing")
            orientation = atan2(d_dot_trajectory, s_dot_trajectory)' + roadOrientation;
            
            trajectoryFrenet = [s_trajectory', d_trajectory'];
            trajectoryCartesian = [laneChangingPositionCartesian, orientation];
            
            isFeasibleTrajectory = LocalTrajectoryPlanner.isFeasibleTrajectory(trajectoryCartesian, ...
                                    d_ddot_trajectory, curvature_max, 30);
            
            % Cost function according to jerk
            cost = 0.5*sum(d_dddot_trajectory.^2);
        end
        
        function [s_trajectory, v_trajectory] = calculateLongitudinalTrajectory(s_0, v_0, v_max, acceleration, trajectoryLength, Ts)
        % Calculate longitudinal trajectory according to the longitudinal reachability analysis using each time step
            
            v_trajectory = zeros(1, trajectoryLength);
            s_trajectory = zeros(1, trajectoryLength);
            
            % Future prediction for displacement s, and velocityv
            s = s_0; 
            v = v_0;
            for k = 1:trajectoryLength 
                v_trajectory(k) = v;
                s_trajectory(k) = s;
                [s, v] = ReachabilityAnalysis.predictLongitudinalFutureState(s, v, v_max, ...
                    acceleration, 0, Ts); % Prediction just for next time step
            end
        end
        
        function isFeasible = isFeasibleTrajectory(trajectory, a_lateral, curvature_max, a_lateral_max)
        % Check if trajectory is feasible accoording to lateral acceleration and trajectory curvature
            
            x = trajectory(:, 1);
            y = trajectory(:, 2);
            orientation = trajectory(:, 3);
            
            delta_orientation = diff(orientation);
            delta_position = sqrt((diff(x)).^2 + (diff(y)).^2);
            
            curvature = delta_orientation./delta_position;
            
            % Alternative: Limit by using centrifugal acceleration
            % a_lateral = v(1:end-1)'.^2.*curvature; 
            
            isFeasibleCurvature = all(abs(curvature) < curvature_max);
            isFeasibleCentrifugalAcceleration = all(abs(a_lateral) < a_lateral_max); % Placeholder for a_lateral_max
            
            isFeasible = isFeasibleCurvature && isFeasibleCentrifugalAcceleration;
        end
        
        function [d_oppositeLane, destinationLane] = getOppositeLane(d_currentLane, laneWidth)
        % Get lane opposite to current lane
            
            d_oppositeLane = 0;
            destinationLane = 'right lane';
        
            if d_currentLane == 0
                d_oppositeLane = laneWidth;
                destinationLane = 'left lane';
            end
        end
    end
end

