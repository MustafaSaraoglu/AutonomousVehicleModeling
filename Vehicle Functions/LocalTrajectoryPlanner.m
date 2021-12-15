classdef LocalTrajectoryPlanner < ReachabilityAnalysis
% LocalTrajectoryPlanner Superclass for generating necessary inputs for 
% the lateral controllers.
    
    properties(Nontunable)
        vEgo_ref % Reference speed for ego vehicle [m/s]
        
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        
        durationToLeftLane % Time for lane changing [s]
        durationToRightLane % Time for overtaking [s]
        partsTimeHorizon % Divide time horizon into partsTimeHorizon equal parts
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
        
        laneChangeCmds % Possible commands for lane changing
        
        d_destination % Reference lateral destination (right or left lane)
        
        trajectoryReferenceLength % Number of points for trajectory generation
        
        fractionTimeHorizon % Fraction of time horizon when divided into partsTimeHorizon equal parts
        counter % Counter to stop at correct simulation time
        predictedTrajectory % Store future trajectory predictions for replanning
        
        laneChangingTrajectoryFrenet % Planned trajectory for lane changing in Frenet coordinates [s, d, s_curve, time] (s_curve = coordinate along the lane changing curve)
        laneChangingTrajectoryCartesian % Planned trajectory for lane changing in Cartesian coordinates [x, y, orientation, time]
        futurePosition % Preedicted future position according to time horizon

        timeStartLaneChange % Time when starting lane changing maneuver
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
            setupImpl@ReachabilityAnalysis(obj) 

            obj.laneChangeCmds = ...
                containers.Map({'CmdIdle', 'CmdStartToLeftLane', 'CmdStartToRightLane'}, [0, 1, -1]);
            
            obj.d_destination = 0; % Start on right lane
            
            % +1 because planned trajectory contains current waypoint at current time
            obj.trajectoryReferenceLength = obj.timeHorizon/obj.Ts + 1; 
            
            obj.fractionTimeHorizon = obj.timeHorizon/obj.partsTimeHorizon;
            obj.counter = 0;
            obj.predictedTrajectory = [];
        end
        
        function planTrajectory(obj, changeLaneCmd, replan, s, d, a, v)
        % Plan trajectory for the next obj.timeHorizon seconds in Frenet and Cartesian coordinates

            if changeLaneCmd 
                obj.timeStartLaneChange = get_param('VehicleFollowing', 'SimulationTime');
                obj.calculateLaneChangingManeuver(changeLaneCmd, s, d, 0, 0, v); 
            end
            
            if replan
                % TODO: Add
            end
            
            % TODO: Do at every simulation time step?
            obj.predictFuturePosition(s, v, a);
        end
        
        function calculateLaneChangingManeuver(obj, changeLaneCmd, s, d, d_dot, d_ddot, v)
        % Calculate the lane changing maneuver either to the left or right lane
            
            if changeLaneCmd == obj.laneChangeCmds('CmdStartToLeftLane')
                obj.d_destination = obj.LaneWidth;
                durationManeuver = obj.durationToLeftLane;
            elseif changeLaneCmd == obj.laneChangeCmds('CmdStartToRightLane')
                obj.d_destination = 0;
                durationManeuver = obj.durationToRightLane;
            end
            obj.calculateLaneChangingTrajectory(s, d, d_dot, d_ddot, obj.d_destination, durationManeuver, v);
        end
        
        function calculateLaneChangingTrajectory(obj, s_current, d_currnet, d_dot_current, d_ddot_current, d_destination, durationManeuver, v_current)
        % Calculate minimum jerk trajectory for lane changing maneuver
            
            % Initial conditions
            t_i = 0; % Start at 0 (relative time frame)
                    
            d_i =         [1  t_i   t_i^2   t_i^3    t_i^4      t_i^5]; % d_initial = d_current 
            d_dot_i =     [0  1     2*t_i   3*t_i^2  4*t_i^3    5*t_i^4]; %  0
            d_ddot_i =    [0  0     2       6*t_i    12*t_i^2   20*t_i^3]; %  0

            % Final conditions
            t_f = durationManeuver; % time to finish maneuver

            d_f =         [1  t_f   t_f^2   t_f^3    t_f^4      t_f^5]; % d_destination
            d_dot_f =     [0  1     2*t_f   3*t_f^2  4*t_f^3    5*t_f^4]; % 0
            d_ddot_f =    [0  0     2       6*t_f    12*t_f^2   20*t_f^3]; % 0

            A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];

            B = [d_currnet; d_dot_current; d_ddot_current; d_destination; 0; 0];

            X = linsolve(A,B);

            obj.a0 = X(1);
            obj.a1 = X(2); 
            obj.a2 = X(3);
            obj.a3 = X(4);
            obj.a4 = X(5);
            obj.a5 = X(6);
            
            % Calculate trajectory for whole maneuver
            t_discrete = 0:obj.Ts:durationManeuver; 
            
            d_trajectory = obj.a0 + obj.a1*t_discrete + obj.a2*t_discrete.^2 + obj.a3*t_discrete.^3 + obj.a4*t_discrete.^4 + obj.a5*t_discrete.^5;
            d_dot_trajectory = obj.a1 + 2*obj.a2*t_discrete + 3*obj.a3*t_discrete.^2 + 4*obj.a4*t_discrete.^3 + 5*obj.a5*t_discrete.^4;

            [s_trajectory, s_curve_trajectory, s_dot_trajectory] = obj.calculate_s_trajectory(s_current, v_current, obj.vEgo_ref, obj.maximumAcceleration, d_dot_trajectory, durationManeuver); % Free Drive
            [s_trajectory_minAcc, ~, ~] = obj.calculate_s_trajectory(s_current, v_current, obj.maximumVelocity, obj.minimumAcceleration, d_dot_trajectory, durationManeuver); 
            [s_trajectory_maxAcc, ~, ~] = obj.calculate_s_trajectory(s_current, v_current, obj.maximumVelocity, obj.maximumAcceleration, d_dot_trajectory, durationManeuver); 
            
            if durationManeuver < obj.timeHorizon
                d_trajectory = [d_trajectory, obj.d_destination*ones(1, length(s_trajectory)-length(t_discrete))];
                d_dot_trajectory = [d_dot_trajectory, zeros(1, length(s_trajectory)-length(t_discrete))];
                t_discrete = [t_discrete, durationManeuver+obj.Ts:obj.Ts:obj.timeHorizon];
            end
            
            [laneChangingPositionCartesian, roadOrientation] = Frenet2Cartesian(s_trajectory', d_trajectory', obj.RoadTrajectory);
            orientation = atan2(d_dot_trajectory, s_dot_trajectory)' + roadOrientation;
            
            time = get_param('VehicleFollowing', 'SimulationTime') + t_discrete;
            
            obj.laneChangingTrajectoryFrenet = [s_trajectory', d_trajectory', s_curve_trajectory', time'];
            obj.laneChangingTrajectoryCartesian = [laneChangingPositionCartesian, orientation, time'];
            
            % Only to plot trajectory for possible minimum and maximum acceleration, might be removed
            [laneChangingPointsCartesian_minAcc, ~] = Frenet2Cartesian(s_trajectory_minAcc', d_trajectory', obj.RoadTrajectory);
            [laneChangingPointsCartesian_maxAcc, ~] = Frenet2Cartesian(s_trajectory_maxAcc', d_trajectory', obj.RoadTrajectory);
            plot(obj.laneChangingTrajectoryCartesian(:, 1), obj.laneChangingTrajectoryCartesian(:,2), 'Color', 'green');
            plot(laneChangingPointsCartesian_minAcc(:, 1), laneChangingPointsCartesian_minAcc(:,2), '--', 'Color', 'green');
            plot(laneChangingPointsCartesian_maxAcc(:, 1), laneChangingPointsCartesian_maxAcc(:,2), '--', 'Color', 'green');
        end 
        
        function [s_trajectory, s_curve_trajectory, s_dot_trajectory] = calculate_s_trajectory(obj, s_0, v_0, v_max, acceleration, d_dot_trajectory, durationManeuver)
        % Calculate s, s_curve_trajectory and s_dot trajectory according to kinematic bicycle speed profile

            % s_curve coordinate going along the lane changing curve
            [s_curve_trajectory, v_trajectory] = obj.calculateLongitudinalTrajectory(s_0, v_0, v_max, acceleration, length(d_dot_trajectory));
            
            s_dot_trajectory = sqrt(v_trajectory.^2 - d_dot_trajectory.^2); 
            
            % Future prediction s along the road
            s_trajectory = zeros(1, length(d_dot_trajectory)); % TODO: Check: s along the road, what if acceleration?
            s = s_0;
            for k = 1:length(d_dot_trajectory) % Numerical integration
                s_trajectory(k) = s;
                s = s + s_dot_trajectory(k)*obj.Ts;
            end
            
            if durationManeuver < obj.timeHorizon
                t_discrete = durationManeuver+obj.Ts:obj.Ts:obj.timeHorizon; 
                
                % Predict one step for durationManeuver+obj.Ts
                [s_0, v_0] = obj.predictLongitudinalFutureState(s_trajectory(end), v_trajectory(end), v_max, acceleration, 0);
                [s_trajectory_straight, v_trajectory_straight] = obj.calculateLongitudinalTrajectory(s_0, v_0, v_max, acceleration, length(t_discrete));
                
                s_trajectory = [s_trajectory, s_trajectory_straight];
                s_curve_trajectory = [s_curve_trajectory, s_trajectory_straight]; % For straight trajectory s_curve = s
                s_dot_trajectory = [s_dot_trajectory, v_trajectory_straight]; % For straight trajectory s_dot = v
            end
        end
        
        function [s_trajectory, v_trajectory] = calculateLongitudinalTrajectory(obj, s_0, v_0, v_max, acceleration, trajectoryLength)
        % Calculate longitudinal trajectory according to the longitudinal
        % reachability analysis using each time step
            
            v_trajectory = zeros(1, trajectoryLength);
            s_trajectory = zeros(1, trajectoryLength);
            
            % Future prediction s, v
            s = s_0; 
            v = v_0;
            for k = 1:trajectoryLength % Numerical integration
                v_trajectory(k) = v;
                s_trajectory(k) = s;
                [s, v] = obj.predictLongitudinalFutureState(s, v, v_max, acceleration, 0); % Prediction just for next time step
            end
        end
        
        function predictFuturePosition(obj, s_current, v_current, a_current)
        % Predict future position in a specified time horizon
            
            % Future prediction until time horizon for constant acceleration
            % TODO: Limit by reference velocity or maximum allowed velocity?
            [s_future, ~] = obj.predictLongitudinalFutureState(s_current, v_current, obj.vEgo_ref, a_current, obj.k_timeHorizon); 
        
            if ~isempty(obj.laneChangingTrajectoryFrenet) && ~isempty(obj.laneChangingTrajectoryCartesian) % For lane changing
                distance_future = s_future - s_current;
                
                % Get future s_curve value
                s_trajectory = obj.laneChangingTrajectoryFrenet(:, 1);
                ID_currentWP = sum(s_current >= s_trajectory);
                
                s_curve_future = obj.laneChangingTrajectoryFrenet(ID_currentWP, 3) + distance_future;
                
                % Get future s and d value from lane changing trajectory using future s_curve value 
                s_curve_trajectory = obj.laneChangingTrajectoryFrenet(:, 3);
                ID_futureWP = sum(s_curve_future >= s_curve_trajectory);
                
                if ID_futureWP >= size(obj.laneChangingTrajectoryFrenet, 1)
                    % Simplification: Predict position on destination lane not considering lane changing trajectory anymore
                    d_future = obj.d_destination; 
                else
                    s_future = obj.laneChangingTrajectoryFrenet(ID_futureWP, 1);
                    d_future = obj.laneChangingTrajectoryFrenet(ID_futureWP, 2);
                end
            else % For following the road
                d_future = obj.d_destination;
            end
            
            futureTime = get_param('VehicleFollowing', 'SimulationTime') + obj.timeHorizon;
            [futurePos, ~] = Frenet2Cartesian(s_future, d_future, obj.RoadTrajectory);
            obj.futurePosition = [futurePos, futureTime]; % [x, y, time]
        end
        
        function [s_ref, d_ref] = getNextFrenetTrajectoryWaypoints(obj, s, v, numberWPs)
        % Get the next waypoint(s) for current trajectory according to current s in Frenet coordinates
            
            if ~isempty(obj.laneChangingTrajectoryFrenet)
                s_trajectory =  obj.laneChangingTrajectoryFrenet(:, 1); 
                ID_nextWP = sum(s >= s_trajectory) + 1;
                
                if ID_nextWP > size(obj.laneChangingTrajectoryFrenet, 1) % No more lane changing points left
                    obj.laneChangingTrajectoryFrenet = [];
                    [s_ref, d_ref] = obj.getNextRoadTrajectoryWaypoints(s, v, numberWPs);
                    return
                end
                
                % Add points from lane changing trajectory
                numberResidualLaneChangingPoints = size(obj.laneChangingTrajectoryFrenet, 1) - (ID_nextWP-1);
                numberPointsFromLaneChanging = min(numberResidualLaneChangingPoints, numberWPs);
                s_ref = obj.laneChangingTrajectoryFrenet(ID_nextWP:ID_nextWP+(numberPointsFromLaneChanging-1), 1);
                d_ref = obj.laneChangingTrajectoryFrenet(ID_nextWP:ID_nextWP+(numberPointsFromLaneChanging-1), 2);
                
                % Add points from road trajectory if necessary
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
        
        function replan = calculateTrajectoryError(obj, s, d)
        % Track error between predicted trajectory and actual trajectory every fractionTimeHorizon seconds
        % If the error exceeds threshold, replan maneuver
           
            replan = false;
            if get_param('VehicleFollowing', 'SimulationTime') > obj.counter*obj.fractionTimeHorizon
                if ~isempty(obj.predictedTrajectory)
                    error_s_d = obj.predictedTrajectory(1:2) - [s, d];
                    % TODO: Find threshold values that make sense and calculate new acceleration command
                    replan = (abs(error_s_d(1)) > 1) || (abs(error_s_d(2)) > 0.1);
                end 

%                 obj.setTrajectoryPrediction(); % TODO: Check again later, only working for lane changing trajectory yet
                obj.counter = obj.counter + 1;
            end
        end
        
        function setTrajectoryPrediction(obj)
        % Set prediction for the future position according to the planned trajectory in the next fractionTimeHorizon seconds
            
            timeToCheck = (obj.counter+1)*obj.fractionTimeHorizon;
            trajectoryTime = obj.laneChangingTrajectoryFrenet(:, end);
            
            ID_nextPrediction = sum(timeToCheck >= trajectoryTime); % ID for next predicted time
            obj.predictedTrajectory = obj.laneChangingTrajectoryFrenet(ID_nextPrediction, :); 
        end
    end
end

