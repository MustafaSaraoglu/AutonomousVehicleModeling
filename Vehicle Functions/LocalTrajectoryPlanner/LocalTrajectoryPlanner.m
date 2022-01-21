classdef LocalTrajectoryPlanner < ReachabilityAnalysis
% LocalTrajectoryPlanner Superclass for generating necessary inputs for 
% the lateral controllers.
    
    properties(Nontunable)
        vEgo_ref % Reference speed for ego vehicle [m/s]
        
        vOtherVehicles_ref % Reference speed for other vehicles [m/s]
        
        v_max % Maximum allowed velocity
        
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        
        spaceDiscretisation % Space Discretisation
    end
    
    properties(Access = protected)
        % Coefficents for lane changing trajectory
        a0
        a1
        a2
        a3
        a4
        a5
        
        laneChangeCmds % Possible commands for lane changing
        plannerModes % Possible planner modes
        
        d_destination % Reference lateral destination (right or left lane)
        
        trajectoryReferenceLength % Number of points for trajectory generation according to specified time horizon
        
        laneChangingTrajectoryFrenet % Planned trajectory for lane changing in Frenet coordinates [s, d] 
        laneChangingTrajectoryCartesian % Planned trajectory for lane changing in Cartesian coordinates [x, y, orientation]
        
        curvature_max % Maximum allowed curvature
        
        a_lateral_max % Maximum allowed lateral acceleration
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
            
            obj.plannerModes = ...
                containers.Map([1, 2], {'MANUAL', 'FORMAL'});
            
            obj.d_destination = 0; % Start on right lane
            
            % +1 because planned trajectory also contains current waypoint at current time
            obj.trajectoryReferenceLength = obj.timeHorizon/obj.Ts + 1; 
            
            obj.curvature_max = tan(obj.steerAngle_max)/obj.wheelBase;
            
            obj.a_lateral_max = 30; % Placeholder
        end
        
        function planReferenceTrajectory(obj, changeLaneCmd, plannerMode, s, d, v, poseOtherVehicles, speedsOtherVehicles)
            
            if strcmp(obj.plannerModes(plannerMode), 'MANUAL')
                if changeLaneCmd 
                    % Store lane changing points if valid lane chaning trajectory found
                    obj.calculateStaticLaneChangingManeuver(changeLaneCmd, s, d, 0, 0, v); 
                end
            elseif strcmp(obj.plannerModes(plannerMode), 'FORMAL')
            end
        end
        
        function d_oppositeLane = getOppositeLane(obj, d_currentLane)
        % Get lane opposite to current lane
            
            d_oppositeLane = 0;
        
            if d_currentLane == 0
                d_oppositeLane = obj.LaneWidth;
            end
        end
        
        function calculateStaticLaneChangingManeuver(obj, changeLaneCmd, s, d, d_dot, d_ddot, v)
        % Calculate lane changing maneuver either to the left or right lane
            
            if changeLaneCmd == obj.laneChangeCmds('CmdStartToLeftLane')
                obj.d_destination = obj.LaneWidth;
            elseif changeLaneCmd == obj.laneChangeCmds('CmdStartToRightLane')
                obj.d_destination = 0; 
            end
            
            % Static maneuver for 4s
            durationManeuver = 4; 
            [trajectoryFrenet, trajectoryCartesian, ~, ~] = obj.calculateLaneChangingTrajectory(s, d, d_dot, d_ddot, obj.d_destination, durationManeuver, v, obj.maximumAcceleration);

            obj.laneChangingTrajectoryFrenet = trajectoryFrenet;
            obj.laneChangingTrajectoryCartesian = trajectoryCartesian;

            x_trajectory = obj.laneChangingTrajectoryCartesian(:, 1);
            y_trajectory = obj.laneChangingTrajectoryCartesian(:, 2);

            plot(x_trajectory, y_trajectory, 'Color', 'green');
        end
        
        function [occupiedCells_otherVehiclesAhead, occupiedCells_otherVehiclesBehind] = getOccupiedCellsForOtherVehicles(obj, sEgo, poseOtherVehicles, speedsOtherVehicles, time)
        % Get occupied cells for all other vehicles ahead and behind the ego vehicle
            
            % TODO: Maybe only necessary to get occupied cells for surrounding vehicles    
            
            % Preallocation: At most 4 cells can be occupied for one (s, d) tuple for each vehicle
            occupiedCells_otherVehiclesAhead = zeros(size(poseOtherVehicles, 2)*4*obj.trajectoryReferenceLength, 4); 
            id_front = 1;
            occupiedCells_otherVehiclesBehind = zeros(size(poseOtherVehicles, 2)*4*obj.trajectoryReferenceLength, 4);
            id_rear = 1;
            
            % Iteration over all other vehicles
            for id_otherVehicle = 1:size(poseOtherVehicles, 2)
                x_otherVehicle = poseOtherVehicles(1, id_otherVehicle);
                y_otherVehicle = poseOtherVehicles(2, id_otherVehicle);
                
                [s_otherVehicle_current, d_otherVehicle] = Cartesian2Frenet(obj.RoadTrajectory, [x_otherVehicle, y_otherVehicle]);
                
                if s_otherVehicle_current > sEgo % Other vehicle ahead of ego vehcile
                    % Worst case: Calculate trajectory for full break
                    acceleration_worstCase = obj.minimumAcceleration; 
                else % Other vehicle behind ego vehicle
                    % Worst case: Calculate trajectory for maximum acceleration
                    acceleration_worstCase = obj.maximumAcceleration; 
                end
                    
                % Calculate other vehicle's trajectory
                [s_otherVehicle_trajectory, ~] = obj.calculateLongitudinalTrajectory(s_otherVehicle_current, speedsOtherVehicles(id_otherVehicle), obj.v_max, acceleration_worstCase, obj.trajectoryReferenceLength);
                d_otherVehicle_trajectory = d_otherVehicle*ones(1, size(s_otherVehicle_trajectory, 2));
                
                % Calculate occupied cells for other vehicle
                occupiedCells_otherVehicle = Continuous2Discrete(obj.spaceDiscretisation, s_otherVehicle_trajectory, d_otherVehicle_trajectory, time);
                
                % Add cells either to the list of cells for vehicles ahead or behind the ego vehicle
                if s_otherVehicle_current > sEgo % Ahead
                    [occupiedCells_otherVehiclesAhead, id_front] = obj.addToArray(occupiedCells_otherVehiclesAhead, occupiedCells_otherVehicle, id_front);
                else % Behind
                    [occupiedCells_otherVehiclesBehind, id_rear] = obj.addToArray(occupiedCells_otherVehiclesBehind, occupiedCells_otherVehicle, id_front);
                end
            end
            
            occupiedCells_otherVehiclesAhead = occupiedCells_otherVehiclesAhead(1:id_front-1, :);
            occupiedCells_otherVehiclesBehind = occupiedCells_otherVehiclesBehind(1:id_rear-1, :);
        end
        
        function [trajectoryFrenet, trajectoryCartesian, cost, isFeasibleTrajectory] = calculateLaneChangingTrajectory(obj, s_current, d_currnet, d_dot_current, d_ddot_current, d_destination, durationManeuver, v_current, a_ref)
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

            B = [d_currnet; d_dot_current; d_ddot_current; d_destination; 0; 0];

            X = linsolve(A,B);

            obj.a0 = X(1);
            obj.a1 = X(2); 
            obj.a2 = X(3);
            obj.a3 = X(4);
            obj.a4 = X(5);
            obj.a5 = X(6);
            
            % Calculate trajectory for complete maneuver
            t_maneuver = 0:obj.Ts:durationManeuver; 
            
            d_trajectory = obj.a0 + obj.a1*t_maneuver + obj.a2*t_maneuver.^2 + obj.a3*t_maneuver.^3 + obj.a4*t_maneuver.^4 + obj.a5*t_maneuver.^5;
            d_dot_trajectory = obj.a1 + 2*obj.a2*t_maneuver + 3*obj.a3*t_maneuver.^2 + 4*obj.a4*t_maneuver.^3 + 5*obj.a5*t_maneuver.^4;
            d_ddot_trajectory = 2*obj.a2 + 6*obj.a3*t_maneuver + 12*obj.a4*t_maneuver.^2 + 20*obj.a5*t_maneuver.^3;
            d_dddot_trajectory = 6*obj.a3 + 24*obj.a4*t_maneuver + 60*obj.a5*t_maneuver.^2;
            
            % Calculate velocity profile according to reference acceleration 
            [~, v_trajectory] = obj.calculateLongitudinalTrajectory(s_current, v_current, obj.vEgo_ref, a_ref, length(t_maneuver));
            
            % Calculate velocity in s-direction 
            s_dot_trajectory = sqrt(v_trajectory.^2 - d_dot_trajectory.^2); 
            
            % Calculate uture prediction for s-trajectory along the road
            s_trajectory = zeros(1, length(t_maneuver)); 
            s = s_current;
            for k = 1:length(t_maneuver) % Numerical integration
                s_trajectory(k) = s;
                
                % Sum up s-values knowing the velocity in s-direction and the discrete sample time
                s = s + s_dot_trajectory(k)*obj.Ts; 
            end
            
            % If the maneuver duration is shorter than the specified time horizon, 
            % a longitudinal trajectory following the destination lane needs to be added
            if durationManeuver < obj.timeHorizon 
                t_longitudinal = durationManeuver+obj.Ts:obj.Ts:obj.timeHorizon; 
                
                % Calculate initial displacement and velocity for t_longitudinal_0 = durationManeuver+obj.Ts
                [s_0, v_0] = obj.predictLongitudinalFutureState(s_trajectory(end), v_trajectory(end), obj.vEgo_ref, a_ref, 0);
                
                [s_trajectory_straight, v_trajectory_straight] = obj.calculateLongitudinalTrajectory(s_0, v_0, obj.vEgo_ref, obj.maximumAcceleration, length(t_longitudinal));
                
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
            
            [laneChangingPositionCartesian, roadOrientation] = Frenet2Cartesian(s_trajectory', d_trajectory', obj.RoadTrajectory);
            % The road orientation needs to be added (See documentation: "Calculate Trajectory for Lane Changing")
            orientation = atan2(d_dot_trajectory, s_dot_trajectory)' + roadOrientation;
            
            trajectoryFrenet = [s_trajectory', d_trajectory'];
            trajectoryCartesian = [laneChangingPositionCartesian, orientation];
            
            isFeasibleTrajectory = obj.isFeasibleTrajectory(trajectoryCartesian, d_ddot_trajectory, v_trajectory);
            
            % Cost function according to jerk
            cost = 0.5*sum(d_dddot_trajectory.^2);
        end 
        
        function [s_trajectory, v_trajectory] = calculateLongitudinalTrajectory(obj, s_0, v_0, v_max, acceleration, trajectoryLength)
        % Calculate longitudinal trajectory according to the longitudinal reachability analysis using each time step
            
            v_trajectory = zeros(1, trajectoryLength);
            s_trajectory = zeros(1, trajectoryLength);
            
            % Future prediction for displacement s, and velocityv
            s = s_0; 
            v = v_0;
            for k = 1:trajectoryLength 
                v_trajectory(k) = v;
                s_trajectory(k) = s;
                [s, v] = obj.predictLongitudinalFutureState(s, v, v_max, acceleration, 0); % Prediction just for next time step
            end
        end
        
        function isFeasible = isFeasibleTrajectory(obj, trajectory, a_lateral, v)
        % Check if trajectory is feasible accoording to lateral acceleration and trajectory curvature
            
            x = trajectory(:, 1);
            y = trajectory(:, 2);
            orientation = trajectory(:, 3);
            
            delta_orientation = diff(orientation);
            delta_position = sqrt((diff(x)).^2 + (diff(y)).^2);
            
            curvature = delta_orientation./delta_position;
            
            % Alternative: Limit by using centrifugal acceleration
            % a_lateral = v(1:end-1)'.^2.*curvature; 
            
            isFeasibleCurvature = all(abs(curvature) < obj.curvature_max);
            isFeasibleCentrifugalAcceleration = all(abs(a_lateral) < obj.a_lateral_max); % Placeholder for a_lateral_max
            
            isFeasible = isFeasibleCurvature && isFeasibleCentrifugalAcceleration;
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
        function [array, id_next] = addToArray(array, toAdd, id)
        % Add new elements to array at given index

            id_next = id + size(toAdd, 1);
            array(id:id_next-1, :) = toAdd;
        end
        
        function isSafeTrajectory = checkTrajectoryIntersection(trajectory_ego, trajectory_other, relativePosition_other)
        % Check whether discrete reference trajectory is safe by calculating the
        % intersection between discrete reference ego trajectory and other discrete trajectories
        %
        % relativePosition_other: Specifies, whether the other trajectories belong to
        %                         vehicles ahead or behind the ego vehicle on the road
        
            if ~(strcmp(relativePosition_other, 'ahead') || strcmp(relativePosition_other, 'behind'))
                error('Invalid relative position of other trajectories. Specify relative position as ''ahead'' or ''behind''.');
            end
            
            % If there is no intersection of the cells, the reference trajectory is safe 
            isSafeTrajectory = true;
            
            occupied_cells_ego = trajectory_ego(:, 1:2);
            occupied_cells_other = trajectory_other(:, 1:2);
            
            [commonCells, id_intersect_ego, id_intersect_other] = intersect(occupied_cells_ego, occupied_cells_other, 'rows');
            
            % Check the cells' temporal intersection
            if ~isempty(commonCells)
                if strcmp(relativePosition_other, 'ahead') % For vehicles ahead of the ego vehicle
                    t_enter_ego = trajectory_ego(id_intersect_ego, 3);
                    t_exit_other = trajectory_other(id_intersect_other, 4);
                    
                    % Safe, if ego vehicle enters after the other vehicle has left the cell
                    isSafeTrajectory = all(t_enter_ego > t_exit_other);
                else % For vehicles behind the ego vehicle
                    t_exit_ego = trajectory_ego(id_intersect_ego, 4);
                    t_enter_other = trajectory_other(id_intersect_other, 3);
                    
                    % Safe, if ego vehicle exits before the other vehicle has entered the cell
                    isSafeTrajectory = all(t_exit_ego < t_enter_other);
                end
            end
        end
    end
end

