classdef LocalTrajectoryPlanner < ReachabilityAnalysis
% LocalTrajectoryPlanner Superclass for generating necessary inputs for 
% the lateral controllers.
    
    properties(Nontunable)
        vEgo_ref % Reference speed for ego vehicle [m/s]
        
        vOtherVehicles_ref % Reference speed for other vehicles [m/s]
        
        v_max % Maximum allowed velocity
        
        LaneWidth % Width of road lane [m]
        RoadTrajectory % Road trajectory according to MOBATSim map format
        
        partsTimeHorizon % Divide time horizon into equal parts
        
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
        
        t_ref % Variable to store a specific simulation time of interest 
        
        isChangingLane % Return if currently executing lane changing maneuver
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
            
            obj.t_ref = 0; 
            
            obj.isChangingLane = false;
        end
        
        function planReferenceTrajectory(obj, changeLaneCmd, plannerMode, s, d, v, orientation, poseOtherVehicles, speedsOtherVehicles)
            
            if strcmp(obj.plannerModes(plannerMode), 'MANUAL')
                if changeLaneCmd 
                    % Store lane changing points if valid lane chaning trajectory found
                    obj.planStaticLaneChangingManeuver(changeLaneCmd, s, d, 0, 0, v); 
                end
            elseif strcmp(obj.plannerModes(plannerMode), 'FORMAL')
                % Only check everey timeHorizon/partsTimeHorizon seconds because expensive operation
                if get_param('VehicleFollowing', 'SimulationTime') >= obj.t_ref
                    obj.t_ref = obj.t_ref + obj.timeHorizon/obj.partsTimeHorizon;
                    
                    currentState_Ego = obj.createState(s, d, orientation, v);
                    
                    [s_other, d_other] = Cartesian2Frenet(obj.RoadTrajectory, [poseOtherVehicles(1, :)', poseOtherVehicles(2, :)']);
                    
                    % TODO: Preallocation
                    for id_other = 1:size(poseOtherVehicles, 2)
                        currentStates_Other(id_other) = obj.createState(s_other(id_other), d_other(id_other), poseOtherVehicles(3, id_other), speedsOtherVehicles(id_other));
                    end
                    
                    bestDecision_Ego = obj.planSafeManeuver(currentState_Ego, obj.d_destination, currentStates_Other, 2);
                    
                    if ~isempty(bestDecision_Ego)
                        nextDecision = bestDecision_Ego(1, :);
                        description_nextDecision = strsplit(nextDecision{4}, '_');
                        nextState = description_nextDecision{1};
                        
                        assignin('base', 'nextState', nextState);
                        
                        % Set lane changing points accordingly
                        obj.laneChangingTrajectoryFrenet = nextDecision{5};
                        obj.laneChangingTrajectoryCartesian = nextDecision{6};
                        if strcmp(nextState, 'ChangeLane')
                            obj.isChangingLane = true;
                            
                            obj.d_destination = nextDecision{3}.d;
                            
                            if abs(d - obj.d_destination) < 0.1
                                obj.isChangingLane = false;
                            end
                            
                            x_trajectory = obj.laneChangingTrajectoryCartesian(:, 1);
                            y_trajectory = obj.laneChangingTrajectoryCartesian(:, 2);

                            plot(x_trajectory, y_trajectory, 'Color', 'green');
                        end
                    end
                end
            end
        end
        
        function planStaticLaneChangingManeuver(obj, changeLaneCmd, s, d, d_dot, d_ddot, v)
        % Calculate lane changing maneuver either to the left or right lane
            
            if changeLaneCmd == obj.laneChangeCmds('CmdStartToLeftLane')
                obj.d_destination = obj.LaneWidth;
                destinationLane = 'left lane';
            elseif changeLaneCmd == obj.laneChangeCmds('CmdStartToRightLane')
                obj.d_destination = 0; 
                destinationLane = 'right lane';
            end
            
            % Static maneuver for 4s
            durationManeuver = 4; 
            [trajectoryFrenet, trajectoryCartesian, ~, ~, ~] = obj.calculateLaneChangingTrajectory(s, d, d_dot, d_ddot, obj.d_destination, durationManeuver, v, obj.maximumAcceleration);

            obj.laneChangingTrajectoryFrenet = trajectoryFrenet;
            obj.laneChangingTrajectoryCartesian = trajectoryCartesian;

            x_trajectory = obj.laneChangingTrajectoryCartesian(:, 1);
            y_trajectory = obj.laneChangingTrajectoryCartesian(:, 2);

            plot(x_trajectory, y_trajectory, 'Color', 'green');
            
            t = get_param('VehicleFollowing', 'SimulationTime');
            
            fprintf('@t=%fs: Start trajectory to %s, duration=%fs.\n', t, destinationLane, durationManeuver);
        end
        
        function bestDecision_Ego = planSafeManeuver(obj, state_Ego, d_goal, states_Other, depth2go)
        % Plan and decide for a safe maneuver according to specified
        % searching depth
            
            bestDecision_Ego = [];
            bestDecisionFuture_Ego = [];
            
            time_trajectory = 0:obj.Ts:obj.timeHorizon;
            
            % Decisions Ego
            if ~obj.isChangingLane
                decisions_Ego = obj.calculateDecisions_Ego_OnLane(state_Ego, d_goal, time_trajectory);
            else
                decisions_Ego = obj.calculateDecisions_Ego_LaneChange(state_Ego, d_goal, time_trajectory);
                obj.isChangingLane = false; % Set to false for future predictions for depth >= 2
            end
            
            % Feasibility check
            isFeasibleDecision = [decisions_Ego{:, 2}];
            decisionsFeasible_Ego = decisions_Ego(isFeasibleDecision, :);
            
            % Decisions other vehicles
            decisions_Other = obj.calculateDecisions_otherVehicles(states_Other, time_trajectory);
            
            % Safety check
            decisionsSafe_Ego = decisionsFeasible_Ego;
            for id_decision = size(decisionsFeasible_Ego, 1):-1:1 % Reverse to remove unsafe decisions without confusing idx
                TS_Ego = decisionsFeasible_Ego{id_decision, 1};
                
                isSafe_decision = true; 
                for id_other = 1:size(decisions_Other, 1)
                    TS_Other = decisions_Other{id_other, 1};
                    [isSafeTS, ~] = CellChecker.isSafeTransitions(TS_Ego, TS_Other);

                    % Even if only one transition is unsafe, decision is unsafe
                    isSafe_decision = isSafe_decision && isSafeTS; 
                end
                
                % Remove unsafe decisions
                if ~isSafe_decision
                    decisionsSafe_Ego(id_decision, :) = [];
                end
            end
            
            % No safe future states
            if isempty(decisionsSafe_Ego)
                return
            end

            % Check depth left to go for tree expansion
            depth2go = depth2go - 1;
            if depth2go == 0
                safeFutureStates_Ego = [decisionsSafe_Ego{:, 3}];

                % Choose according to max s-value
                [~, id_max] = max([safeFutureStates_Ego.s]);
                bestDecision_Ego = decisionsSafe_Ego(id_max, :);
                return
            end
            
            possibleFutureStates_Other = [decisions_Other{:, 3}];
%             % MODEL-FREE
%             futureStatesCombinations_Other = obj.getStateCombinations(possibleFutureStates_Other);
            
            % Expand tree for future states of safe decisions
            s_future_max = -Inf;
            for id_safeDecision = 1:size(decisionsSafe_Ego, 1)
                futureState_Ego = decisionsSafe_Ego{id_safeDecision, 3};
                d_futureGoal = futureState_Ego.d;
                
                % MODEL
                futureStates_Other = obj.findMostUnsafeAdversatialFutureStates(futureState_Ego, possibleFutureStates_Other);
                decisionSafeFuture_Ego = obj.planSafeManeuver(futureState_Ego, d_futureGoal, futureStates_Other, depth2go);
                
                if ~isempty(decisionSafeFuture_Ego)
                    bestDecisionFinal_Ego = decisionSafeFuture_Ego(end, :);
                    s_future = bestDecisionFinal_Ego{3}.s;
                    if s_future > s_future_max
                        bestDecision_Ego = decisionsSafe_Ego(id_safeDecision, :);
                        bestDecisionFuture_Ego = decisionSafeFuture_Ego;
                        s_future_max = s_future;
                    end     
                end
                
%                 % MODEL-FREE
%                 % Each safe future state needs to be called with 
%                 % all combinations of possible future states for other vehicles
%                 for id_statesOther = 1:size(futureStatesCombinations_Other, 1)
%                     futureStates_Other = futureStatesCombinations_Other(id_statesOther, :);
%                     futureDecisions_Ego = obj.planSafeManeuver(futureState_Ego, d_futureGoal, futureStates_Other, depth2go);
%                     
%                     temp = 3;
%                 end
            end
            
            bestDecision_Ego = [bestDecision_Ego; bestDecisionFuture_Ego];
        end
        
        function decisions = calculateDecisions_Ego_OnLane(obj, state, d_goal, time)
        % Calculate candidate trajectories (decisions) for different
        % driving modes while staying on one lane
            
            % FreeDrive
            decisionsFD = obj.getDecisionsForDrivingMode(state, d_goal, 1, obj.maximumAcceleration, 'FreeDrive', time);

            % VehicleFollowing
            decisionsVF = obj.getDecisionsForDrivingMode(state, d_goal, obj.minimumAcceleration, 0, 'VehicleFollowing', time);

            % EmergencyBrake
            decisionsEB = obj.getDecisionsForDrivingMode(state, d_goal, obj.emergencyAcceleration, obj.emergencyAcceleration, 'EmergencyBrake', time);

            % ChangeLane
            d_otherLane = obj.getOppositeLane(d_goal);
            decisionsCL = obj.getDecisionsForLaneChange(state, d_otherLane, 0, 0, time);
            
            % All possible decisions
            decisions = [decisionsFD; decisionsVF; decisionsEB; decisionsCL];
        end
        
        function decisions = calculateDecisions_Ego_LaneChange(obj, state, d_goal, time)
        % Calculate candidate trajectories (decisions) while executing lane change
        
            [~, roadOrientation] = Frenet2Cartesian(state.s, state.d, obj.RoadTrajectory);
            d_dot = state.speed*sin(state.orientation - roadOrientation);
            
            d_ddot = 0; % Simplification: Assume d_ddot = 0
        
            % ChangeLane
            decisionsCL = obj.getDecisionsForLaneChange(state, d_goal, d_dot, d_ddot, time);
            
            % Abort: ChangeBack
            d_otherLane = getOppositeLane(obj, obj.d_destination);
            decisionsCB = obj.getDecisionsForLaneChange(state, d_otherLane, d_dot, d_ddot, time);
            
            % All possible decisions
            decisions = [decisionsCL; decisionsCB];
        end
        
        function decisions = getDecisionsForDrivingMode(obj, state, d_goal, acc_lower, acc_upper, name_DrivingMode, time)
        % Get the decisions for a driving mode for different accelerations
            
            number_decisions = length(acc_lower:1:acc_upper);

            decisions = cell(number_decisions, 3);
            id_decision = 1;
            
            for acc = acc_lower:1:acc_upper 
                description = [name_DrivingMode, '_acc', num2str(acc)];
                
                % Trajectory prediction
                [s_trajectory, v_trajectory] = obj.calculateLongitudinalTrajectory(state.s, state.speed, obj.v_max, acc, obj.trajectoryReferenceLength);
                d_trajectory = d_goal*ones(1, size(s_trajectory, 2));
                
                % Future state prediction
                [~, futureOrientation] = Frenet2Cartesian(s_trajectory(end), d_trajectory(end), obj.RoadTrajectory);
                futureState = obj.createState(s_trajectory(end), d_trajectory(end), futureOrientation, v_trajectory(end));
                
                % Discrete trajectory
                occupiedCells = Continuous2Discrete(obj.spaceDiscretisation, s_trajectory, d_trajectory, time);
                TS  = CellChecker.createTSfromCells(occupiedCells);
                
                [decisions, id_decision] = obj.addDecision(decisions, TS, true, futureState, description, [], [], id_decision);
            end
        end
        
        function decisions = getDecisionsForLaneChange(obj, state, d_goal, d_dot, d_ddot, time)
        % Get the decisions for a lane change for different maneuver times
            
            number_decisions = length(2:1:obj.timeHorizon);

            decisions = cell(number_decisions, 3);
            id_decision = 1;
            
            acc = 0;
            for durManeuver = 2:1:obj.timeHorizon
                description = ['ChangeLane', '_T', num2str(durManeuver)];
                
                [trajectoryFrenet, trajectoryCartesian, trajectorySpeed, ~, isFeasibleTrajectory] = obj.calculateLaneChangingTrajectory(state.s, state.d, d_dot, d_ddot, d_goal, durManeuver, state.speed, acc);
                s_trajectory = trajectoryFrenet(:, 1)';
                d_trajectory = trajectoryFrenet(:, 2)';

                % Future state prediction
                futureOrientation = trajectoryCartesian(end, 3);
                futureState = obj.createState(s_trajectory(end), d_trajectory(end), futureOrientation, trajectorySpeed(end));
                
                occupiedCells = Continuous2Discrete(obj.spaceDiscretisation, s_trajectory, d_trajectory, time);
                
                TS  = CellChecker.createTSfromCells(occupiedCells);
                
                [decisions, id_decision] = obj.addDecision(decisions, TS, isFeasibleTrajectory, futureState, description, trajectoryFrenet, trajectoryCartesian, id_decision);
            end  
        end
        
        function d_oppositeLane = getOppositeLane(obj, d_currentLane)
        % Get lane opposite to current lane
            
            d_oppositeLane = 0;
        
            if d_currentLane == 0
                d_oppositeLane = obj.LaneWidth;
            end
        end
        
        function decision_otherVehicles = calculateDecisions_otherVehicles(obj, states_Other, time)
        % Get occupied cells for all other vehicles ahead and behind the ego vehicle
            
            % TODO: Maybe only necessary to get occupied cells for surrounding vehicles    
            n_other = length(states_Other); 
            
            decision_otherVehicles = cell(1, n_other); 
            
            % Iteration over all other vehicles
            for id_otherVehicle = 1:n_other 
                description = ['Other Vehicle ', num2str(id_otherVehicle)];
                
                state_Other = states_Other(id_otherVehicle);
                
                % Calculate other vehicle's trajectory for a_min and a_max
                [s_trajectory_min, v_trajectory_min] = obj.calculateLongitudinalTrajectory(state_Other.s, state_Other.speed, obj.v_max, obj.minimumAcceleration, obj.trajectoryReferenceLength);
                d_trajectory = state_Other.d*ones(1, size(s_trajectory_min, 2));
                
                [s_trajectory_max, v_trajectory_max] = obj.calculateLongitudinalTrajectory(state_Other.s, state_Other.speed, obj.v_max, obj.maximumAcceleration, obj.trajectoryReferenceLength);
                
                % Future state prediction (Min)
                [~, futureOrientation_min] = Frenet2Cartesian(s_trajectory_min(end), d_trajectory(end), obj.RoadTrajectory);
                futureState_min = obj.createState(s_trajectory_min(end), d_trajectory(end), futureOrientation_min, v_trajectory_min(end));
                
                 % Future state prediction (Max)
                [~, futureOrientation_max] = Frenet2Cartesian(s_trajectory_max(end), d_trajectory(end), obj.RoadTrajectory);
                futureState_max = obj.createState(s_trajectory_max(end), d_trajectory(end), futureOrientation_max, v_trajectory_max(end));
                
                % Calculate occupied cells for other vehicle
                occupiedCells_min = Continuous2Discrete(obj.spaceDiscretisation, s_trajectory_min, d_trajectory, time);
                occupiedCells_max = Continuous2Discrete(obj.spaceDiscretisation, s_trajectory_max, d_trajectory, time);
                
                % Worst case: earliest entering time (occupiedCells_min) 
                % latest exiting time (occupiedCells_max)
                [~, id_intersect_min, id_intersect_max] = intersect(occupiedCells_min(:, 1:2), occupiedCells_max(:, 1:2), 'rows');
                occupiedCells_worstCase = occupiedCells_max; % Entering times from occupiedCells_max
                occupiedCells_worstCase(:, 4) = Inf; % Assume worst case for exit time (vehicle could potentially stop at every cell)
                occupiedCells_worstCase(id_intersect_max, 4) = occupiedCells_min(id_intersect_min, 4); % Exit times from occupiedCells_min
                
                TS_otherVehicle = CellChecker.createTSfromCells(occupiedCells_worstCase);
                
                % Concrete trajectories not needed for other vehicles
                [decision_otherVehicles, ~] = obj.addDecision(decision_otherVehicles, TS_otherVehicle, true, [futureState_min; futureState_max], description, [], [], id_otherVehicle);
            end 
        end
        
        function [trajectoryFrenet, trajectoryCartesian, v_trajectory, cost, isFeasibleTrajectory] = calculateLaneChangingTrajectory(obj, s_current, d_current, d_dot_current, d_ddot_current, d_destination, durationManeuver, v_current, a_ref)
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
            
            
            if any(v_trajectory.^2 < d_dot_trajectory.^2)
                % Trajectory not feasible
                isFeasibleTrajectory = false;
                
                trajectoryFrenet = [s_current, d_current];
                
                [position_current, roadOrientation_current] = Frenet2Cartesian(s_current, d_current, obj.RoadTrajectory);
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
        function state = createState(s, d, orientation, speed)
        % Create State with information about pose and speed
            
            state.s = s;
            state.d = d;
            state.orientation = orientation;
            state.speed = speed;
        end
        
        function stateCombinations = getStateCombinations(states)
        % Get all possible state combinations for each decision of each
        % vehicle
            
            n_decisions = size(states, 1);
            n_vehicles = size(states, 2);
            
            id_combinations = 1:n_vehicles;
            for id_vehicle = 2:n_vehicles
                id_combinations = combvec(id_combinations, 1:n_decisions);
            end
            
            id_combinations = id_combinations';
            
            % TODO: Preallocation
            for id_vehicle = 1:n_vehicles
                stateCombinations(:, id_vehicle) = states(id_combinations(:, id_vehicle), id_vehicle);
            end
        end
        
        function worstFutureState = findMostUnsafeAdversatialFutureStates(futureState_Ego, possibleFutureStates_Other)
        % Find the future states for the other vehicles which are the most unsafe for the ego
        % vehicle according to some metric
            
            % TODO: Preallocation
            for id_vehicle_other = 1:size(possibleFutureStates_Other, 2)
                % Distance metric according to delta_s
                s_other = [possibleFutureStates_Other(:, id_vehicle_other).s];
                
                delta_s = futureState_Ego.s - s_other;
                [~, id_min] = min(abs(delta_s));
                worstFutureState(id_vehicle_other) = possibleFutureStates_Other(id_min, id_vehicle_other);
            end
        end
        
        function [array, id_next] = addToArray(array, toAdd, id)
        % Add new elements to array at given index

            id_next = id + size(toAdd, 1);
            array(id:id_next-1, :) = toAdd;
        end
        
        function [array, id_next] = addDecision(array, TS, isFeasibile, futureState, description, trajectoryFrenet_LC, trajectoryCartesian_LC, id)
        % Add new decisions (TS, feasibility, futureState, description, trajectoryFrenet, trajectoryCartesian) 
        % to cell array at the given index
            
            array{id, 1} = TS;
            array{id, 2} = isFeasibile;
            array{id, 3} = futureState;
            array{id, 4} = description;
            array{id, 5} = trajectoryFrenet_LC;
            array{id, 6} = trajectoryCartesian_LC;
            
            id_next = id + 1;
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

