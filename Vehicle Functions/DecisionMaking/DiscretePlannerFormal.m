classdef DiscretePlannerFormal < DecisionMaking
% Select driving mode and decide if to execute lane changing maneuver according to formal design
    
    properties(Nontunable)
        timeHorizon % Time horizon for trajectory genereation [s]
        partsTimeHorizon % Divide time horizon into equal parts
        
        minimumAcceleration % Minimum longitudinal acceleration [m/s^2]
        maximumAcceleration % Maximum longitudinal acceleration [m/s^2]
        emergencyAcceleration % Acceleration for emergency break [m/s^2]
        maximumVelocity % Maximum allowed longitudinal velocity [m/s]
        vOtherVehicles_ref % Reference speed for other vehicles [m/s]
        
        wheelBase % Wheel base vehicle [m]
        steerAngle_max % Maximum steering angle [rad]
        
        sigmaS % Standard deviation for measuring other vehicles' s-coordinate [m]
        sigmaV % Standard deviation for measuring other vehicles' speeds [m/s]
        
        spaceDiscretisation % Space Discretisation
    end

    % Pre-computed constants
    properties(Access = protected)
        d_destination % Reference lateral destination (right or left lane)
        isChangingLane % Return if currently executing lane changing maneuver
        
        trajectoryReferenceLength % Number of points for trajectory generation according to specified time horizon 
        
        t_ref % Variable to store a specific simulation time of interest 
        
        nodeID % To identify unique nodes in digraph
        searchDepth  % Depth for search algorithm
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            setupImpl@DecisionMaking(obj); 
            
            obj.d_destination = 0; % Start on right lane
            obj.isChangingLane = false;
            
            obj.t_ref = 0; 
            
            % +1 because planned trajectory also contains current waypoint at current time
            obj.trajectoryReferenceLength = obj.timeHorizon/obj.Ts + 1; 
            
            obj.curvature_max = tan(obj.steerAngle_max)/obj.wheelBase;
            
            obj.nodeID = 0;
            obj.searchDepth = 2;
            
            stateNames = {...
                % Keep Lane
                'FreeDrive', 1;
                'VehicleFollowing', 2;
                'EmergencyBrake', 3;
                
                % Change Lane
                'ChangeLane', 4;
                };
            obj.states = containers.Map(stateNames(:, 1)', [stateNames{:, 2}]);
            
            % Initial state: Free Drive and on the right lane 
            obj.currentState = obj.states('FreeDrive');
            disp('@t=0s: Initial state is: ''FreeDrive''.');
        end
        
        function [changeLaneCmd, drivingMode] = stepImpl(obj, poseEgo, poseOtherVehicles, speedsOtherVehicles, vEgo)
        % Return lane change command, the current lane state and the current driving mode (see system description)
            
            [sEgo, dEgo] = Cartesian2Frenet(obj.RoadTrajectory, [poseEgo(1) poseEgo(2)]);
            orientationEgo = poseEgo(3);
            
            obj.previousState = obj.currentState;
            
            % Necessary to return some output even if there is no command
            changeLaneCmd = obj.laneChangeCmds('CmdIdle');
            
            if obj.isChangingLane && (abs(dEgo - obj.d_destination) < 0.05)
                obj.isChangingLane = false;
            end
            
            % Only check everey timeHorizon/partsTimeHorizon seconds because expensive operation
            % and if not changing lane
            if get_param('VehicleFollowing', 'SimulationTime') >= obj.t_ref && ~obj.isChangingLane
                obj.t_ref = get_param('VehicleFollowing', 'SimulationTime') + obj.timeHorizon/obj.partsTimeHorizon;
                
                % Define vehicle states
                currentState_Ego = obj.createState(sEgo, dEgo, orientationEgo, vEgo);
                    
                [sOther, dOther] = Cartesian2Frenet(obj.RoadTrajectory, [poseOtherVehicles(1, :)', poseOtherVehicles(2, :)']);

                currentStates_Other = obj.preallocateStates(1, size(poseOtherVehicles, 2));
                for id_other = 1:size(poseOtherVehicles, 2)
                    currentStates_Other(id_other) = obj.createState(sOther(id_other), dOther(id_other), poseOtherVehicles(3, id_other), speedsOtherVehicles(id_other));
                end
                
                % Initialise digraph
                obj.nodeID = 0;
                initNode_Ego = DigraphTree.getNodeName(obj.nodeID, currentState_Ego, 0);

                dG_initial = DigraphTree.initialise(initNode_Ego, [0, 1, 1]);

                [dG_initial, ~, obj.nodeID] = DigraphTree.expand(dG_initial, obj.nodeID, initNode_Ego, currentStates_Other, 'InitialStates', [1, 0, 1], [1, 0, 1], 0);
                
                % Best decision Ego
                [bestDecision_Ego, dG_final] = obj.planSafeManeuver(currentState_Ego, obj.d_destination, currentStates_Other, -Inf, Inf, dG_initial, obj.nodeID, obj.searchDepth);
            
                if isempty(bestDecision_Ego)
                    % TODO: Minimum Safety Violation
                    obj.currentState = obj.states('EmergencyBrake');
                else
                    nextDecision = bestDecision_Ego(1, :);
                    description_nextDecision = strsplit(nextDecision{4}, '_');
                    nextState = description_nextDecision{1};
                    obj.currentState = obj.states(nextState);
                    
                    if strcmp(nextState, 'ChangeLane')
                        obj.isChangingLane = true;

                        obj.d_destination = nextDecision{3}.d;
                        
                        
                        T_LC = extractBetween(description_nextDecision{2}, '{T', '}');
                        changeLaneCmd = str2double(T_LC{1});
                    end
                end
            end
            
            drivingMode = obj.getStateInfo(obj.currentState);
            
            obj.displayNewState(obj.currentState, obj.previousState);
        end
        
        function [decisionMax_Ego, graph] = planSafeManeuver(obj, state_Ego, d_goal, states_Other, alpha, beta, graph, parentID, depth2go)
        % Plan and decide for a safe maneuver according to specified
        % searching depth
            
            decisionMax_Ego = [];
            decisionNext_Ego = [];
            
            parentNode = DigraphTree.getNodeName(parentID, states_Other, obj.searchDepth-depth2go);
            bestNode = [];
            
            time_trajectory = 0:obj.Ts:obj.timeHorizon;
            
            depth2go = depth2go - 1;
            
            % Decisions Ego
            decisions_Ego = obj.calculateDecisions_Ego(state_Ego, d_goal, time_trajectory);
            
            % Feasibility check
            isFeasibleDecision = [decisions_Ego{:, 2}];
            decisionsFeasible_Ego = decisions_Ego(isFeasibleDecision, :);
            
            % Decisions other vehicles
            decisions_Other = obj.calculateDecisions_Other(states_Other, time_trajectory);
            possibleFutureStates_Other = [decisions_Other{:, 3}];
            futureStatesCombinations_Other = obj.getStateCombinations(possibleFutureStates_Other);
            
            % Safety check
            decisionsSafe_Ego = decisionsFeasible_Ego;
            s_future_max = -Inf;
            for id_decision = size(decisionsFeasible_Ego, 1):-1:1 % Reverse to remove unsafe decisions without confusing idx
                TS_Ego = decisionsFeasible_Ego{id_decision, 1};
                
                isSafe_decision = true; 
                for id_other = 1:size(decisions_Other, 1)
                    TS_Other = decisions_Other{id_other, 1};
                    [isSafeTS, ~] = CellChecker.isSafeTransitions(TS_Ego, TS_Other);

                    % Even if only one transition is unsafe, decision is unsafe
                    isSafe_decision = isSafe_decision && isSafeTS; 
                end
                
                if ~isSafe_decision
                    % Remove unsafe decisions
                    decisionsSafe_Ego(id_decision, :) = [];
                else 
                    % Add safe decision to digraph
                    [graph, childNode, obj.nodeID] = DigraphTree.expand(graph, obj.nodeID, parentNode, ...
                        decisionsSafe_Ego{id_decision, 3}, decisionsSafe_Ego{id_decision, 4}, ...
                        [0, 1, 0], [0, 1, 0], obj.searchDepth-depth2go);
                    
                    % Expand tree for future states of safe decisions
                    if depth2go > 0
                        futureState_Ego = decisionsSafe_Ego{id_decision, 3};
                        d_futureGoal = futureState_Ego.d;

                        [decisionMinFuture_Ego, graph] = obj.planUnsafeManeuver(futureState_Ego, d_futureGoal, futureStatesCombinations_Other, alpha, beta, graph, obj.nodeID, depth2go);

                        % Max behaviour: Ego vehicle
                        if ~isempty(decisionMinFuture_Ego)
                            decisionFinal_Ego = decisionMinFuture_Ego(end, :);
                            s_future = decisionFinal_Ego{3}.s;
                            if s_future > s_future_max
                                decisionNext_Ego = decisionMinFuture_Ego;
                                decisionMax_Ego = decisionsSafe_Ego(id_decision, :);
                                bestNode = childNode;
                                s_future_max = s_future;
                            end    

                            alpha = max(alpha, s_future);
                            if beta <= alpha
                                break
                            end
                        end
                    end
                end
            end
            
            % No safe future states
            if isempty(decisionsSafe_Ego)
                return
            end

            % Check depth left to go for tree expansion
            if depth2go == 0
                safeFutureStates_Ego = [decisionsSafe_Ego{:, 3}];

                % Choose according to max s-value
                [~, id_max] = max([safeFutureStates_Ego.s]);
                decisionMax_Ego = decisionsSafe_Ego(id_max, :);
                
                % Higlight best node and edge in digraph
                bestNode = DigraphTree.getNodeName(obj.nodeID+1-id_max, safeFutureStates_Ego(id_max), obj.searchDepth-depth2go);
                graph = DigraphTree.changeNodeColor(graph, bestNode, [0, 1, 1]);
                graph = DigraphTree.changeEdgeColor(graph, parentNode, bestNode, [0, 1, 1]);
                
                return
            end
            
            % Higlight best node and edge in digraph
            if ~isempty(decisionMax_Ego) 
                graph = DigraphTree.changeNodeColor(graph, bestNode, [0, 1, 1]);
                graph = DigraphTree.changeEdgeColor(graph, parentNode, bestNode, [0, 1, 1]);
            end
            
            decisionMax_Ego = [decisionMax_Ego; decisionNext_Ego];
        end
        
        function [decisionMinFuture_Ego, graph] = planUnsafeManeuver(obj, futureState_Ego, d_futureGoal, futureStatesCombinations_Other, alpha, beta, graph, parentID, depth2go)
        % For other vehicles choose the action, which is considered the most unsafe
            
            parentNode = DigraphTree.getNodeName(parentID, futureState_Ego, obj.searchDepth-depth2go);
            worstNode = [];
        
            s_future_min = Inf;
            for id_statesOther = 1:size(futureStatesCombinations_Other, 1)
                futureStates_Other = futureStatesCombinations_Other(id_statesOther, :);
                
                % Add other vehicles' possible decisions to digraph
                [graph, childNode, obj.nodeID] = DigraphTree.expand(graph, obj.nodeID, parentNode, ...
                        futureStates_Other, ['Combination', num2str(id_statesOther)], ...
                        [1, 0, 0], [1, 0, 0], obj.searchDepth-depth2go);
                
                [decisionFuture_Ego, graph] = obj.planSafeManeuver(futureState_Ego, d_futureGoal, futureStates_Other, alpha, beta, graph, obj.nodeID, depth2go);
                
                % Min behaviour: Other vehicles
                if isempty(decisionFuture_Ego)
                    decisionMinFuture_Ego = [];
                    break % This decision is unsafe if at least one possibility is unsafe
                else
                    decisionFinal_Ego = decisionFuture_Ego(end, :);
                    s_future = decisionFinal_Ego{3}.s;
                    if s_future < s_future_min
                        decisionMinFuture_Ego = decisionFuture_Ego;
                        worstNode = childNode;
                        s_future_min = s_future;
                    end   
                    
                    beta = min(beta, s_future);
                    if beta <= alpha
                        break
                    end
                end
            end
            
            % Higlight worst node and edge in digraph
            if ~isempty(worstNode)
                graph = DigraphTree.changeNodeColor(graph, worstNode, [1, 0, 1]);
                graph = DigraphTree.changeEdgeColor(graph, parentNode, worstNode, [1, 0, 1]);
            end
        end
        
        function decisions = calculateDecisions_Ego(obj, state, d_goal, time)
        % Calculate candidate trajectories (decisions) for different driving modes 
            
            % FreeDrive
            decisionsFD = obj.getDecisionsForDrivingMode(state, d_goal, 1, obj.maximumAcceleration, 'FreeDrive', time);

            % VehicleFollowing
            decisionsVF = [];
            % ChangeLane
            decisionsCL = [];
            if state.speed > 0.001 % Avoid very slow velocities
                % For v <= 0, vehicle following results in the same as EmergencyBrake
                decisionsVF = obj.getDecisionsForDrivingMode(state, d_goal, obj.minimumAcceleration, 0, 'VehicleFollowing', time);
                
                % For v <= 0, ChangeLane is unfeasible
                d_otherLane = obj.getOppositeLane(d_goal, obj.LaneWidth);
                decisionsCL = obj.getDecisionsForLaneChange(state, d_otherLane, 0, 0, time);
            end

            % EmergencyBrake
            decisionsEB = obj.getDecisionsForDrivingMode(state, d_goal, obj.emergencyAcceleration, obj.emergencyAcceleration, 'EmergencyBrake', time);
            
            % All possible decisions
            % TODO: Find order for eficient expansion
            decisions = [decisionsCL; decisionsEB; decisionsVF; decisionsFD];
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

            decisions = cell(number_decisions, 6);
            id_decision = 1;
            
            for acc = acc_lower:1:acc_upper 
                description = [name_DrivingMode, '_{acc', num2str(acc), '}'];
                
                % Trajectory prediction
                [s_trajectory, v_trajectory] = ...
                    LocalTrajectoryPlanner.calculateLongitudinalTrajectory(state.s, state.speed, ...
                                                                           obj.vEgo_ref, acc, ...
                                                                           obj.trajectoryReferenceLength, ...
                                                                           obj.Ts);
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
            
            dur_lower = min(2, obj.timeHorizon);
        
            number_decisions = length(dur_lower:1:obj.timeHorizon);

            decisions = cell(number_decisions, 6);
            id_decision = 1;
            
            acc = obj.maximumAcceleration; % Free Drive
            for durManeuver = dur_lower:1:obj.timeHorizon
                description = ['ChangeLane', '_{T', num2str(durManeuver), '}'];
                
                [trajectoryFrenet, trajectoryCartesian, trajectorySpeed, ~, isFeasibleTrajectory] = ...
                    LocalTrajectoryPlanner.calculateLaneChangingTrajectory(state.s, state.d, d_dot, d_ddot, d_goal, ...
                                                                           state.speed, obj.vEgo_ref, acc, ...
                                                                           durManeuver, obj.curvature_max, ...
                                                                           obj.RoadTrajectory, obj.timeHorizon, obj.Ts);
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
        
        function decision_otherVehicles = calculateDecisions_Other(obj, states_Other, time)
        % Get occupied cells for all other vehicles ahead and behind the ego vehicle
            
            % TODO: Maybe only necessary to get occupied cells for surrounding vehicles    
            n_other = length(states_Other); 
            
            decision_otherVehicles = cell(n_other, 6); 
            
            % Iteration over all other vehicles
            for id_otherVehicle = 1:n_other 
                description = ['Other Vehicle ', num2str(id_otherVehicle)];
                
                state_Other = states_Other(id_otherVehicle);
                
                % Worst case states according to uncertainty
                s_min = state_Other.s - 3*obj.sigmaS;
                v_min = state_Other.speed - 3*obj.sigmaV; 
                s_max = state_Other.s + 3*obj.sigmaS;
                v_max = state_Other.speed + 3*obj.sigmaV; 
                
                % Calculate other vehicle's trajectory for a_min and a_max
                [s_trajectory_min, v_trajectory_min] = ...
                    LocalTrajectoryPlanner.calculateLongitudinalTrajectory(s_min, v_min, ...
                                                                           obj.maximumVelocity, obj.minimumAcceleration, ...
                                                                           obj.trajectoryReferenceLength, obj.Ts);
                [s_trajectory_max, v_trajectory_max] = ...
                    LocalTrajectoryPlanner.calculateLongitudinalTrajectory(s_max, v_max, ...
                                                                           obj.maximumVelocity, obj.maximumAcceleration, ...
                                                                           obj.trajectoryReferenceLength, obj.Ts);
                                                                       
                d_trajectory = state_Other.d*ones(1, size(s_trajectory_min, 2));
                
                % Future state prediction (Min)
                [~, futureOrientation_min] = Frenet2Cartesian(s_trajectory_min(end), d_trajectory(end), obj.RoadTrajectory);
                futureState_min = obj.createState(s_trajectory_min(end), d_trajectory(end), futureOrientation_min, v_trajectory_min(end));
                
                 % Future state prediction (Max)
                [~, futureOrientation_max] = Frenet2Cartesian(s_trajectory_max(end), d_trajectory(end), obj.RoadTrajectory);
                futureState_max = obj.createState(s_trajectory_max(end), d_trajectory(end), futureOrientation_max, v_trajectory_max(end));
                
                % Calculate occupied cells for other vehicle
                occupiedCells_min = Continuous2Discrete(obj.spaceDiscretisation, s_trajectory_min, d_trajectory, time);
                occupiedCells_max = Continuous2Discrete(obj.spaceDiscretisation, s_trajectory_max, d_trajectory, time);
                
                % Worst case: earliest entrance times (occupiedCells_max) 
                % latest exit times (occupiedCells_min)
                occupiedCells_worstCase = zeros(size(occupiedCells_max, 1)+1, size(occupiedCells_max, 2));
                occupiedCells_worstCase(2:end, :) = occupiedCells_max; % Entrance times from occupiedCells_max
                if isempty(intersect(occupiedCells_min(1, 1:2), occupiedCells_max(1, 1:2), 'rows'))
                    % First occupied cell is not identical for min/max
                    % case, thus add the min case starting cell 
                    occupiedCells_worstCase(1, :) = occupiedCells_min(1, :);
                else
                    occupiedCells_worstCase(1, :) = [];
                end
                
                [~, id_intersect_min, id_intersect_max] = intersect(occupiedCells_min(:, 1:2), occupiedCells_worstCase(:, 1:2), 'rows');
                occupiedCells_worstCase(:, 4) = Inf; % Assume worst case for exit time (vehicle could potentially stop at every cell)
                occupiedCells_worstCase(id_intersect_max, 4) = occupiedCells_min(id_intersect_min, 4); % Exit times from occupiedCells_min
                
                TS_otherVehicle = CellChecker.createTSfromCells(occupiedCells_worstCase);
                
                % Concrete trajectories not needed for other vehicles
                [decision_otherVehicles, ~] = obj.addDecision(decision_otherVehicles, TS_otherVehicle, true, [futureState_min; futureState_max], description, [], [], id_otherVehicle);
            end 
        end 
        
        function [drivingMode, changeLaneCmd] = makeDecision(obj)
        % Make decision about driving mode and whether to change lane   
        
            % Necessary to return some output even if there is no command
            changeLaneCmd = obj.laneChangeCmds('CmdIdle');
            
            obj.currentState = obj.states(evalin('base', 'nextState'));
            
            drivingMode = obj.getStateInfo(obj.currentState);
        end
        
        function drivingMode = getStateInfo(obj, state)
        % Get information about driving mode given a state

            switch state
                case obj.states('FreeDrive')
                    drivingMode = obj.drivingModes('FreeDrive');
                case obj.states('VehicleFollowing')
                    drivingMode = obj.drivingModes('VehicleFollowing');
                case obj.states('EmergencyBrake')
                    drivingMode = obj.drivingModes('EmergencyBrake');
                case obj.states('ChangeLane')
                    drivingMode = obj.drivingModes('FreeDrive');
            end
        end
        
        function [out1, out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out1 = [1 1];
            out2 = [1 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end

        function [out1, out2] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out1 = "double";
            out2 = "double";

            % Example: inherit data type from first input port
            % out = propagatedInputDataType(obj,1);
        end

        function [out1, out2] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;

            % Example: inherit complexity from first input port
            % out = propagatedInputComplexity(obj,1);
        end

        function [out1, out2] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end    
    end
    
    methods(Static)
        function d_oppositeLane = getOppositeLane(d_currentLane, laneWidth)
        % Get lane opposite to current lane
            
            d_oppositeLane = 0;
        
            if d_currentLane == 0
                d_oppositeLane = laneWidth;
            end
        end
        
        function state = createState(s, d, orientation, speed)
        % Create State with information about pose and speed
            
            state.s = s;
            state.d = d;
            state.orientation = orientation;
            state.speed = speed;
        end
        
        function states = preallocateStates(n_row, n_col)
        % Preallocate struct array containing the states
            
            states = struct('s', cell(n_row, n_col), 'd', cell(n_row, n_col), ...
                'orientation', cell(n_row, n_col), 'speed', cell(n_row, n_col));
        end
        
        function stateCombinations = getStateCombinations(possiblestates)
        % Get all possible state combinations for each decision of each
        % vehicle
            
            % n_stateCombinations = n_possibleStates^n_vehicles
            n_possibleStates = size(possiblestates, 1);
            n_vehicles = size(possiblestates, 2);
            
            id_combinations = 1:n_possibleStates;
            for id_vehicle = 2:n_vehicles
                id_combinations = combvec(id_combinations, 1:n_possibleStates);
            end
            
            id_combinations = id_combinations';
            
            stateCombinations = DiscretePlannerFormal.preallocateStates(size(id_combinations, 1), n_vehicles);
            for id_vehicle = 1:n_vehicles
                stateCombinations(:, id_vehicle) = possiblestates(id_combinations(:, id_vehicle), id_vehicle);
            end
        end
        
        function worstFutureState = findMostUnsafeAdversatialFutureStates(futureState_Ego, possibleFutureStates_Other)
        % Find the future states for the other vehicles which are the most unsafe for the ego
        % vehicle according to some metric
            
            worstFutureState = DiscretePlannerFormal.preallocateStates(1, size(possibleFutureStates_Other, 2));
            for id_vehicle_other = 1:size(possibleFutureStates_Other, 2)
                % Distance metric according to delta_s
                s_other = [possibleFutureStates_Other(:, id_vehicle_other).s];
                
                delta_s = futureState_Ego.s - s_other;
                [~, id_min] = min(abs(delta_s));
                worstFutureState(id_vehicle_other) = possibleFutureStates_Other(id_min, id_vehicle_other);
            end
        end
        
        function [decisions, id_next] = addDecision(decisions, TS, isFeasibile, futureState, description, trajectoryFrenet_LC, trajectoryCartesian_LC, id)
        % Add new decisions (TS, feasibility, futureState, description, trajectoryFrenet, trajectoryCartesian) 
        % to cell array at the given index
            
            decisions{id, 1} = TS;
            decisions{id, 2} = isFeasibile;
            decisions{id, 3} = futureState;
            decisions{id, 4} = description;
            decisions{id, 5} = trajectoryFrenet_LC;
            decisions{id, 6} = trajectoryCartesian_LC;
            
            id_next = id + 1;
        end
    end
end
