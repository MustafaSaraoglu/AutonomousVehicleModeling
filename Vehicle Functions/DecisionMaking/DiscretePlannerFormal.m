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
        k_timeHorizon % Number of discrete time steps according to specified time horizon
        
        t_ref % Variable to store a specific simulation time of interest 
        
        nodeID % To identify unique nodes in digraph
        searchDepth  % Depth for search algorithm
        depthBound % Maximum depth to search for in one iteration
        safety_limit % Safety boundary for each depth
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
            
            % Discrete steps for one time horizon according to: timeHorizon = (k+1)*Ts
            obj.k_timeHorizon = obj.timeHorizon/obj.Ts - 1;
            
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
                
                % TODO: Free Drive if no vehicle in front on right lane for faster computation?
                
                obj.safety_limit = -Inf*ones(1, obj.searchDepth); % Maximum safety level for each depth
                for depth = 1:obj.searchDepth % Iterative deepening
                    % Initialise digraph
                    obj.nodeID = 0;
                    initNode_Ego = DigraphTree.getNodeName(obj.nodeID, currentState_Ego, obj.safety_limit(1));
                    dG_initial = DigraphTree.initialise(initNode_Ego, [0, 1, 1]);
                    [dG_initial, ~, obj.nodeID] = DigraphTree.expand(dG_initial, obj.nodeID, initNode_Ego, currentStates_Other, 'InitialStates', [1, 0, 1], [1, 0, 1], obj.safety_limit(1));
                    
                    obj.depthBound = depth; % Remember depth boundary for each iteration
                    alpha_0.safety = -Inf;
                    alpha_0.liveness = -Inf;
                    beta_0.safety = Inf;
                    beta_0.liveness = Inf;
                    
                    % Best decisions iteratively
                    [bestDecision_iteration, value_iteration, dG_final] = obj.planMaxManeuver(currentState_Ego, obj.d_destination, currentStates_Other, alpha_0, beta_0, dG_initial, obj.nodeID, depth);
                    obj.safety_limit(obj.depthBound) = value_iteration.safety;
                    if ~isempty(bestDecision_iteration)
                        % Minimum Violation Planning
                        bestDecision_Ego = bestDecision_iteration;
                    end
                end
                    
                nextDecision = bestDecision_Ego(1, :);
                description_nextDecision = strsplit(nextDecision{4}, '_');
                nextState = description_nextDecision{1};
                obj.currentState = obj.states(nextState);

                if strcmp(nextState, 'ChangeLane')
                    obj.isChangingLane = true;

                    % Round: avoid very small value differences
                    % Use either 0 or 3.7
                    obj.d_destination = round(nextDecision{3}.d, 1);

                    T_LC = extractBetween(description_nextDecision{2}, '{T', '}');
                    changeLaneCmd = str2double(T_LC{1});
                end
            end
            
            drivingMode = obj.getStateInfo(obj.currentState);
            
            obj.displayNewState(obj.currentState, obj.previousState);
        end
        
        function [decisionsNext_Ego, value_max, graph] = planMaxManeuver(obj, state_Ego, d_goal, states_Other, alpha, beta, graph, parentID, depth2go)
        % Plan and decide for a safe maneuver according to specified searching depth
            
            decisionMax_Ego = []; % Decision of current depth, that maximises future value
            decisionsNext_Ego = []; % Next planned decisions (starting with most recent one)
            value_max.safety = -Inf;
            value_max.liveness = -Inf;
            
            depth2go = depth2go - 1;
            depthCurrent = obj.depthBound - depth2go;
            depthPrev = depthCurrent - 1;
            
            if depthPrev == 0
                depthPrev = 1;
            end
            parentNode = DigraphTree.getNodeName(parentID, states_Other, obj.safety_limit(depthPrev));
            maxNode = [];
            
            time_trajectory = 0:obj.Ts:obj.timeHorizon;
            
            % Decisions Ego
            decisions_Ego = obj.calculateDecisions_Ego(state_Ego, d_goal, time_trajectory);
            
            % Feasibility check
            isFeasibleDecision = [decisions_Ego{:, 2}];
            decisionsFeasible_Ego = decisions_Ego(isFeasibleDecision, :);
            
            % Decisions other vehicles
            [decisions_Other, possibleFutureStates_Other] = obj.calculateDecisions_Other(states_Other, depth2go, time_trajectory);
            
            % Safety check
            decisionsSafe_Ego = decisionsFeasible_Ego;
            for id_decision = size(decisionsFeasible_Ego, 1):-1:1 % Reverse to remove unsafe decisions without confusing idx
                TS_Ego = decisionsFeasible_Ego{id_decision, 1};
                
                safetyLevel = 0;
                for id_other = 1:size(decisions_Other, 1)
                    TS_Other = decisions_Other{id_other, 1};
                    [~, unsafeDiscreteStates] = CellChecker.isSafeTransitions(TS_Ego, TS_Other);

                    safetyLevel = min(safetyLevel, -length(unsafeDiscreteStates));
                    
                    if safetyLevel < obj.safety_limit(depthCurrent)
                        % Remove unsafer decisions
                        decisionsSafe_Ego(id_decision, :) = [];
                        break
                    end
                end
                
                if safetyLevel >= obj.safety_limit(depthCurrent) % Only consider safer/as safe decisions 
                    % Add safe decision to digraph
                    [graph, childNode, obj.nodeID] = DigraphTree.expand(graph, obj.nodeID, parentNode, ...
                        decisionsSafe_Ego{id_decision, 3}, decisionsSafe_Ego{id_decision, 4}, ...
                        [0, 1, 0], [0, 1, 0], safetyLevel);
                    
                    % Expand tree for future states of safe decisions
                    decisionSafe_Ego = decisionsSafe_Ego(id_decision, :);
                    futureState_Ego = decisionSafe_Ego{3};
                    d_futureGoal = futureState_Ego.d;
                    
                    if depth2go == 0
                        % No future decision for this depth
                        decisionsFuture_Ego = [];
                        % Combined value for liveness and safety
                        value = obj.evaluate(safetyLevel, futureState_Ego);  
                    else
                        futureStatesCombinations_Other = obj.getStateCombinations(possibleFutureStates_Other);
                        [decisionsFuture_Ego, value, graph] = obj.planMinManeuver(futureState_Ego, d_futureGoal, futureStatesCombinations_Other, alpha, beta, graph, obj.nodeID, depth2go);
                    end

                    % Max behaviour: Ego vehicle
                    if Values.isGreater(value, value_max)
                        value_max = value;
                        decisionsNext_Ego = decisionsFuture_Ego;
                        decisionMax_Ego = decisionSafe_Ego;
                        maxNode = childNode;
                        
                        alpha = Values.Max(alpha, value_max);
                        if Values.isLessEqual(beta, alpha) 
                            break
                        end
                    end    
                end
            end
            
            % Higlight best node and edge in digraph
            if ~isempty(maxNode) 
                graph = DigraphTree.changeNodeColor(graph, maxNode, [0, 1, 1]);
                graph = DigraphTree.changeEdgeColor(graph, parentNode, maxNode, [0, 1, 1]);
            end
            
            decisionsNext_Ego = [decisionMax_Ego; decisionsNext_Ego];
        end
        
        function [decisionsNext_Ego, value_min, graph] = planMinManeuver(obj, futureState_Ego, d_futureGoal, futureStatesCombinations_Other, alpha, beta, graph, parentID, depth2go)
        % For other vehicles choose the action, which is considered the most unsafe
            
            decisionsNext_Ego = [];
            value_min.safety = Inf;
            value_min.liveness = Inf;
            
            depthCurrent = obj.depthBound - depth2go;
        
            parentNode = DigraphTree.getNodeName(parentID, futureState_Ego, obj.safety_limit(depthCurrent));
            minNode = [];
        
            for id_statesOther = 1:size(futureStatesCombinations_Other, 1)
                futureStates_Other = futureStatesCombinations_Other(id_statesOther, :);
                
                % Add other vehicles' possible decisions to digraph
                [graph, childNode, obj.nodeID] = DigraphTree.expand(graph, obj.nodeID, parentNode, ...
                        futureStates_Other, ['Combination', num2str(id_statesOther)], ...
                        [1, 0, 0], [1, 0, 0], obj.safety_limit(depthCurrent));
                
                [decisionsFuture_Ego, value, graph] = obj.planMaxManeuver(futureState_Ego, d_futureGoal, futureStates_Other, alpha, beta, graph, obj.nodeID, depth2go);
                
                % Min behaviour: Other vehicles
                if isempty(decisionsFuture_Ego)
                    decisionsNext_Ego = [];
                    minNode = [];
                    value_min.safety = -Inf;
                    value_min.liveness = -Inf;
                    break % These decisions are unsafe if at least one combination of the
                          % other vehicles' possible future states might be unsafe
                elseif value.safety <= value_min.safety
                    if Values.isLess(value, value_min)
                        value_min = value;
                        decisionsNext_Ego = decisionsFuture_Ego;
                        minNode = childNode;
                        
                        beta = Values.Min(beta, value_min);
                        if Values.isLessEqual(beta, alpha)
                            break
                        end
                    end   
                end
            end
            
            % Higlight worst node and edge in digraph
            if ~isempty(minNode)
                graph = DigraphTree.changeNodeColor(graph, minNode, [1, 0, 1]);
                graph = DigraphTree.changeEdgeColor(graph, parentNode, minNode, [1, 0, 1]);
            end
        end
        
        function value = evaluate(obj, safety, state)
        % Evaluate state
           
            value.safety = safety;
            liveness = state.s + state.speed*obj.timeHorizon; % - state.d; to goback to right lane 
            value.liveness = liveness;
        end
        
        function decisions = calculateDecisions_Ego(obj, state, d_goal, time)
        % Calculate candidate trajectories (decisions) for different driving modes 
            
            % FreeDrive
            decisionsFD = obj.getDecisionsForDrivingMode(state, d_goal, 1, obj.maximumAcceleration, 'FreeDrive', time);

            % TODO: For v <= 0, LC might be unfeasible vehicle following might result in the same as EmergencyBrake
            % VehicleFollowing
            decisionsVF = obj.getDecisionsForDrivingMode(state, d_goal, obj.minimumAcceleration, 0, 'VehicleFollowing', time);

            % ChangeLane
            d_otherLane = obj.getOppositeLane(d_goal, obj.LaneWidth);
            decisionsCL = obj.getDecisionsForLaneChange(state, d_otherLane, 0, 0, time);

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
        
        function [decisions, futureStates] = calculateDecisions_Other(obj, states, depth2go, time)
        % Get possible decisions for all other vehicles 
            
            % TODO: Maybe only necessary to get occupied cells for surrounding vehicles    
            n_other = length(states); 
            n_decisions = 1; % Keep Lane; n=2: +(Change Lane)
            
            decisions = cell(n_other*n_decisions, 6); 
            futureStates = obj.preallocateStates(2*n_decisions, n_other);
            id_decision = 1;
            
            % Iteration over all other vehicles
            for id_otherVehicle = 1:n_other 
                descriptionVehicle = ['Other Vehicle ', num2str(id_otherVehicle), ': '];
                
                state = states(id_otherVehicle);
                
                % Only consider uncertainty for first depth
                if depth2go == obj.searchDepth-1
                    % Worst case states according to uncertainty
                    s_min = state.s - 3*obj.sigmaS;
                    v_min = state.speed - 3*obj.sigmaV; 
                    s_max = state.s + 3*obj.sigmaS;
                    v_max = state.speed + 3*obj.sigmaV; 
                else
                    s_min = state.s;
                    v_min = state.speed; 
                    s_max = state.s;
                    v_max = state.speed; 
                end
                
                % Decision: Keep Lane 
                [decision_KL, futureStates_KL] = obj.getDecisionForKeepLane_Other(s_min, v_min, s_max, v_max, state.d, descriptionVehicle, time);
                decisions(id_decision, :) = decision_KL;
                futureStates(1:2, id_otherVehicle) = futureStates_KL;
                id_decision = id_decision + 1;
                
                % TO ADD DECISION FOR CHANGE LANE n_decisions=2
%                 % Decision: Change Lane Static for a=0 and T=4s
%                 [decision_CL, futureStates_CL] = obj.getDecisionForChangeLane_Other(s_min, v_min, s_max, v_max, state.d, descriptionVehicle, time);
%                 if ~isempty(decision_CL)
%                     decisions(id_decision, :) = decision_CL;
%                     futureStates(3:4, id_otherVehicle) = futureStates_CL;
%                     id_decision = id_decision + 1;
%                 end
            end
            
            decisions = decisions(1:id_decision-1, :);
        end 
        
        function [decision_KL, futureStates_KL] = getDecisionForKeepLane_Other(obj, s_min, v_min, s_max, v_max, d, descriptionVehicle, time)
        % Get the decision to keep lane for other vehicle    
            
            decision_KL = cell(1, 6);
            descriptionDecision = [descriptionVehicle, 'Keep Lane'];

            % Calculate other vehicle's trajectory for a_min and a_max
            [s_trajectory_min, v_trajectory_min] = ...
                LocalTrajectoryPlanner.calculateLongitudinalTrajectory(s_min, v_min, ...
                                                                       obj.maximumVelocity, obj.minimumAcceleration, ...
                                                                       obj.trajectoryReferenceLength, obj.Ts);
            [s_trajectory_max, v_trajectory_max] = ...
                LocalTrajectoryPlanner.calculateLongitudinalTrajectory(s_max, v_max, ...
                                                                       obj.maximumVelocity, obj.maximumAcceleration, ...
                                                                       obj.trajectoryReferenceLength, obj.Ts);

            d_trajectory = d*ones(1, size(s_trajectory_min, 2));

            % Future state prediction (Min)
            futureState_min = obj.createState(s_trajectory_min(end), d, 'don''t care', v_trajectory_min(end));

             % Future state prediction (Max)
            futureState_max = obj.createState(s_trajectory_max(end), d, 'don''t care', v_trajectory_max(end));
            
            futureStates_KL = [futureState_min, futureState_max];

            TS = obj.calculateTS_Other(s_trajectory_min, d_trajectory, s_trajectory_max, d_trajectory, time);

            [decision_KL, ~] = obj.addDecision(decision_KL, TS, true, ...
                                               [futureState_min; futureState_max], ...
                                               descriptionDecision, [], [], 1);
        end
        
        function [decision_CL, futureStates_CL] = getDecisionForChangeLane_Other(obj, s_min, v_min, s_max, v_max, d, descriptionVehicle, time)
        % Get the decision to change lane for other vehicle for static maneuver with a=0, T=4s  
            
            decision_CL = cell(0, 6);
            futureStates_CL = [obj.createState([], [], [], []); obj.createState([], [], [], [])];
            descriptionDecision = [descriptionVehicle, 'Change Lane'];
                
            d_goal = obj.getOppositeLane(d, obj.LaneWidth);
            [trajectoryFrenet_min, ~, ~, ~, isFeasiblTrajectory_min] = ...
                LocalTrajectoryPlanner.calculateLaneChangingTrajectory(s_min, d, 0, 0, d_goal, ...
                                                                       v_min, obj.maximumVelocity, 0, ...
                                                                       4, obj.curvature_max, ...
                                                                       obj.RoadTrajectory, obj.timeHorizon, obj.Ts);
            % Trajectories must be feasible                                                       
            if isFeasiblTrajectory_min    
                [trajectoryFrenet_max, ~, ~, ~, isFeasiblTrajectory_max] = ...
                    LocalTrajectoryPlanner.calculateLaneChangingTrajectory(s_max, d, 0, 0, d_goal, ...
                                                                           v_max, obj.maximumVelocity, 0, ...
                                                                           4, obj.curvature_max, ...
                                                                           obj.RoadTrajectory, obj.timeHorizon, obj.Ts);
                if isFeasiblTrajectory_max
                    % Future state prediction (Min)
                    futureState_min = obj.createState(trajectoryFrenet_min(end, 1), d_goal, 'don''t care', v_min);

                    % Future state prediction (Max)
                    futureState_max = obj.createState(trajectoryFrenet_max(end, 1), d_goal, 'don''t care', v_max);
                    
                    futureStates_CL = [futureState_min, futureState_max];

                    TS = obj.calculateTS_Other(trajectoryFrenet_min(:, 1), trajectoryFrenet_min(:, 2), ...
                                               trajectoryFrenet_max(:, 1), trajectoryFrenet_max(:, 2), time);
                    
                    [decision_CL, ~] = obj.addDecision(decision_CL, TS, true, ...
                                                       [futureState_min; futureState_max], ...
                                                       descriptionDecision, [], [], 1);
                end
            end
        end
        
        function TS = calculateTS_Other(obj, s_trajectory_min, d_trajectory_min, s_trajectory_max, d_trajectory_max, time)
        % Calculate transition system (TS) for other vehicle
            
            % Calculate occupied cells for other vehicle
            occupiedCells_min = Continuous2Discrete(obj.spaceDiscretisation, s_trajectory_min, d_trajectory_min, time);
            occupiedCells_max = Continuous2Discrete(obj.spaceDiscretisation, s_trajectory_max, d_trajectory_max, time);

            % Worst case: earliest entrance times (occupiedCells_max) 
            % latest exit times (occupiedCells_min)
            occupiedCells_worstCase = zeros(size(occupiedCells_max, 1)+1, size(occupiedCells_max, 2)); % Preallocation
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

            TS = CellChecker.createTSfromCells(occupiedCells_worstCase);
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
        
        function states = preallocateStates(n_row, n_col)
        % Preallocate struct array containing the states
            
            states = struct('s', cell(n_row, n_col), 'd', cell(n_row, n_col), ...
                'orientation', cell(n_row, n_col), 'speed', cell(n_row, n_col));
        end
        
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
            
            % n_stateCombinations = \prod_{i=1}^{n_vehicles} n_states_{i}
            n_vehicles = size(states, 2);
            
            for id_vehicle = 1:n_vehicles
                % Get number of nonempty states for each vehicle
                n_states = sum(arrayfun(@(x) ~any(structfun(@isempty, x)), states(:, id_vehicle)));
                if id_vehicle == 1
                    id_combinations = 1:n_states;
                else
                    id_combinations = combvec(id_combinations, 1:n_states);
                end
            end
            
            id_combinations = id_combinations';
            
            stateCombinations = DiscretePlannerFormal.preallocateStates(size(id_combinations, 1), n_vehicles);
            for id_vehicle = 1:n_vehicles
                stateCombinations(:, id_vehicle) = states(id_combinations(:, id_vehicle), id_vehicle);
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
