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
        
        DecisionGenerator % Generate decisions
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
            
            curvature_max = tan(obj.steerAngle_max)/obj.wheelBase; % Maximum allowed curvature
            obj.DecisionGenerator = ...
                DecisionGeneration(obj.RoadTrajectory, obj.LaneWidth, obj.Ts, obj.timeHorizon, ...
                                   obj.minimumAcceleration, obj.maximumAcceleration, ...
                                   obj.emergencyAcceleration, obj.maximumVelocity, obj.vEgo_ref, ...
                                   obj.vOtherVehicles_ref, curvature_max, obj.sigmaS, ...
                                   obj.sigmaV, obj.spaceDiscretisation);
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
                currentState_Ego = State(sEgo, dEgo, orientationEgo, vEgo);
                    
                [sOther, dOther] = Cartesian2Frenet(obj.RoadTrajectory, [poseOtherVehicles(1, :)', poseOtherVehicles(2, :)']);
                
                currentStates_Other(1, size(poseOtherVehicles, 2)) = State([], [], [], []); % Preallocation
                for id_other = 1:size(poseOtherVehicles, 2)
                    currentStates_Other(id_other) = State(sOther(id_other), dOther(id_other), poseOtherVehicles(3, id_other), speedsOtherVehicles(id_other));
                end
                
                % TODO: Free Drive if no vehicle in front on right lane for faster computation?
                % [sEgo_max, ~] = ReachabilityAnalysis.predictLongitudinalFutureState(sEgo, vEgo, obj.vEgo_ref, obj.minimumAcceleration, obj.k_timeHorizon, obj.Ts);
                
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
                        % Minimum Violation Planning:
                        % Take solution of previous iteration if no safe
                        % option was found in this iteration
                        bestDecision_Ego = bestDecision_iteration;
                    end
                end
                    
                nextDecision = bestDecision_Ego(1);
                description_nextDecision = strsplit(nextDecision.description, '_');
                nextState = description_nextDecision{1};
                obj.currentState = obj.states(nextState);

                if strcmp(nextState, 'ChangeLane')
                    obj.isChangingLane = true;

                    % Round: avoid very small value differences
                    % Use either 0 or 3.7
                    obj.d_destination = round(nextDecision.futureState.d, 1);

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
            decisions_Ego = obj.DecisionGenerator.calculateDecisions_Ego(state_Ego, d_goal, time_trajectory);

            % Decisions other vehicles
            [decisions_Other, possibleFutureStates_Other] = obj.DecisionGenerator.calculateDecisions_Other(states_Other, depthCurrent, time_trajectory);
            
            % Safety check
            for id_decision = length(decisions_Ego):-1:1 % Reverse to remove unsafe decisions without confusing idx
                decision_Ego = decisions_Ego(id_decision);
                
                % Feasibility check
                if ~decision_Ego.isFeasible
                    decisions_Ego(id_decision) = [];
                    continue
                end
                
                TS_Ego = decision_Ego.TS;
                safetyLevel = 0;
                for id_other = 1:length(decisions_Other)
                    decision_Other = decisions_Other(id_other);
                    TS_Other = decision_Other.TS;
                    [~, unsafeDiscreteStates] = CellChecker.isSafeTransitions(TS_Ego, TS_Other);

                    safetyLevel = min(safetyLevel, -length(unsafeDiscreteStates));
                    
                    if safetyLevel < obj.safety_limit(depthCurrent) % Better option available previously
                        % Remove unsafer decisions
                        decisions_Ego(id_decision) = [];
                        break
                    end
                end
                
                if safetyLevel >= obj.safety_limit(depthCurrent) % Only consider safer/as safe decisions 
                    % Add safe decision to digraph
                    [graph, childNode, obj.nodeID] = DigraphTree.expand(graph, obj.nodeID, parentNode, ...
                        decision_Ego.futureState, decision_Ego.description, ...
                        [0, 1, 0], [0, 1, 0], safetyLevel);
                    
                    % Expand tree for future states of safe decisions
                    futureState_Ego = decision_Ego.futureState;
                    
                    if depth2go == 0
                        % No future decision for this depth
                        decisionsFuture_Ego = [];
                        % Combined value for liveness and safety
                        value = obj.evaluate(safetyLevel, futureState_Ego);  
                    else
                        d_futureGoal = futureState_Ego.d;
                        futureStatesCombinations_Other = State.getStateCombinations(possibleFutureStates_Other);
                        [decisionsFuture_Ego, value, graph] = obj.planMinManeuver(futureState_Ego, d_futureGoal, futureStatesCombinations_Other, alpha, beta, graph, obj.nodeID, depth2go);
                    end

                    % Max behaviour: Ego vehicle
                    if Values.isGreater(value, value_max)
                        value_max = value;
                        decisionsNext_Ego = decisionsFuture_Ego;
                        decisionMax_Ego = decision_Ego;
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
        % Evaluate safety and state
           
            value.safety = safety;
            liveness = state.s + state.speed*obj.timeHorizon; % - state.d; to goback to right lane 
            value.liveness = liveness;
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
end
