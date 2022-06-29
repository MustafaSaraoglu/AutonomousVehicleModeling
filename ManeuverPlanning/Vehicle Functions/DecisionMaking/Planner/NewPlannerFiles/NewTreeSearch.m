classdef NewTreeSearch
% Search tree to find best decision
    
    properties
        Ts % Sample time [s]
        Th % Time horizon for trajectory genereation [s]
        
        searchDepth  % Depth for search algorithm
        depthBound % Maximum depth to search for in one iteration
        safety_limit % Safety boundary for each depth/iteration, only necessary if safety of depth d 
                     % in any case is more important than safety of depth d+1
        discount % Discount future safety levels
        
        NewManeuverPlanner % Generate decisions
    end
    
    methods
        function obj = NewTreeSearch(Ts, Th, NewManeuverPlanner)
            %TREESEARCH Construct an instance of this class
            obj.Ts = Ts;
            obj.Th = Th;
            
            obj.NewManeuverPlanner = NewManeuverPlanner;
           
            obj.searchDepth = 2;
            obj.discount = 0.1; % Sf = Sf(d=1) + 0.1*Sf(d=2) + ...  + 0.1^(d_final-1)*Sf(d_final)
                                % Decimal: Sf = Sf(d=1),Sf(d=2)Sf(d=3)...
        end
        
        function [bestDecision_total, dG_final] = iterativeMinimax(obj, state_Ego0, ...
                                                                   states_Other0, d_destination)
        % Minimax algorithm using iterative deepening
        
            dG_final = cell(obj.searchDepth, 1);
            obj.safety_limit = -Inf*ones(obj.searchDepth, 1); % Maximum safety level for each depth
            for depth = 1:obj.searchDepth % Iterative deepening
                % Initialise digraph
                ID_global = NewDigraphTree.getNewID(0);
                initNode = NewDigraphTree.getNodeName(ID_global, [state_Ego0, states_Other0]);
                dG_initial = NewDigraphTree.initialise(initNode, [1, 0, 1]);

                obj.depthBound = depth; % Remember depth boundary for each iteration
                alpha_0 = Values(-Inf, -Inf);
                beta_0 = Values(Inf, Inf);

                % Best decisions iteratively
                [bestDecision_iteration, value_iteration, dG_iteration, ~] = ...
                    obj.planMaxManeuver(state_Ego0, d_destination, states_Other0, alpha_0, ...
                                        beta_0, 0, dG_initial, ID_global, depth, true);
                dG_final{depth} = dG_iteration;
                if ~isempty(bestDecision_iteration)
                    % Minimum Violation Planning:
                    % Take solution of previous iteration if no safe
                    % option was found in this iteration
                    bestDecision_total = bestDecision_iteration;
                    obj.safety_limit(obj.depthBound) = value_iteration.safety;
                end
            end
        end
        
        function [decisionsNext_Ego, value_max, graph, parentNode] = ...
                    planMaxManeuver(obj, state_Ego, d_goal, states_Other, alpha, beta, ...
                                    parentSafety, graph, parentID, depth2go, usePruning)
        % Plan and decide for a safe maneuver according to specified searching depth
            
            decisionMax_Ego = []; % Decision of current depth, that maximises future value
            decisionsNext_Ego = []; % Next planned decisions (starting with most recent one)
            value_max = Values(-Inf, -Inf);
            
            depth2go = depth2go - 1;
            depthCurrent = obj.depthBound - depth2go;
            
            parentNode = NewDigraphTree.getNodeName(parentID, [state_Ego, states_Other]);
            maxNode = [];
            
            % Decisions Ego
            decisions_Ego = obj.NewManeuverPlanner.calculateManeuvers_Ego(state_Ego, d_goal);

            % Decisions other vehicles
            [decisions_Other, possibleFutureStates_Other] = ...
                obj.NewManeuverPlanner.calculateDecisions_Other(states_Other, depthCurrent);
            
            % Safety check
            for id_decision = length(decisions_Ego):-1:1 % Reverse to remove unsafe decisions 
                                                         % without confusing idx
                decision_Ego = decisions_Ego(id_decision);
                
                % Feasibility check
                if ~decision_Ego.isFeasible
                    decisions_Ego(id_decision) = [];
                    continue
                end
                
                safety = 0;
                for id_other = 1:length(decisions_Other)
                    decision_Other = decisions_Other(id_other);
                    [~, unsafeDiscreteStates] = ...
                        DiscreteTrajectory.isSafeTransitions(decision_Ego.trajectoryDiscrete, ...
                                                             decision_Other.trajectoryDiscrete);
                    
                    % TODO: Safety saturation [0, 9]?
                    newSafety = ...
                        parentSafety - obj.discount^(depthCurrent-1)*length(unsafeDiscreteStates);
                    safety = min(safety, newSafety);
                    
                    if usePruning && safety < obj.safety_limit(depthCurrent) % Better option 
                                                                             % available previously
                        % Remove unsafer decisions
                        decisions_Ego(id_decision) = [];
                        break
                    end
                end
                
                if ~usePruning || safety >= obj.safety_limit(depthCurrent) % Only consider safer/as 
                                                                           % safe decisions 
                    % Add safe decision to digraph
                    [graph, childNode, childID] = NewDigraphTree.expand(graph, parentNode, ...
                        decision_Ego.futureState, decision_Ego.description, ...
                        [0, 1, 0], [0, 1, 0]);
                    
                    % Expand tree for future states of safes decisions
                    futureState_Ego = decision_Ego.futureState;
                    futureStatesCombinations_Other = ...
                        State.getStateCombinations(possibleFutureStates_Other);
                    
                    if depth2go == 0
                        % No future decision for this depth
                        decisionsFuture_Ego = [];
                        % Combined value for liveness and safety
                        value = obj.evaluate(safety, futureState_Ego); 
                        
                        % Plot decisions of other vehicles at end of the tree
                        for id_statesOther = 1:size(futureStatesCombinations_Other, 1)
                            futureStates_Other = futureStatesCombinations_Other(id_statesOther, :);

                            % Add other vehicles' possible decisions to digraph
                            [graph, leafNode, ~] = ...
                                NewDigraphTree.expand(graph, childNode, ...
                                [state_Ego, futureStates_Other], ...
                                ['Combination', num2str(id_statesOther)], [1, 0, 0], [1, 0, 0]);
                            [graph, ~] = NewDigraphTree.updateNodeValue(graph, leafNode, value);
                        end
                        
                        % Update value in graph
                        [graph, childNode] = NewDigraphTree.updateNodeValue(graph, childNode, value);
                    else
                        [decisionsFuture_Ego, value, graph, childNode] = ...
                            obj.planMinManeuver(futureState_Ego, futureState_Ego.d, ...
                                                futureStatesCombinations_Other, alpha, beta, ...
                                                safety, graph, childID, depth2go, usePruning);
                    end

                    % Max behaviour: Ego vehicle
                    if Values.isGreater(value, value_max)
                        value_max = value;
                        decisionsNext_Ego = decisionsFuture_Ego;
                        decisionMax_Ego = decision_Ego;
                        maxNode = childNode;
                        
                        alpha = Values.Max(alpha, value_max);
                        if usePruning && Values.isLessEqual(beta, alpha) 
                            break
                        end
                    end    
                end
            end
            
            if ~isempty(maxNode) 
                [graph, parentNode] = ...
                    NewDigraphTree.backpropagate(graph, maxNode, parentNode, value_max, [0, 1, 1]);
            end
            
            decisionsNext_Ego = [decisionMax_Ego; decisionsNext_Ego];
        end
        
        function [decisionsNext_Ego, value_min, graph, parentNode] = ...
                planMinManeuver(obj, state_Ego, d_futureGoal, stateCombinations_Other, alpha, ...
                                beta, parentSafety, graph, parentID, depth2go, usePruning)
        % For other vehicles choose the action, which is considered the most unsafe
            
            decisionsNext_Ego = [];
            value_min = Values(Inf, Inf);
            
            parentNode = NewDigraphTree.getNodeName(parentID, state_Ego);
            minNode = [];
        
            for id_statesOther = 1:size(stateCombinations_Other, 1)
                futureStates_Other = stateCombinations_Other(id_statesOther, :);
                
                % Add other vehicles' possible decisions to digraph
                [graph, ~, childID] = NewDigraphTree.expand(graph, parentNode, ...
                        [state_Ego, futureStates_Other], ['Combination', num2str(id_statesOther)], ...
                        [1, 0, 0], [1, 0, 0]);
                    
                [decisionsFuture_Ego, value, graph, childNode] = ...
                    obj.planMaxManeuver(state_Ego, d_futureGoal, futureStates_Other, alpha, ...
                                        beta, parentSafety, graph, childID, depth2go, usePruning);
                
                % Min behaviour: Other vehicles
                if isempty(decisionsFuture_Ego)
                    decisionsNext_Ego = [];
                    minNode = [];
                    value_min = Values(-Inf, -Inf);
                    break % These decisions are unsafe if at least one combination of the
                          % other vehicles' possible future states might be unsafe
                end
                
                if Values.isLess(value, value_min)
                    value_min = value;
                    decisionsNext_Ego = decisionsFuture_Ego;
                    minNode = childNode;

                    beta = Values.Min(beta, value_min);
                    if usePruning && Values.isLessEqual(beta, alpha)
                        break
                    end
                end   
            end
            
            if ~isempty(minNode)
                [graph, parentNode] = ...
                    NewDigraphTree.backpropagate(graph, minNode, parentNode, value_min, [1, 0, 1]);
            end
        end
        
        function value = evaluate(obj, safety, state)
        % Evaluate safety and state
           
            liveness = round(state.s, 1) + round(state.speed, 1)*obj.Th - abs(round(state.d, 1));
            value = Values(safety, liveness);
        end
    end
end

