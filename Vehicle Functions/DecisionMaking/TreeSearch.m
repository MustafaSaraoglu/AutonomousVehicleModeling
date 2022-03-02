classdef TreeSearch
% Search tree to find best decision
    
    properties
        Ts % Sample time [s]
        Th % Time horizon for trajectory genereation [s]
        
        searchDepth  % Depth for search algorithm
        depthBound % Maximum depth to search for in one iteration
        safety_limit % Safety boundary for each depth/iteration
        
        DecisionGenerator % Generate decisions
    end
    
    methods
        function obj = TreeSearch(Ts, Th, DecisionGenerator)
            %TREESEARCH Construct an instance of this class
            obj.Ts = Ts;
            obj.Th = Th;
            
            obj.DecisionGenerator = DecisionGenerator;
           
            obj.searchDepth = 2;
        end
        
        function [bestDecision_total, dG_final] = iterativeMinimax(obj, state_Ego0, ...
                                                                   states_Other0, d_destination)
        % Minimax algorithm using iterative deepening
        
            dG_final = cell(obj.searchDepth, 1);
            obj.safety_limit = -Inf*ones(obj.searchDepth, 1); % Maximum safety level for each depth
            for depth = 1:obj.searchDepth % Iterative deepening
                % Initialise digraph
                ID_global = DigraphTree.getNewID(0);
                initNode = DigraphTree.getNodeName(ID_global, [state_Ego0, states_Other0], 0);
                dG_initial = DigraphTree.initialise(initNode, [1, 0, 1]);

                obj.depthBound = depth; % Remember depth boundary for each iteration
                alpha_0.safety = -Inf;
                alpha_0.liveness = -Inf;
                beta_0.safety = Inf;
                beta_0.liveness = Inf;

                % Best decisions iteratively
                [bestDecision_iteration, value_iteration, dG_iteration] = ...
                    obj.planMaxManeuver(state_Ego0, d_destination, states_Other0, alpha_0, ...
                                        beta_0, 0, dG_initial, ID_global, depth);
                dG_final{depth} = dG_iteration;
                obj.safety_limit(obj.depthBound) = value_iteration.safety;
                if ~isempty(bestDecision_iteration)
                    % Minimum Violation Planning:
                    % Take solution of previous iteration if no safe
                    % option was found in this iteration
                    bestDecision_total = bestDecision_iteration;
                end
            end
        end
        
        function [decisionsNext_Ego, value_max, graph] = planMaxManeuver(obj, state_Ego, d_goal, states_Other, alpha, beta, parentSafety, graph, parentID, depth2go)
        % Plan and decide for a safe maneuver according to specified searching depth
            
            decisionMax_Ego = []; % Decision of current depth, that maximises future value
            decisionsNext_Ego = []; % Next planned decisions (starting with most recent one)
            value_max.safety = -Inf;
            value_max.liveness = -Inf;
            
            depth2go = depth2go - 1;
            depthCurrent = obj.depthBound - depth2go;
            
            parentNode = DigraphTree.getNodeName(parentID, [state_Ego, states_Other], parentSafety);
            maxNode = [];
            
            time_trajectory = 0:obj.Ts:obj.Th;
            
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
                    [graph, childNode, childID] = DigraphTree.expand(graph, parentNode, ...
                        decision_Ego.futureState, decision_Ego.description, ...
                        [0, 1, 0], [0, 1, 0], safetyLevel);
                    
                    % Expand tree for future states of safes decisions
                    futureState_Ego = decision_Ego.futureState;
                    futureStatesCombinations_Other = State.getStateCombinations(possibleFutureStates_Other);
                    
                    if depth2go == 0
                        % No future decision for this depth
                        decisionsFuture_Ego = [];
                        % Combined value for liveness and safety
                        value = obj.evaluate(safetyLevel, futureState_Ego);  
                        % Plot decisions of other vehicles at end of the tree
                        [~, ~, graph] = obj.planMinManeuver(futureState_Ego, futureState_Ego.d, futureStatesCombinations_Other, alpha, beta, safetyLevel, graph, childID, depth2go);
                    else
                        [decisionsFuture_Ego, value, graph] = obj.planMinManeuver(futureState_Ego, futureState_Ego.d, futureStatesCombinations_Other, alpha, beta, safetyLevel, graph, childID, depth2go);
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
        
        function [decisionsNext_Ego, value_min, graph] = planMinManeuver(obj, state_Ego, d_futureGoal, stateCombinations_Other, alpha, beta, parentSafety, graph, parentID, depth2go)
        % For other vehicles choose the action, which is considered the most unsafe
            
            decisionsNext_Ego = [];
            value_min.safety = Inf;
            value_min.liveness = Inf;
            
            parentNode = DigraphTree.getNodeName(parentID, state_Ego, parentSafety);
            minNode = [];
        
            for id_statesOther = 1:size(stateCombinations_Other, 1)
                futureStates_Other = stateCombinations_Other(id_statesOther, :);
                
                % Add other vehicles' possible decisions to digraph
                [graph, childNode, childID] = DigraphTree.expand(graph, parentNode, ...
                        [state_Ego, futureStates_Other], ['Combination', num2str(id_statesOther)], ...
                        [1, 0, 0], [1, 0, 0], parentSafety);
                
                if depth2go == 0
                    % Plot at end of the tree without calling planMaxManeuver again
                    if id_statesOther == size(stateCombinations_Other, 1)
                        return
                    end
                    continue
                end
                    
                [decisionsFuture_Ego, value, graph] = obj.planMaxManeuver(state_Ego, d_futureGoal, futureStates_Other, alpha, beta, parentSafety, graph, childID, depth2go);
                
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
            liveness = state.s + state.speed*obj.Th; % - state.d; to go back to right lane 
            value.liveness = liveness;
        end
    end
end
