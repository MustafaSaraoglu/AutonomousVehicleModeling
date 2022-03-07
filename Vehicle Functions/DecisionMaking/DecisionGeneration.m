classdef DecisionGeneration
% Generate decisions for ego vehicle and other vehicles
    
    properties
        RoadTrajectory % Road trajectory according to MOBATSim map format
        LaneWidth % Width of road lane [m]
        
        Ts % Sample time [s]
        Th % Time horizon for trajectory genereation [s]
        
        minimumAcceleration % Minimum longitudinal acceleration [m/s^2]
        maximumAcceleration % Maximum longitudinal acceleration [m/s^2]
        emergencyAcceleration % Acceleration for emergency break [m/s^2]
        maximumVelocity % Maximum allowed longitudinal velocity [m/s]
        vEgo_ref % Reference velocity for ego vehicle [m/s]
        vOtherVehicles_ref % Reference speed for other vehicles [m/s]
        
        sigmaS % Standard deviation for measuring other vehicles' s-coordinate [m]
        sigmaV % Standard deviation for measuring other vehicles' speeds [m/s]
        
        spaceDiscretisation % Space Discretisation
        
        TrajectoryGenerator % Generate trajectories
    end
    
    methods
        function obj = DecisionGeneration(RoadTrajectory, LaneWidth, Ts, Th, minimumAcceleration, ...
                                          maximumAcceleration, emergencyAcceleration, ...
                                          maximumVelocity, vEgo_ref, vOtherVehicles_ref, ...
                                          curvature_max, sigmaS, sigmaV, spaceDiscretisation)
            %DECISIONGENERATION Construct an instance of this class
            obj.RoadTrajectory = RoadTrajectory; 
            obj.LaneWidth = LaneWidth;

            obj.Ts = Ts;
            obj.Th = Th;

            obj.minimumAcceleration = minimumAcceleration;
            obj.maximumAcceleration = maximumAcceleration;
            obj.emergencyAcceleration = emergencyAcceleration;
            obj.maximumVelocity = maximumVelocity;
            obj.vEgo_ref = vEgo_ref;
            obj.vOtherVehicles_ref = vOtherVehicles_ref;

            obj.sigmaS = sigmaS;
            obj.sigmaV = sigmaV;

            obj.spaceDiscretisation = spaceDiscretisation;
            
            a_lateral_max = 30; % Maximum allowed lateral acceleration
            obj.TrajectoryGenerator = TrajectoryGeneration(obj.Ts, obj.Th, obj.RoadTrajectory, ...
                                                           curvature_max, a_lateral_max);
        end
        
        function decisions = calculateDecisions_Ego(obj, state, d_goal)
        % Calculate candidate trajectories (decisions) for different driving modes 
            
            % FreeDrive
            decisionsFD = obj.getDecisionsForDrivingMode(state, d_goal, 1, ...
                                                         obj.maximumAcceleration, 'FreeDrive');

            % TODO: For v <= 0, LC might be unfeasible vehicle following might result in the same 
            %as EmergencyBrake
            
            % VehicleFollowing
            decisionsVF = obj.getDecisionsForDrivingMode(state, d_goal, obj.minimumAcceleration, ...
                                                         0, 'VehicleFollowing');

            % ChangeLane
            d_otherLane = obj.getOppositeLane(d_goal, obj.LaneWidth);
            decisionsCL = obj.getDecisionsForLaneChange(state, d_otherLane, 0, 0);

            % EmergencyBrake
            decisionsEB = obj.getDecisionsForDrivingMode(state, d_goal, ...
                                                         obj.emergencyAcceleration, ...
                                                         obj.emergencyAcceleration, ...
                                                         'EmergencyBrake');
            
            % All possible decisions
            % TODO: Find order for eficient tree expansion
            decisions = [decisionsCL; decisionsEB; decisionsVF; decisionsFD];
        end
        
        function decisions = getDecisionsForDrivingMode(obj, state, d_goal, acc_lower, ...
                                                        acc_upper, name_DrivingMode)
        % Get the decisions for a driving mode for different accelerations
            
            number_decisions = length(acc_lower:1:acc_upper);

            decisions(number_decisions, 1) = Decision([], [], [], [], [], []);
            id_decision = 1;
            
            for acc = acc_lower:1:acc_upper 
                description = [name_DrivingMode, '_{acc', num2str(acc), '}'];
                
                % Trajectory prediction
                trajectoryFrenet = ...
                    obj.TrajectoryGenerator.calculateLongitudinalTrajectory(state.s, d_goal, ...
                                                                            state.speed, ...
                                                                            obj.vEgo_ref, acc, ...
                                                                            obj.Th);
                % Future state prediction
                [~, futureOrientation] = Frenet2Cartesian(trajectoryFrenet.s(end), ...
                                                          trajectoryFrenet.d(end), ...
                                                          obj.RoadTrajectory);
                futureState = State(trajectoryFrenet.s(end), trajectoryFrenet.d(end), ...
                                    futureOrientation, trajectoryFrenet.velocity(end));
                
                % Discrete trajectory
                trajectoryDiscrete = Continuous2Discrete(obj.spaceDiscretisation, trajectoryFrenet);
                
                newDecision = Decision(trajectoryDiscrete, true, futureState, description, [], []);
                [decisions, id_decision] = newDecision.addDecisionToArray(decisions, id_decision);
            end
        end
        
        function decisions = getDecisionsForLaneChange(obj, state, d_goal, d_dot, d_ddot)
        % Get the decisions for a lane change for different maneuver times
            
            dur_lower = min(2, obj.Th);
        
            number_decisions = length(dur_lower:1:obj.Th);

            decisions(number_decisions, 1) = Decision([], [], [], [], [], []);
            id_decision = 1;
            
            acc = obj.maximumAcceleration; % Free Drive
            for durManeuver = dur_lower:1:obj.Th
                description = ['ChangeLane', '_{T', num2str(durManeuver), '}'];
                
                [trajectoryFrenet, trajectoryCartesian] = ...
                    obj.TrajectoryGenerator.calculateLaneChangingTrajectory(state.s, state.d, ...
                                                                            d_dot, d_ddot, d_goal, ...
                                                                            state.speed, ...
                                                                            obj.vEgo_ref, acc, ...
                                                                            durManeuver);
                % Future state prediction
                futureOrientation = trajectoryCartesian.orientation(end);
                futureState = State(trajectoryFrenet.s(end), trajectoryFrenet.d(end), ...
                                    futureOrientation, trajectoryFrenet.velocity(end));
                
                trajectoryDiscrete = Continuous2Discrete(obj.spaceDiscretisation, trajectoryFrenet);
                
                newDecision = Decision(trajectoryDiscrete, trajectoryFrenet.isFeasible(), ...
                                       futureState, description, trajectoryFrenet, ...
                                       trajectoryCartesian);
                [decisions, id_decision] = newDecision.addDecisionToArray(decisions, id_decision);
            end  
        end
        
        function [decisions, futureStates] = calculateDecisions_Other(obj, states, currentDepth)
        % Get possible decisions for all other vehicles 
            
            % TODO: Maybe only necessary to get occupied cells for surrounding vehicles    
            n_other = length(states); 
            n_decisions = 1; % Keep Lane; n=2: +(Change Lane)
            
            decisions(n_other*n_decisions, 1) = Decision([], [], [], [], [], []); % Preallocate
            futureStates(2*n_decisions, n_other) = State([], [], [], []);
            id_decision = 1;
            
            % Iteration over all other vehicles
            for id_otherVehicle = 1:n_other 
                descriptionVehicle = ['Other Vehicle ', num2str(id_otherVehicle), ': '];
                
                state = states(id_otherVehicle);
                
                % Only consider uncertainty for first depth
                if currentDepth == 1
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
                [decision_KL, futureStates_KL] = obj.getDecisionForKeepLane_Other(s_min, v_min, ...
                                                                                  s_max, v_max, ...
                                                                                  state.d, ...
                                                                                  descriptionVehicle);
                futureStates(1:2, id_otherVehicle) = futureStates_KL;
                [decisions, id_decision] = decision_KL.addDecisionToArray(decisions, id_decision);
            end
            
            decisions = decisions(1:id_decision-1);
        end 
        
        function [decision_KL, futureStates_KL] = getDecisionForKeepLane_Other(obj, s_min, v_min, ...
                                                                               s_max, v_max, d, ...
                                                                               descriptionVehicle)
        % Get the decision to keep lane for other vehicle    
            
            descriptionDecision = [descriptionVehicle, 'Keep Lane'];

            % Calculate other vehicle's trajectory for a_min and a_max
            trajectoryFrenet_min = ...
                obj.TrajectoryGenerator.calculateLongitudinalTrajectory(s_min, d, v_min, ...
                                                                       obj.maximumVelocity, ...
                                                                       obj.minimumAcceleration, ...
                                                                       obj.Th);
            trajectoryFrenet_max = ...
                obj.TrajectoryGenerator.calculateLongitudinalTrajectory(s_max, d, v_max, ...
                                                                       obj.maximumVelocity, ...
                                                                       obj.maximumAcceleration, ...
                                                                       obj.Th);

            % Future state prediction (Min)
            futureState_min = State(trajectoryFrenet_min.s(end), d, 'don''t care', ...
                                    trajectoryFrenet_min.velocity(end));

             % Future state prediction (Max)
            futureState_max = State(trajectoryFrenet_max.s(end), d, 'don''t care', ...
                                    trajectoryFrenet_max.velocity(end));
            
            futureStates_KL = [futureState_min, futureState_max];

            discreteTrajectory = ...
                obj.calculateDiscreteTrajectory_Other(trajectoryFrenet_min, trajectoryFrenet_max);

            decision_KL = Decision(discreteTrajectory, true, [futureState_min; futureState_max], ...
                                   descriptionDecision, [], []);
        end
        
        function [decision_CL, futureStates_CL] = getDecisionForChangeLane_Other(obj, s_min, ...
                                                                                 v_min, s_max, ...
                                                                                 v_max, d, ...
                                                                                 descriptionVehicle)
        % Get the decision to change lane for other vehicle for static maneuver with a=0, T=4s  
            
            decision_CL = Decision([], [], [], [], [], []);
            futureStates_CL(2, 1) = State([], [], [], []);
            descriptionDecision = [descriptionVehicle, 'Change Lane'];
                
            d_goal = obj.getOppositeLane(d, obj.LaneWidth);
            [trajectoryFrenet_min, ~] = ...
                obj.TrajectoryGenerator.calculateLaneChangingTrajectory(s_min, d, 0, 0, d_goal, ...
                                                                       v_min, ...
                                                                       obj.maximumVelocity, 0, 4);
            % Trajectories must be feasible                                                       
            if trajectoryFrenet_min.isFeasible()    
                [trajectoryFrenet_max, ~] = ...
                    obj.TrajectoryGenerator.calculateLaneChangingTrajectory(s_max, d, 0, 0, ...
                                                                            d_goal, v_max, ...
                                                                            obj.maximumVelocity, ...
                                                                            0, 4);
                if trajectoryFrenet_max.isFeasible()
                    % Future state prediction (Min)
                    futureState_min = State(trajectoryFrenet_min.s(end), d_goal, 'don''t care', ...
                                            v_min);

                    % Future state prediction (Max)
                    futureState_max = State(trajectoryFrenet_max.s(end), d_goal, 'don''t care', ...
                                            v_max);
                    
                    futureStates_CL = [futureState_min, futureState_max];

                    discreteTrajectory = ...
                        obj.calculateDiscreteTrajectory_Other(trajectoryFrenet_min, ...
                                                              trajectoryFrenet_max);
                    
                    decision_CL = Decision(discreteTrajectory, true, ...
                                           [futureState_min; futureState_max], ...
                                           descriptionDecision, [], []);
                end
            end
        end
        
        function trajectoryDiscrete_worst = calculateDiscreteTrajectory_Other(obj, ...
                                                                              trajectoryFrenet_min, ...
                                                                              trajectoryFrenet_max)
        % Calculate worst case discrete trajectory for other vehicle
            
            % Calculate discrete trajectories for other vehicle
            trajectoryDiscrete_min = Continuous2Discrete(obj.spaceDiscretisation, ...
                                                         trajectoryFrenet_min);
            trajectoryDiscrete_max = Continuous2Discrete(obj.spaceDiscretisation, ...
                                                         trajectoryFrenet_max);

            % Assume worst case: 
            % ------------------
            % Earliest entrance times from trajectoryDiscrete_max (max. possible acceleration):
            % --> Vehicle cannot enter cell before these times
            % Latest exit times from trajectoryDiscrete_min (min. possible acceleration):
            % --> Vehicle cannot exit after these times

            % First occupied cells may not be identical for min/max case due to uncertainty
            cellDif = trajectoryDiscrete_max.cells(1, 1) - trajectoryDiscrete_min.cells(1, 1);
            if cellDif ~= 0
                trajectoryDiscrete_start = ...
                    DiscreteTrajectory(trajectoryDiscrete_min.cells(1:cellDif, :), ...
                                       trajectoryDiscrete_min.entranceTimes(1:cellDif), ...
                                       trajectoryDiscrete_min.exitTimes(1:cellDif));
                trajectoryDiscrete_worst = trajectoryDiscrete_start.append(trajectoryDiscrete_max);
            else
                trajectoryDiscrete_worst = trajectoryDiscrete_max;
            end
            
            % Find overlapping cells with min case
            [~, id_intersect_min, id_intersect_worst] = intersect(trajectoryDiscrete_min.cells, ...
                                                                trajectoryDiscrete_worst.cells, ...
                                                                'rows');
            % Assume worst case for exit time (vehicle could potentially stop at any cell)
            trajectoryDiscrete_worst.exitTimes(:) = Inf; 
            % Relax by considering known possible latest exit times from trajectoryDiscrete_min
            trajectoryDiscrete_worst.exitTimes(id_intersect_worst) = ...
                trajectoryDiscrete_min.exitTimes(id_intersect_min); 
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
    end
end

