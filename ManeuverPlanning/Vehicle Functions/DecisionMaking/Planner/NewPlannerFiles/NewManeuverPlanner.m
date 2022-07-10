classdef NewManeuverPlanner
    % Generate decisions for ego vehicle and other vehicles
    
    properties
        Maneuvers
        
        EgoInfo % Temp solution
        
        RoadTrajectory % Road trajectory according to MOBATSim map format
        LaneWidth % Width of road lane [m]
        
        Ts % Sample time [s]
        Th % Time horizon for trajectory genereation [s]
        
        minimumAcceleration % Minimum longitudinal acceleration [m/s^2]
        maximumAcceleration % Maximum longitudinal acceleration [m/s^2]
        emergencyAcceleration % Acceleration for emergency break [m/s^2]
        maximumVelocity % Maximum allowed longitudinal velocity [m/s]
        curvature_max % Maximum allowed curvature
        vEgo_ref % Reference velocity for ego vehicle [m/s]
        
        
        vOtherVehicles_ref % Reference speed for other vehicles [m/s]
        
        sigmaS % Standard deviation for measuring other vehicles' s-coordinate [m]
        sigmaV % Standard deviation for measuring other vehicles' speeds [m/s]
        
        spaceDiscretisation % Space Discretisation
        
        NewTrajectoryGenerator % Generate trajectories
    end
    
    methods
        function obj = NewManeuverPlanner(Maneuvers,EgoInfo,RoadInfo,OthersInfo, NewTrajectoryGenerator)
            %DECISIONGENERATION Construct an instance of this class
            obj.Maneuvers = Maneuvers;
            
            obj.EgoInfo = EgoInfo;
            obj.Ts = EgoInfo.Ts;
            obj.Th = EgoInfo.timeHorizon;
            obj.curvature_max= EgoInfo.curvature_max;
            obj.minimumAcceleration = EgoInfo.minimumAcceleration;
            obj.maximumAcceleration = EgoInfo.maximumAcceleration;
            obj.emergencyAcceleration = EgoInfo.emergencyAcceleration;
            obj.maximumVelocity = EgoInfo.maximumVelocity;
            obj.vEgo_ref = EgoInfo.vEgo_ref;
            
            obj.vOtherVehicles_ref = OthersInfo.vOtherVehicles_ref;
            obj.sigmaS = OthersInfo.sigmaS;
            obj.sigmaV = OthersInfo.sigmaV;
            
            obj.RoadTrajectory = RoadInfo.RoadTrajectory;
            obj.LaneWidth = RoadInfo.LaneWidth;
            obj.spaceDiscretisation = RoadInfo.spaceDiscretisation;
            
            obj.NewTrajectoryGenerator = NewTrajectoryGenerator;
            
            
        end
        
        function maneuvers = calculateManeuvers_Ego(obj, state, d_goal)
            % Calculate candidate trajectories (decisions) for different driving modes
            
            accFD_min = 1;
            accVF_min = obj.minimumAcceleration;
            accVF_max = 0;
            accEB = obj.emergencyAcceleration;
            
            % For speed close to maximum speed, acceleration >= 0 results in the same final state
            % --> Free Drive as preference (instead of Vehicle Following)
            if state.speed >= obj.vEgo_ref - eps
                % FreeDrive: acc = acc_max
                accFD_min = obj.maximumAcceleration;
                % VehicleFollowing: acc < 0
                accVF_max = -1;
                % For speed close to 0, acceleration <= 0 results in same final state
                % --> Vehicle Following as preference (instead of Emergency Brake)
            elseif state.speed <= 0 + eps
                % VehicleFollowing: acc = 0
                accVF_min = 0;
                % Emergency Break: ignore
                %accEB = []; % temp solution
            end
            
            maneuvers = [];
            allAcc = [accFD_min, obj.maximumAcceleration, accVF_min, accVF_max, accEB, accEB];
            
            for maneuverType = obj.Maneuvers
                
                if isequal(maneuverType{1}.getName,'LaneChanging')
                    % ChangeLane
                    d_otherLane = obj.getOppositeLane(d_goal, obj.LaneWidth);
                    maneuver = maneuverType{1}.getDecisionsForLaneChange(state, d_otherLane, 0, 0, maneuverType{1}.getName,ManeuverPlanner);
                    
                else
                    ManeuverPlanner = obj;
                    maneuver = maneuverType{1}.getDecisionsForDrivingMode(state, d_goal, allAcc, maneuverType{1}.getName,ManeuverPlanner);
                    
                end
                maneuvers = [maneuvers; maneuver];
            end
            
        end
        
        
        function [decisions, futureStates] = calculateDecisions_Other(obj, states, currentDepth)
            % Get possible decisions for all other vehicles
            
            % TODO: Maybe only necessary to get occupied cells for surrounding vehicles
            n_other = length(states);
            n_decisions = 1; % Keep Lane; n=2: +(Change Lane)
            
            %decisions(n_other*n_decisions, 1) = NewManeuver([], [], [], [], [], []); % Preallocate
            decisions = [];
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
                v_min = max(v_min, 0); % No backward motion
                v_max = min(v_max, obj.maximumVelocity); % Not faster than maximum velocity
                
                % Decision: Keep Lane
                % Get the decision to keep lane for other vehicle
                
                descriptionDecision = [descriptionVehicle, 'Keep Lane'];
                
                % Calculate other vehicle's trajectory for a_min and a_max
                trajectoryFrenet_min = ...
                    obj.NewTrajectoryGenerator.calculateLongitudinalTrajectory(s_min, state.d, v_min, ...
                    obj.maximumVelocity, ...
                    obj.minimumAcceleration, ...
                    obj.Th);
                trajectoryFrenet_max = ...
                    obj.NewTrajectoryGenerator.calculateLongitudinalTrajectory(s_max, state.d, v_max, ...
                    obj.maximumVelocity, ...
                    obj.maximumAcceleration, ...
                    obj.Th);
                
                % Future state prediction (Min)
                futureState_min = State(trajectoryFrenet_min.s(end), state.d, 'don''t care', ...
                    trajectoryFrenet_min.velocity(end));
                
                % Future state prediction (Max)
                futureState_max = State(trajectoryFrenet_max.s(end), state.d, 'don''t care', ...
                    trajectoryFrenet_max.velocity(end));
                
                futureStates_KL = [futureState_min, futureState_max];
                
                discreteTrajectory = ...
                    obj.calculateDiscreteTrajectory_Other(trajectoryFrenet_min, trajectoryFrenet_max);
                
                Other_NewManeuver = NewManeuver("Other",1,obj.NewTrajectoryGenerator);
                decision_KL = Other_NewManeuver.assignTrajectory(discreteTrajectory, true, [futureState_min; futureState_max], ...
                    descriptionDecision, [], []);
                
                futureStates(1:2, id_otherVehicle) = futureStates_KL;
                
                decisions = [decisions; decision_KL];
            end
            
        end
        
        
        function trajectoryDiscrete_worst = calculateDiscreteTrajectory_Other(obj, trajectoryFrenet_min, trajectoryFrenet_max)
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

