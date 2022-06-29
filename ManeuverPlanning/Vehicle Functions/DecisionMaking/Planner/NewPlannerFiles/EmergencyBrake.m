classdef EmergencyBrake < Maneuver
    %EmergencyBrake Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
        
        function decisions = getDecisionsForDrivingMode(~, state, d_goal, accAll, name_DrivingMode,ManeuverPlanner)
            % Get the decisions for a driving mode for different accelerations
            
            acc_lower = accAll(5);
            acc_upper = accAll(6);
            
            number_decisions = length(acc_lower:1:acc_upper);
            if number_decisions == 0
                decisions = [];
                return
            end
            
            decisions(number_decisions, 1) = NewManeuver([], [], [], [], [], []);
            id_decision = 1;
            
            for acc = acc_lower:1:acc_upper
                description = [name_DrivingMode, '_{acc', num2str(acc), '}'];
                
                % Trajectory prediction
                trajectoryFrenet = ...
                    ManeuverPlanner.NewTrajectoryGenerator.calculateLongitudinalTrajectory(state.s, d_goal, ...
                    state.speed, ...
                    ManeuverPlanner.vEgo_ref, acc, ...
                    ManeuverPlanner.Th);
                % Future state prediction
                [~, futureOrientation] = Frenet2Cartesian(trajectoryFrenet.s(end), ...
                    trajectoryFrenet.d(end), ...
                    ManeuverPlanner.RoadTrajectory);
                futureState = State(trajectoryFrenet.s(end), trajectoryFrenet.d(end), ...
                    futureOrientation, trajectoryFrenet.velocity(end));
                
                % Discrete trajectory
                trajectoryDiscrete = Continuous2Discrete(ManeuverPlanner.spaceDiscretisation, trajectoryFrenet);
                
                newDecision = NewManeuver(trajectoryDiscrete, true, futureState, description, [], []);
                [decisions, id_decision] = newDecision.addDecisionToArray(decisions, id_decision);
            end
        end
        
        
    end
    
    methods (Static)
        function nextState = apply(state,deltaT)
            %Apply Emergency Brake
            speed_new = state.speed -3;
            s_new = state.s + speed_new*deltaT;
            d_new = state.d;
            orientation_new = state.orientation;
            
            nextState = State(s_new,d_new,orientation_new,speed_new);
        end
    end
end

