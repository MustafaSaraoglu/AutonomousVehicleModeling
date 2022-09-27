classdef EmergencyBrake < NewManeuver
    %EmergencyBrake Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
        
        function decisions = getDecisionsForDrivingMode(obj, state, d_goal, accAll, name_DrivingMode,ManeuverPlanner,~)
            % Get the decisions for a driving mode for different accelerations
            
            acc_lower = accAll(5);
            acc_upper = accAll(6);
            
            decisions = [];
                    
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
                
                new_Maneuver = NewManeuver(obj.name,obj.id,obj.NewTrajectoryGenerator);
                
                new_Maneuver = new_Maneuver.assignTrajectory(trajectoryDiscrete, true, futureState, description, [], []);
                decisions = [decisions; new_Maneuver];
            end
        end
        
        
    end
    
    methods (Static)
        function nextState = apply(state,deltaT,~)
            %Apply Emergency Brake -> Acc= -5 (According to the min dec in the Simulink model)
            minDec= -5; % Constant acc motion equations
            
            s_new = state.s + max(((state.speed*deltaT)+(0.5)*minDec*(deltaT^2)),0); % use Max (A,0) to avoid backward motion
            d_new = state.d;
            orientation_new = state.orientation;
            speed_new = max((state.speed+(deltaT*minDec)),0); % use Max (A,0) to avoid backward motion
            
            nextState = State(s_new,d_new,orientation_new,speed_new);
        end
    end
end

