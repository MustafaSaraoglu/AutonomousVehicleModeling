classdef FreeDrive < NewManeuver
    %FREEDRIVE Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
                    
        function decisions = getDecisionsForDrivingMode(obj, state, d_goal, accAll, name_DrivingMode,ManeuverPlanner,~)
            % Get the decisions for a driving mode for different accelerations
            
            acc_lower = accAll(1);
            acc_upper = accAll(2);

            decisions = [];
            
            %Just to equalize with POMDP the other lane change options are
            %disabled only 2 seconds for now instead of
            % for acc = acc_lower:1:acc_upper            
            for acc = acc_upper:1:acc_upper % TODO: re-enable more options
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
            %Apply some acceleration -> acc = +2, +1 or +0 depending on the
            %current speed and the speed limit (TODO: Rework later)
            acc = 2;

            s_new = state.s + state.speed*deltaT + (0.5)*acc*deltaT^2;
            d_new = state.d;
            orientation_new = state.orientation;
            speed_new = state.speed +deltaT*acc;
            
            nextState = State(s_new,d_new,orientation_new,speed_new);
        end
    end
end

