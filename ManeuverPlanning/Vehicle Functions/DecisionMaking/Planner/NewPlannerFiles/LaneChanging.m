classdef LaneChanging < NewManeuver
    %LaneChanging Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
        function decisions = getDecisionsForLaneChange(obj, state, d_goal, d_dot, d_ddot,name_DrivingMode, ManeuverPlanner)
            % Get the decisions for a lane change for different maneuver times
            
            
            dur_lower = min(2, ManeuverPlanner.Th);
            
            decisions = [];
            
            acc = ManeuverPlanner.maximumAcceleration; % Free Drive
            for durManeuver = dur_lower:1:ManeuverPlanner.Th
                description = [name_DrivingMode, '_{T', num2str(durManeuver), '}'];
                
                [trajectoryFrenet, trajectoryCartesian] = ...
                    ManeuverPlanner.NewTrajectoryGenerator.calculateLaneChangingTrajectory(state.s, state.d, ...
                    d_dot, d_ddot, d_goal, ...
                    state.speed, ...
                    ManeuverPlanner.vEgo_ref, acc, ...
                    durManeuver);
                % Future state prediction
                futureOrientation = trajectoryCartesian.orientation(end);
                futureState = State(trajectoryFrenet.s(end), trajectoryFrenet.d(end), ...
                    futureOrientation, trajectoryFrenet.velocity(end));
                
                trajectoryDiscrete = Continuous2Discrete(ManeuverPlanner.spaceDiscretisation, trajectoryFrenet);
                
                new_Maneuver = NewManeuver(obj.name,obj.id,obj.NewTrajectoryGenerator);
                
                new_Maneuver = new_Maneuver.assignTrajectory(trajectoryDiscrete, true, futureState, description, [], []);
                decisions = [decisions; new_Maneuver];
            end
        end
        
    end
    
    methods (Static)
        function nextState = apply(state,deltaT)
            %Apply Lane Changing
            speed_new = state.speed;
            s_new = state.s + speed_new*deltaT;
            if abs(state.d)<0.05 % tolerance for d value
                d_new = 3.7;
            elseif abs(state.d)-3.7<0.05
                d_new = 0;
            end
            orientation_new = state.orientation;
            
            nextState = State(s_new,d_new,orientation_new,speed_new);
        end
    end
end


