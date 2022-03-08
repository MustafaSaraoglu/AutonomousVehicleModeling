classdef TrajectoryGeneration
% Generate longitudinal and lange changing trajectories
    
    properties
        Ts % Sampling time for trajectory generation [s]
        Th % Time horizon for trajectory genereation [s]
        
        RoadTrajectory % Road trajectory according to MOBATSim map format
        
        curvature_max % Maximum allowed curvature
        a_lateral_max % Maximum allowed lateral acceleration
    end
    
    methods
        function obj = TrajectoryGeneration(Ts, Th, RoadTrajectory, curvature_max, a_lateral_max)
            %TRAJECTORYGENERATION Construct an instance of this class
            obj.Ts = Ts;
            obj.Th = Th;

            obj.RoadTrajectory = RoadTrajectory;

            obj.curvature_max = curvature_max;
            obj.a_lateral_max = a_lateral_max;
        end
        
        function [trajectoryFrenet, trajectoryCartesian] = ...
                    calculateLaneChangingTrajectory(obj, s_current, d_current, d_dot_current, ...
                                                    d_ddot_current, d_destination, v_current, ...
                                                    v_ref, a_ref, durationManeuver)
        % Calculate minimum jerk trajectory for lane changing maneuver
            
            % Initial conditions
            t_i = 0; % Start at 0 (relative time frame)
                    
            d_i =         [1  t_i   t_i^2   t_i^3    t_i^4      t_i^5]; % d_initial = d_current 
            d_dot_i =     [0  1     2*t_i   3*t_i^2  4*t_i^3    5*t_i^4]; %  0
            d_ddot_i =    [0  0     2       6*t_i    12*t_i^2   20*t_i^3]; %  0

            % Final conditions
            t_f = durationManeuver; % Time to finish maneuver

            d_f =         [1  t_f   t_f^2   t_f^3    t_f^4      t_f^5]; % d_destination
            d_dot_f =     [0  1     2*t_f   3*t_f^2  4*t_f^3    5*t_f^4]; % 0
            d_ddot_f =    [0  0     2       6*t_f    12*t_f^2   20*t_f^3]; % 0

            A = [d_i; d_dot_i; d_ddot_i; d_f; d_dot_f; d_ddot_f];

            B = [d_current; d_dot_current; d_ddot_current; d_destination; 0; 0];

            X = linsolve(A,B);

            a0 = X(1);
            a1 = X(2); 
            a2 = X(3);
            a3 = X(4);
            a4 = X(5);
            a5 = X(6);
            
            % Calculate trajectory for complete maneuver
            t_maneuver = (0:obj.Ts:durationManeuver)'; 
            
            d_trajectory = a0 + a1*t_maneuver + a2*t_maneuver.^2 + a3*t_maneuver.^3 + ...
                           a4*t_maneuver.^4 + a5*t_maneuver.^5;
            d_dot_trajectory = a1 + 2*a2*t_maneuver + 3*a3*t_maneuver.^2 + 4*a4*t_maneuver.^3 + ...
                               5*a5*t_maneuver.^4;
            d_ddot_trajectory = 2*a2 + 6*a3*t_maneuver + 12*a4*t_maneuver.^2 + 20*a5*t_maneuver.^3;
            d_dddot_trajectory = 6*a3 + 24*a4*t_maneuver + 60*a5*t_maneuver.^2;
            
            % Calculate velocity profile according to reference acceleration 
            v_trajectory = obj.predictVelocityMotion(v_current, v_ref, a_ref, t_maneuver);
            
            % Avoid complex values!
            if any(v_trajectory.^2 < d_dot_trajectory.^2) 
                % Trajectory not feasible
                trajectoryFrenet = FrenetTrajectory(0, s_current, d_current, v_current, false, Inf);
                
                [position_current, roadOrientation_current] = Frenet2Cartesian(s_current, ...
                                                                               d_current, ...
                                                                               obj.RoadTrajectory);
                trajectoryCartesian = CartesianTrajectory(0, position_current(1), ...
                                                          position_current(2), ...
                                                          roadOrientation_current, v_current, ...
                                                          false, Inf);
                
                return
            end
            
            % Calculate velocity in s-direction 
            s_dot_trajectory = sqrt(v_trajectory.^2 - d_dot_trajectory.^2); 
            
            % Calculate future prediction for s-trajectory along the road
            s_trajectory = zeros(length(t_maneuver), 1); 
            s = s_current;
            for k = 1:length(t_maneuver) % Numerical integration
                s_trajectory(k) = s;
                
                % Sum up s-values knowing the velocity in s-direction and the discrete sample time
                s = s + s_dot_trajectory(k)*obj.Ts; 
            end
            
            % If the duration for the lane change is shorter than the specified time horizon, 
            % a longitudinal trajectory following the destination lane needs to be added
            if durationManeuver < obj.Th
                t_straight = (durationManeuver:obj.Ts:obj.Th)';
                [s_straight, v_straight] = obj.predictMotion(s_trajectory(end), ...
                                                             v_trajectory(end), v_ref, a_ref, ...
                                                             t_straight-durationManeuver);
                
                % Append trajectory
                length_trajectory = obj.Th/obj.Ts+1;
                t_maneuver(end:length_trajectory) = t_straight; 
                
                s_trajectory(end:length_trajectory) = s_straight;
                v_trajectory(end:length_trajectory) = v_straight;
                s_dot_trajectory(end:length_trajectory) = v_straight; % For straight traj. s_dot = v
                
                d_trajectory(end:length_trajectory) = d_destination; 
                d_dot_trajectory(end:length_trajectory) = 0;
                d_ddot_trajectory(end:length_trajectory) = 0;
                d_dddot_trajectory(end:length_trajectory) = 0;
            end
            
            [laneChangingPositionCartesian, roadOrientation] = Frenet2Cartesian(s_trajectory, ...
                                                                                d_trajectory, ...
                                                                                obj.RoadTrajectory);
            % Road orientation needs to be added
            orientation = atan2(d_dot_trajectory, s_dot_trajectory) + roadOrientation;
            
            isFeasibleTrajectory = obj.checkFeasibilityTrajectory(laneChangingPositionCartesian(:, 1), ...
                                                               laneChangingPositionCartesian(:, 2), ...
                                                               orientation, d_ddot_trajectory);
                                                        
            % Cost function according to jerk
            cost = 0.5*sum(d_dddot_trajectory.^2);
            
            trajectoryFrenet = FrenetTrajectory(t_maneuver, s_trajectory, d_trajectory, v_trajectory, ...
                                                isFeasibleTrajectory, cost);
            trajectoryCartesian = CartesianTrajectory(t_maneuver, ...
                                                      laneChangingPositionCartesian(:, 1), ...
                                                      laneChangingPositionCartesian(:, 2), ...
                                                      orientation, v_trajectory, ...
                                                      isFeasibleTrajectory, cost);
        end
        
        function trajectoryFrenet = calculateLongitudinalTrajectory(obj, s_0, d_0, v_0, v_max, ...
                                                                    acceleration, duration)
        % Calculate longitudinal trajectory according to the double integrator model 
            
            time = (0:obj.Ts:duration)'; % Start with 0 (relative time)
            [s_trajectory, v_trajectory] = obj.predictMotion(s_0, v_0, v_max, acceleration, time);
            d_trajectory = d_0*ones(length(time), 1);
            
            trajectoryFrenet = FrenetTrajectory(time, s_trajectory, d_trajectory, v_trajectory, ...
                                                true, 0);
        end
        
        function isFeasible = checkFeasibilityTrajectory(obj, x, y, orientation, a_lateral)
        % Check if trajectory is feasible accoording to lateral acceleration and trajectory curvature
            
            delta_orientation = diff(orientation);
            delta_position = sqrt((diff(x)).^2 + (diff(y)).^2);
            
            curvature = delta_orientation./delta_position;
            
            % Alternative: Limit by using centrifugal acceleration
            % a_lateral = v(1:end-1)'.^2.*curvature; 
            
            isFeasibleCurvature = all(abs(curvature) < obj.curvature_max);
            % Placeholder for a_lateral_max
            isFeasibleCentrifugalAcceleration = all(abs(a_lateral) < obj.a_lateral_max); 
            
            isFeasible = isFeasibleCurvature && isFeasibleCentrifugalAcceleration;
        end
    end
       
    methods(Static)
        function [s, v] = predictMotion(s_0, v_0, v_limit, a, time)
        % Restricted equation of motion for displacement and velocity according to double integrator
            
            s = s_0 + v_0*time + 0.5*a*time.^2;
            v = v_0 + a*time;
            if any(v > v_limit) % Not faster than limit velocity
                id_limit = find(v>v_limit, 1);
                s(v>v_limit) = s(id_limit) + v_limit*(time(v>v_limit) - time(id_limit));
                v(v>v_limit) = v_limit;
            end
            if any(v < 0) % No backward motion
                id_limit = find(v<0, 1);
                s(v<0) = s(id_limit);
                v(v<0) = 0;
            end
        end
        
        function v = predictVelocityMotion(v_0, v_limit, a, time)
         % Restricted equation of motion for velocity according to double integrator
            
            v = v_0 + a*time;
            v(v>v_limit) = v_limit; % Not faster than limit velocity
            v(v<0) = 0; % No backward motion
        end
    end
end

