classdef ReachabilityAnalysis < matlab.System & handle & matlab.system.mixin.Propagates & ...
        matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
% Superclass for reachability analysis

     properties(Nontunable)
        minimumAcceleration % Minimum longitudinal acceleration [m/s^2]
        maximumAcceleration % Maximum longitudinal acceleration [m/s^2]
        emergencyAcceleration % Acceleration for emergency break [m/s^2]
        
        maximumVelocity % Maximum longitudinal velocity [m/s]
        
        RoadTrajectory % Road trajectory according to MOBATSim map format
        wheelBase % Wheel base vehicle [m]
        steerAngle_max % Maximum steering angle [rad]
         
        timeHorizon % Time horizon for trajectory genereation [s]
        Ts % Sampling time for trajectory generation [s]
    end

    % Pre-computed constants
    properties(Access = protected)
        k_timeHorizon % Number of discrete time steps according to specified time horizon
        
        numberPointsSteering  % Number of points for steering reachability
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants

            % Discrete steps for one time horizon according to: timeHorizon = (k+1)*Ts
            obj.k_timeHorizon = obj.timeHorizon/obj.Ts - 1;
            
            % Number of points to calculate the boundary curves for steering reachability should 
            % increase whith larger time horizon and larger maximum steering angle,
            % since then the curves are going to get larger
            obj.numberPointsSteering = 2*ceil(obj.timeHorizon*rad2deg(abs(obj.steerAngle_max))); 
        end
        
        function steeringReachability = calculateSteeringReachability(obj, pose, v)
        % Calcuate reachability for all possible steering angles and all allowed accelerations
             
            % TODO: Transformaation might not be needed, since only delta s is necessary
            [s, ~] = Cartesian2Frenet(obj.RoadTrajectory, [pose(1), pose(2)]);
        
            % Get future longitudinal displacement, which is equal to arc length 
            % when applying a steering angle
            [s_future_min, ~] = ReachabilityAnalysis.predictLongitudinalFutureState(s, v, ...
                obj.maximumVelocity, obj.minimumAcceleration, obj.k_timeHorizon, obj.Ts);
            [s_future_max, ~] = ReachabilityAnalysis.predictLongitudinalFutureState(s, v, ...
                obj.maximumVelocity, obj.maximumAcceleration, obj.k_timeHorizon, obj.Ts);
            [s_future_emergency, ~] = ReachabilityAnalysis.predictLongitudinalFutureState(s, v, ...
                obj.maximumVelocity, obj.emergencyAcceleration, obj.k_timeHorizon, obj.Ts);
            
            possibleSteeringAngles = linspace(-obj.steerAngle_max, obj.steerAngle_max, ...
                                              obj.numberPointsSteering);
            
            % Variation of possible steering angles and constant arc length
            % according to constant acceleration for longitudinal boundary curves
            destinations_lowerBoundary = obj.predictFutureDestinations(pose, ...
                                                                       possibleSteeringAngles, ...
                                                                       s_future_min-s);
            destinations_upperBoundary = obj.predictFutureDestinations(pose, ...
                                                                       possibleSteeringAngles, ...
                                                                       s_future_max-s);
            destinations_emergencyBoundary = obj.predictFutureDestinations(pose, ...
                                                                           possibleSteeringAngles, ...
                                                                           s_future_emergency-s);
           
            possibleArcLengths =  linspace(s_future_min, s_future_max, obj.numberPointsSteering) - s;
            
            % Variation of possible arc lengths for minimum to maximum
            % acceleration and constant +/- maximum steering angle for
            % right and left boundary curves
            destinations_rightBoundary = obj.predictFutureDestinations(pose, -obj.steerAngle_max, ...
                                                                       possibleArcLengths);
            destinations_leftBoundary = obj.predictFutureDestinations(pose, obj.steerAngle_max, ...
                                                                      possibleArcLengths);
            
            steeringReachability = [destinations_lowerBoundary, destinations_upperBoundary, ...
                                    destinations_rightBoundary, destinations_leftBoundary, ...
                                    destinations_emergencyBoundary];
        end
        
        function destinations = predictFutureDestinations(obj, pose, steeringAngle, arcLength)
        % Predict points for future destinations according to steering angle and arc length
            
            % Radius might be < 0, which works programatically
            turningRadius = obj.wheelBase./tan(steeringAngle);
            
            % Limit turningAngle =  arcLength/turningRadius 
            % to get clean plot and to avoid backward motion
            turningAngleLimit = pi;
            
            isAngleOverLimit = abs(arcLength./turningRadius) > turningAngleLimit; 
            
            if any(isAngleOverLimit) 
                if length(arcLength) > 1 % Right/left boundary [constant steering angle; varying arc length]
                    arcLength(isAngleOverLimit) = ...
                        abs(ones(1, length(arcLength(isAngleOverLimit)))*turningRadius*turningAngleLimit);
                else % Longitudinal boundary [varying steering angle; constant arc length]
                    turningRadius(isAngleOverLimit) = ...
                        sign(turningRadius(isAngleOverLimit))*arcLength/turningAngleLimit;
                end
            end
            
            % Derived formula to calculate a destination position inside
            % the reachable area for given initial pose, arc length and turning radius
            x_destination = pose(1) + ...
                            turningRadius.*(sin(pose(3) + arcLength./turningRadius) - sin(pose(3)));
            y_destination = pose(2) + ...
                            turningRadius.*(cos(pose(3)) - cos(pose(3) + arcLength./turningRadius));
            
            % Correct values for infinite turning radius to avoid division by 0
            % TODO: Find more elegant way to work with either longitudinal boundary or 
            % right/left boundary
            if isinf(turningRadius) % Right/left boundary[constant steering angle; varying arc length]
                x_destination = pose(1) + arcLength.*cos(pose(3)); 
                y_destination = pose(2) + arcLength.*sin(pose(3));
            elseif any(isinf(turningRadius)) % Longitudinal boundary[varying steering angle; constant arc length]
                x_destination(isinf(turningRadius)) = pose(1) + arcLength*cos(pose(3));
                y_destination(isinf(turningRadius)) = pose(2) + arcLength*sin(pose(3));
            end
            
            destinations = [x_destination', y_destination'];
        end
    end
    
    methods(Static)
        function [s_future, v_future] = predictLongitudinalFutureState(s_0, v_0, v_max, ...
                                                                       acceleration, k, Ts)
        % Predict longitudinal future state (longitudinal displacement s_future,
        % longitudinal velocity v_future) according to an itnitial state 
        % and a constant acceleration in k+1 time steps
            
            if v_0 < 0
                v_0 = 0;
            end
            initialState = [s_0; v_0];
            
            A_matrix = ReachabilityAnalysis.calculate_A_matrix(k, Ts);
            B_matrix = ReachabilityAnalysis.calculate_B_matrix(k, Ts);
            
            % Future state according to longitudinal reachability analysis
            % for constant acceleration
            futureState = A_matrix*initialState + B_matrix*acceleration;
            
            % Maximum future velocity should be in [0, v_max],
            % Modify displacement accordingly
            if futureState(2, :) < 0 
                futureState(2, :) = 0;
                t_stop = -v_0/acceleration; % v(t) = 0 = acc*t + v_0 if acc = const.
                futureState(1, :) = s_0 + 0.5*v_0.*t_stop;
            elseif futureState(2, :) > v_max
                futureState(2) = v_max;
                if acceleration == 0
                    t_v_max = 0;
                else
                    t_v_max = (v_max - v_0)/acceleration; % v(t) = v_max = acc*t + v_0 if acc = const.
                end
                futureState(1, :) = s_0 + v_max*(k+1)*Ts - 0.5*(v_max - v_0).*t_v_max;
            end
            
            s_future = futureState(1, :);
            v_future = futureState(2, :);
        end
        
        function A_matrix = calculate_A_matrix(k, Ts)
        % Calculate modified A-Matrix for constant accelerations and k time steps

            A_matrix = [1, (k+1)*Ts; 
                       0,       1  ];
        end

        function B_matrix = calculate_B_matrix(k, Ts)
        % Calculate modified B-Matrix for constant accelerations and k time steps

            B_matrix = [Ts^2/2*sum(1:2:(2*k+1)); 
                            Ts*(k+1)           ];
        end
    end
end
