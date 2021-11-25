classdef ReachabilityAnalysis < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
% Superclass for reachability analysis

     properties(Nontunable)
        minimumAcceleration % Minimum longitudinal acceleration [m/s^2]
        maximumAcceleration % Maximum longitudinal acceleration [m/s^2]
        emergencyAcceleration % Acceleration for emergency break [m/s^2]
        
        minimumVelocity % Minimum longitudinal velocity [m/s]
        maximumVelocity % Maximum longitudinal velocity [m/s]
        
        wheelBase % Wheel base vehicle [m]
        steerAngle_max % Maximum steering angle [rad]
         
        timeHorizon % Time horizon for trajectory genereation [s]
        Ts % Sampling time for trajectory generation [s]
    end

    % Pre-computed constants
    properties(Access = protected)
        k % Number of discrete time steps according to specified time horizon
        
        A_prime % Modified A-Matrix for reachability analysis
        B_prime % Modified B-Matrix for reachability analysis
        
        numberPointsSteering  % Number of points for steering reachability
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            obj.k = obj.timeHorizon/obj.Ts - 1; % -1 because time of state(k+1) should be equal to time horizon
            
            obj.A_prime = obj.calculateAPrime(obj.k);
            obj.B_prime = obj.calculateBPrime(obj.k);
            
            % Needs to be even number TODO: why? Probably because of division by 0/'Inf*0= NaN' -> Fixed
            obj.numberPointsSteering = 2*ceil(obj.timeHorizon*rad2deg(abs(obj.steerAngle_max))); % Consider steering angle range and time horizon
        end
        
        function futureState = predictLongitudinalFutureState(obj, s_0, v_0, acceleration)
        % Predict longitudinal future state (longitudinal displacement, longitudinal velocity) 
        % according to an itnitial state and a constant acceleration
            
            initialState = [s_0; v_0];
            futureState = obj.A_prime*initialState + obj.B_prime*acceleration;
            if futureState(2) < 0 
                futureState(2) = 0;
                t_stop = -v_0/acceleration; % v(t) = 0 = acc*t + v_0 if acc = const.
                futureState(1) = s_0 + 0.5*v_0*t_stop;
            elseif futureState(2) > obj.maximumVelocity
                futureState(2) = obj.maximumVelocity;
                t_v_max = (obj.maximumVelocity - v_0)/acceleration; % v(t) = v_max = acc*t + v_0 if acc = const.
                futureState(1) = s_0 + obj.maximumVelocity*obj.timeHorizon - 0.5*(obj.maximumVelocity - v_0)*t_v_max;
            end
        end
        
        function steeringReachability = calculateSteeringReachability(obj, pose, s, v)
        % Calcuate reachability for all possible steering angles
                        
            longitudinalFutureState_min = obj.predictLongitudinalFutureState(s, v, obj.minimumAcceleration);
            longitudinalFutureState_max = obj.predictLongitudinalFutureState(s, v, obj.maximumAcceleration);
            longitudinalFutureState_emergency = obj.predictLongitudinalFutureState(s, v, obj.emergencyAcceleration);
            
            steeringAngles = linspace(-obj.steerAngle_max, obj.steerAngle_max, obj.numberPointsSteering);
            
            destinations_minBoundary = obj.predictFutureDestinations(pose, steeringAngles, longitudinalFutureState_min(1)-s);
            destinations_maxBoundary = obj.predictFutureDestinations(pose, steeringAngles, longitudinalFutureState_max(1)-s);
            destinations_emergencyBoundary = obj.predictFutureDestinations(pose, steeringAngles, longitudinalFutureState_emergency(1)-s);
           
            arcLengths =  linspace(longitudinalFutureState_min(1), longitudinalFutureState_max(1), obj.numberPointsSteering) - s;
            
            destinations_rightBoundary = obj.predictFutureDestinations(pose, -obj.steerAngle_max, arcLengths);
            destinations_leftBoundary = obj.predictFutureDestinations(pose, obj.steerAngle_max, arcLengths);
            
            steeringReachability = [destinations_minBoundary, destinations_maxBoundary, ...
                                    destinations_rightBoundary, destinations_leftBoundary, destinations_emergencyBoundary];
        end
        
        function destinations = predictFutureDestinations(obj, pose, steeringAngle, arcLength)
        % Predict points for future destinations according to steering angle and arc length
            
            % TODO: cclockwise variable instead of radii < 0
            turningRadius = obj.wheelBase./tan(steeringAngle);
            
            x_destination = pose(1) + turningRadius.*(sin(pose(3) + arcLength./turningRadius) - sin(pose(3)));
            y_destination = pose(2) + turningRadius.*(cos(pose(3)) - cos(pose(3) + arcLength./turningRadius));
            
            % TODO: Find more elegant way to work with either min/max boundary[varying steering angle; constant arc length] 
            % or right/left boundary[constant steering angle; varying arc length]
            if isinf(turningRadius) % right/left boundary[constant steering angle; varying arc length]
                x_destination = pose(1) + arcLength.*cos(pose(3)); % Correct values for infinite turning radius/0 steering angle
                y_destination = pose(2) + arcLength.*sin(pose(3));
            elseif any(isinf(turningRadius)) % min/max boundary[varying steering angle; constant arc length]
                x_destination(isinf(turningRadius)) = pose(1) + arcLength*cos(pose(3));
                y_destination(isinf(turningRadius)) = pose(2) + arcLength*sin(pose(3));
            end
            
            destinations = [x_destination', y_destination'];
        end
        

        function A_prime = calculateAPrime(obj, k)
        % Calculate modified A-Matrix for constant accelerations and k time steps

            A_prime = [1, (k+1)*obj.Ts; 
                       0,       1      ];
        end

        function B_prime = calculateBPrime(obj, k)
        % Calculate modified B-Matrix for constant accelerations and k time steps

            B_prime = [obj.Ts^2/2*sum(1:2:(2*k+1)); 
                            obj.Ts*(k+1)           ];
        end
    end
end