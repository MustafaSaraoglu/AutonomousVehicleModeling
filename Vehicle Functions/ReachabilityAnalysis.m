classdef ReachabilityAnalysis < matlab.System & handle & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon
% Superclass for reachability analysis

     properties(Nontunable)
        minimumAcceleration % Minimum longitudinal acceleration
        maximumAcceleration % Maximum longitudinal acceleration
        
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
    
    methods(Static)
        function destinations = predictFutureDestinations(pose, turningRadius, arcLength)
        % Predict points for future destinations according to truning radius and arc length
            
            arcAngle = arcLength./turningRadius;
        
            x_destination = pose(1) + turningRadius.*(sin(pose(3) + arcAngle) - sin(pose(3)));
            y_destination = pose(2) + turningRadius.*(cos(pose(3)) - cos(pose(3) + arcAngle));
            
            destinations = [x_destination', y_destination'];
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            obj.k = obj.timeHorizon/obj.Ts - 1; % -1 because state(k+1) should be equal to time horizon
            
            obj.A_prime = obj.calculateAPrime(obj.k);
            obj.B_prime = obj.calculateBPrime(obj.k);
            
            obj.numberPointsSteering = ceil(rad2deg(2*abs(obj.steerAngle_max))) + 1; % At least one point every degree
        end
        
        function [futureState_min, futureState_max] = predictLongitudinalFutureState(obj, s_0, v_0)
        % Predict future state (longitudinal displacement, longitudinal velocity) according to 
        % longitudinal reachability analysis for a constant minimum and maximum acceleration
            
            initialState = [s_0; v_0];
        
            futureState_min = obj.A_prime*initialState + obj.B_prime*obj.minimumAcceleration;
            if futureState_min(2) < 0
                % Account for the -speed error in the prediction and
                % correct the position value and set predicted speed to 0
                
                futureState_min(2) = 0;
                t_stop = -v_0/obj.minimumAcceleration; % v(t) = 0 = acc*t + v_0 if acc = const.
                futureState_min(1) = s_0 + v_0*t_stop/2;
            end
            
            futureState_max = obj.A_prime*initialState + obj.B_prime*obj.maximumAcceleration;
        end
        
        function steeringReachability = calculateSteeringReachability(obj, pose, s, v)
        % Calcuate reachability for all possible steering angles
        
            steeringAngles = linspace(-obj.steerAngle_max, obj.steerAngle_max, obj.numberPointsSteering);
            turningRadii = obj.wheelBase./tan(steeringAngles);
            [longitudinalFutureState_min, longitudinalFutureState_max] = obj.predictLongitudinalFutureState(s, v);
            
            destinations_minBoundary = obj.predictFutureDestinations(pose, turningRadii, longitudinalFutureState_min(1)-s);
            destinations_maxBoundary = obj.predictFutureDestinations(pose, turningRadii, longitudinalFutureState_max(1)-s);
           
            arcLengths =  linspace(longitudinalFutureState_min(1), longitudinalFutureState_max(1), obj.numberPointsSteering) - s;
            turningRadius_minimum = obj.wheelBase./tan(obj.steerAngle_max);
            
            destinations_rightBoundary = obj.predictFutureDestinations(pose, -turningRadius_minimum, arcLengths);
            destinations_leftBoundary = obj.predictFutureDestinations(pose, turningRadius_minimum, arcLengths);
            
            steeringReachability = [destinations_minBoundary, destinations_maxBoundary, ...
                                    destinations_rightBoundary, destinations_leftBoundary];
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
