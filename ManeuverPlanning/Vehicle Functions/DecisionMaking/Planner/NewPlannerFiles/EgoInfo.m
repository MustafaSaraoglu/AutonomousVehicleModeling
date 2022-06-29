classdef EgoInfo
    %EGOINFO keeps all the constants related to the ego vehicle
    %   Detailed explanation goes here
    
    properties
        Ts % Sample time [s]
        Ts_decision % Decision-making frequency
        
        timeHorizon % Time horizon for trajectory genereation [s]
        partsTimeHorizon % Divide time horizon into equal parts

        minimumAcceleration % Minimum longitudinal acceleration [m/s^2]
        maximumAcceleration % Maximum longitudinal acceleration [m/s^2]
        emergencyAcceleration % Acceleration for emergency break [m/s^2]
        maximumVelocity % Maximum allowed longitudinal velocity [m/s]
        curvature_max % Maximum allowed curvature
        a_lateral_max % Maximum allowed lateral acceleration
        
        wheelBase % Wheel base vehicle [m]
        steerAngle_max % Maximum steering angle [rad]

        vEgo_ref % Reference velocity for ego vehicle [m/s]
        
        
        
    end
    
    methods
        function obj = EgoInfo(Ts,timeHorizon,partsTimeHorizon,minimumAcceleration,maximumAcceleration,emergencyAcceleration,maximumVelocity,wheelBase,steerAngle_max,vEgo_ref)
            %EGOINFO Construct an instance of this class
            obj.Ts = Ts;
            obj.timeHorizon = timeHorizon;
            obj.partsTimeHorizon = partsTimeHorizon;
            obj.minimumAcceleration = minimumAcceleration;
            obj.maximumAcceleration = maximumAcceleration;
            obj.emergencyAcceleration = emergencyAcceleration;
            obj.maximumVelocity = maximumVelocity;
            
            obj.wheelBase = wheelBase;
            obj.steerAngle_max = steerAngle_max;
            obj.curvature_max = tan(obj.steerAngle_max)/obj.wheelBase; % Maximum allowed curvature;
            obj.a_lateral_max= 30; 
            obj.vEgo_ref = vEgo_ref;
            
            
            obj.Ts_decision = 0.1;
            
                        

        end
        

    end
end

