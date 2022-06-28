classdef Maneuver
    %MANEUVER Class: A library or motion primitives or more complex motion patterns
    
    properties
        name
        id
    end
    
    methods
        function obj = Maneuver(name,id)
            %MANEUVER Construct an instance of this class
            %   Detailed explanation goes here
            obj.name = name;
            obj.id = id;
        end
        
        function name = getName(obj)
            name = obj.name;
        end
        
        
    end
    methods (Static)
        function maneuvers = getallActions()
            
            maneuvers = {FreeDrive('FD',1), VehicleFollowing('VF',2), EmergencyBrake('EB',3), LaneChanging('LC',4)};
            %maneuvers = {FreeDrive('FD',1), LaneChanging('LC',4)};
        
        end
        
        
        function pd = calculatePDFofOtherVehicles(states,deltaT,k)
            % Calculate the other vehicles' motion as a normal distribution centered around x1 = x0 + v*t
            mean = states.s + states.speed*deltaT*k;
            variance = deltaT*k;
            pd = makedist('Normal','mu',mean,'sigma',variance);
            
            %range = [mean-(4*variance):0.1:mean+(4*variance)];
            %plot(range,pdf(pd,range));
            
        end
    end
    methods (Abstract)
        apply(obj)
        
        
    end
    
end

