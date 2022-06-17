classdef Maneuver
    %MANEUVER Class: A library or motion primitives or more complex motion patterns
    
    properties
        name
    end
    
    methods
        function obj = Maneuver(name)
            %MANEUVER Construct an instance of this class
            %   Detailed explanation goes here
            obj.name = name;
        end
        
        function name = getName(obj)
            name = obj.name;
        end
        
        
    end
    methods (Static)
        function maneuvers = getallActions();
            
            maneuvers = {FreeDrive('FD'), EmergencyBrake('EB'), LaneChanging('LC')};
        
        end
    end
    methods (Abstract)
        apply(obj)
        
        
    end
    
end

