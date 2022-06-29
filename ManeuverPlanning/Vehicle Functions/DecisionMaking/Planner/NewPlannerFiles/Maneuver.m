classdef Maneuver
    %MANEUVER Class: A library or motion primitives or more complex motion patterns
    
    properties
        name
        id
        
        % Every maneuver has a trajectory generator
        NewTrajectoryGenerator
        
    end
    
    methods
        function obj = Maneuver(name,id,TG)
            %MANEUVER Construct an instance of this class
            %   Detailed explanation goes here
            obj.name = name;
            obj.id = id;
            obj.NewTrajectoryGenerator =TG;
        end
        

        function name = getName(obj)
            name = obj.name;
        end
        
        
    end
    methods (Static)
        function maneuvers = getallActions(TG)
            
            maneuvers = {FreeDrive('FreeDrive',1,TG), Vehicle_Following('VehicleFollowing',2,TG), EmergencyBrake('EmergencyBrake',3,TG), LaneChanging('LaneChanging',1,TG)};
            %maneuvers = {FreeDrive('FD',1), LaneChanging('LC',1)};
        
        end
        
        
    end
    methods (Abstract)
        apply(obj)
        
        
    end
    
end

