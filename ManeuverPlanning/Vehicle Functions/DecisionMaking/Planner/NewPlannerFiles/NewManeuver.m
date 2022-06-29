classdef NewManeuver
% Decision for maneuvers
    
    properties
        name
        id
        
        % Every maneuver has a trajectory generator
        NewTrajectoryGenerator
        
        trajectoryDiscrete % Discrete trajectory for maneuver
        isFeasible % Feasibility of maneuver
        futureState % Future state after executing maneuver
        description % Maneuver description
        trajectoryFrenet_LC % Trajectory for lane change in Frenet coordinates
        trajectoryCartesian_LC % Trajectory for lane change in Cartesian coordinates
        
        
    end
    
    methods
        function obj = NewManeuver(name,id,TG)
            %MANEUVER Construct an instance of this class
            %   Detailed explanation goes here
            obj.name = name;
            obj.id = id;
            obj.NewTrajectoryGenerator =TG;
        end
        
        function name = getName(obj)
            name = obj.name;
        end
        
        function obj = assignTrajectory(obj,trajectoryDiscrete, isFeasible, futureState, description, ...
                trajectoryFrenet_LC, trajectoryCartesian_LC)
                %DECISION Construct an instance of this class
                obj.trajectoryDiscrete = trajectoryDiscrete;
                obj.isFeasible = isFeasible;
                obj.futureState = futureState;
                obj.description = description;
                obj.trajectoryFrenet_LC = trajectoryFrenet_LC;
                obj.trajectoryCartesian_LC = trajectoryCartesian_LC;

        end
        
        % For a temporary solution
        function New_Maneuver = cast2SuperClass(obj)
            New_Maneuver = NewManeuver(obj.name, obj.id,  obj.NewTrajectoryGenerator);
            
            New_Maneuver = New_Maneuver.assignTrajectory(obj.trajectoryDiscrete, obj.isFeasible ,obj.futureState...
                ,obj.description,obj.trajectoryFrenet_LC,obj.trajectoryCartesian_LC);
        end
        
        
        

        
        function [decisions, id_next] = addDecisionToArray(obj, decisions, id)
        % Add new decisions to Decision array at the given index
            
            decisions(id) = obj;
            id_next = id + 1;
        end
        
        
    end
    methods (Static)
        function maneuvers = getallActions(TG)
            
            maneuvers = {FreeDrive('FreeDrive',1,TG), Vehicle_Following('VehicleFollowing',2,TG), EmergencyBrake('EmergencyBrake',3,TG), LaneChanging('LaneChanging',1,TG)};
            %maneuvers = {FreeDrive('FD',1), LaneChanging('LC',1)};
            
        end
        
        
    end
end

