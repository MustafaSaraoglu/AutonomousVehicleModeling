classdef RoadInfo
    %ROADINFO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        RoadTrajectory % Road trajectory according to MOBATSim map format
        LaneWidth % Width of road lane [m]
        spaceDiscretisation
    end
    
    methods
        function obj = RoadInfo(RoadTrajectory, LaneWidth, spaceDiscretisation)
            %ROADINFO Construct an instance of this class
            %   Detailed explanation goes here
            obj.RoadTrajectory = RoadTrajectory;
            obj.LaneWidth = LaneWidth;
            obj.spaceDiscretisation = spaceDiscretisation;
        end
        
    end
end

