classdef DiscreteTrajectory
% Discrete Trajectory containing discrete cells and entrance/exit times to of the cells
    
    properties
        cells % Discrete cells defined by row and column index      
        entranceTimes % Entrance times to discrete cells
        exitTimes % Exit times of discrete cells
    end
    
    methods
        function obj = DiscreteTrajectory(cells, entranceTimes, exitTimes)
            %DISCRETETRAJECTORY Construct an instance of this class
            obj.cells = cells;
            obj.entranceTimes = entranceTimes;
            obj.exitTimes = exitTimes;
        end
        
        function obj = append(obj, discreteTrajectory)
        % Append another discrete trajectory to this trajectory
            
            obj.cells = [obj.cells; discreteTrajectory.cells];
            obj.entranceTimes = [obj.entranceTimes; discreteTrajectory.entranceTimes];
            obj.exitTimes = [obj.exitTimes; discreteTrajectory.exitTimes];
        end
    end
end

