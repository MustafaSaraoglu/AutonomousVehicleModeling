classdef DiscreteTrajectory
% Discrete Trajectory containing discrete cells and entrance/exit times to of the cells
    
    properties
        cells % Discrete cells defined by row and column index    
        states % Discrete states
        entranceTimes % Entrance times to discrete cells
        exitTimes % Exit times of discrete cells
    end
    
    methods
        function obj = DiscreteTrajectory(cells, entranceTimes, exitTimes)
            %DISCRETETRAJECTORY Construct an instance of this class
            obj.cells = cells;
            obj.states = DiscreteTrajectory.createDiscreteStates(cells);
            obj.entranceTimes = entranceTimes;
            obj.exitTimes = exitTimes;
        end
        
        function obj = append(obj, discreteTrajectory)
        % Append another discrete trajectory to this trajectory
            
            obj.cells = [obj.cells; discreteTrajectory.cells];
            obj.states = [obj.states; discreteTrajectory.states];
            obj.entranceTimes = [obj.entranceTimes; discreteTrajectory.entranceTimes];
            obj.exitTimes = [obj.exitTimes; discreteTrajectory.exitTimes];
        end
    end
        
    methods(Static)
        function discreteStates = createDiscreteStates(cells)
        % Create discrete states (transition from state to next state in the list) from cells
        
            X_cell = cells(:, 1);
            Y_cell = cells(:, 2);
            
            discreteStates = ["X" + string(X_cell), "Y" + string(Y_cell)];
            discreteStates = discreteStates(:, 1) + discreteStates(:, 2);
        end
        
        function [isSafe, unsafeStates] = isSafeTransitions(traj1, traj2)
        % Check whether two discrete trajectories are safe against each other
            
            isSafe = true;
            unsafeStates = [];
            
            [overlappingStates, id_overlapping_traj1, id_overlapping_traj2] = ...
                intersect(traj1.states, traj2.states);
            
            if ~isempty(overlappingStates)
                [isSafe, unsafeStates] = ...
                    DiscreteTrajectory.isSafeTemporalDiff(traj1, traj2, id_overlapping_traj1, ...
                                                          id_overlapping_traj2);
            end
        end

        function [isSafe, unsafeStates] = isSafeTemporalDiff(traj1, traj2, id_criticalStates_traj1, ...
                                                             id_criticalStates_traj2)
        % Check whether the temporal difference between relevant equal states of  
        % two discrete trajectories are safe against each other
            
            unsafeStates = [];
            % Enter after other has left ...OR... 
            isSafeEntranceTime = ...
                traj1.entranceTimes(id_criticalStates_traj1) > traj2.exitTimes(id_criticalStates_traj2); 
            % Exit before other has entered
            isSafeExitTime = ...
                traj1.exitTimes(id_criticalStates_traj1) < traj2.entranceTimes(id_criticalStates_traj2); 
            isSafe = all(isSafeEntranceTime | isSafeExitTime); 
                 
            if ~isSafe
                isUnsafeState = ~isSafeEntranceTime & ~isSafeExitTime;
                unsafeStates = traj1.states(id_criticalStates_traj1(isUnsafeState));
            end
        end
    end
end

