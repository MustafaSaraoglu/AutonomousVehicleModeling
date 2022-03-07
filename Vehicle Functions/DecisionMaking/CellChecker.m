classdef CellChecker
% Check if discrete trajectory (consisting of discrete discrete cells) is safe
% against other discrete trajectories by comparing the transition systems
    
    methods
        function obj = CellChecker()
            %CELLCHECKER Construct an instance of this class
        end
    end
    
    methods (Static)
        function TS = createTSfromCells(discreteTrajectory)
        % Create a transition system containing the discrete states (transition from state to next 
        % state in the list) and the entrance and exit time to each state
            
            X_cell = discreteTrajectory.cells(:, 1);
            Y_cell = discreteTrajectory.cells(:, 2);
            
            states = ["X" + string(X_cell), "Y" + string(Y_cell)];
            uStates = states(:, 1) + states(:, 2);
            
            TS.states = uStates;
            TS.entranceTime = discreteTrajectory.entranceTimes;
            TS.exitTime = discreteTrajectory.exitTimes;
        end
        
        function [isSafe, unsafeStates] = isSafeTransitions(TS1, TS2)
        % Check whether two transition systems are safe against each other
            
            isSafe = true;
            unsafeStates = [];
            
            [overlappingStates, id_overlapping_TS1, id_overlapping_TS2] = intersect(TS1.states, ...
                                                                                    TS2.states);
            
            if ~isempty(overlappingStates)
                [isSafe, unsafeStates] = CellChecker.isSafeTemporalDiff(TS1, TS2, ...
                                                                        id_overlapping_TS1, ...
                                                                        id_overlapping_TS2);
            end
        end

        function [isSafe, unsafeStates] = isSafeTemporalDiff(TS1, TS2, id_state2check_TS1, ...
                                                             id_state2check_TS2)
        % Check whether the temporal difference between relevant equal states of  
        % two transition systems are safe against each other
            
            unsafeStates = [];
            % Enter after other has left ...OR... 
            isSafeEntranceTime = ...
                TS1.entranceTime(id_state2check_TS1) > TS2.exitTime(id_state2check_TS2); 
            % Exit before other has entered
            isSafeExitTime = ...
                TS1.exitTime(id_state2check_TS1) < TS2.entranceTime(id_state2check_TS2); 
            isSafe = all(isSafeEntranceTime | isSafeExitTime); 
                 
            if ~isSafe
                isUnsafeState = ~isSafeEntranceTime & ~isSafeExitTime;
                unsafeStates = TS1.states(id_state2check_TS1(isUnsafeState));
            end
        end
    end
end

