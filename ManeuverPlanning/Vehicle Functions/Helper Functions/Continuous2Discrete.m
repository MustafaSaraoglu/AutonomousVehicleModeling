function discreteTrajectory = Continuous2Discrete(spaceDiscretisation, frenetTrajectory)
% Maps continuous Frenet trajectory to discrete rectangle cell containing the entrance and 
% exit time to each cell (discrete trajectory)

    % Converting cell array to matrix makes it easier and faster
    spaceDiscretisationMatrix = cell2mat(spaceDiscretisation);
    
    % Rows and columns in space discretisation
    rows = spaceDiscretisationMatrix(1:2:end, 1:2); 
    columns = (reshape(spaceDiscretisationMatrix(2, :), 2, []))';
    
    % Preallocate: At most 4 cells can be occupied for one (s, d) tuple
    occupiedCells = zeros(4*frenetTrajectory.length, 2); 
    entranceTimes = -Inf*ones(4*frenetTrajectory.length, 1); 
    exitTimes = Inf*ones(4*frenetTrajectory.length, 1); 
    idx = 1;
    idx_prev = []; % Store previous index to set exit time after knowing next entrance time
    for i = 1:frenetTrajectory.length
        % Find row and column to given (s, d) tuple
        idxRow = (find(frenetTrajectory.s(i)>=rows(:, 1) & frenetTrajectory.s(i)<=rows(:, 2)))';
        idxColumn = (find(frenetTrajectory.d(i)>=columns(:, 1) & ...
                     frenetTrajectory.d(i)<=columns(:, 2)))';
        
        % All combination if in between discrete cells
        newCells = (combvec(idxRow, idxColumn))';
        
        % Skip cells that are already in discrete trajectory
        newCells = newCells(~ismember(newCells, occupiedCells, 'rows'), :); 
        if isempty(newCells)
            continue
        end
                
        % Because at most 4 cells can be occupied, idx increases dynamically
        idx_next = idx + size(newCells, 1);
        
        occupiedCells(idx:idx_next-1, :) = newCells;
        entranceTimes(idx:idx_next-1) = frenetTrajectory.time(i); % Entrance time
        if ~isempty(idx_prev)
            exitTimes(idx_prev) = frenetTrajectory.time(i); % Exit time is entrance time of next cell
        end
        idx_prev = idx:idx_next-1;
            
        idx = idx_next;
    end
    exitTimes(idx_prev) = Inf; % Exit time for last cell is unknown 
    
    discreteTrajectory = ...
        DiscreteTrajectory(occupiedCells(1:idx-1, :), entranceTimes(1:idx-1), exitTimes(1:idx-1));
end