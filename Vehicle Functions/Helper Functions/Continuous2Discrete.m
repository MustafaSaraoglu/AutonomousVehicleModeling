function discreteCells = Continuous2Discrete(spaceDiscretisation, s, d, time)
% Maps continuous (s, d)-tuple(s) to discrete rectangle cell(s) and stores the entrance and exit time to each cell
% discreteCell: [idxRow, idxColumn, time_enter, time_exit]
    
    % Converting cell array to matrix makes it easier and faster
    spaceDiscretisationMatrix = cell2mat(spaceDiscretisation);
    
    rows = spaceDiscretisationMatrix(1:2:end, 1:2); 
    columns = (reshape(spaceDiscretisationMatrix(2, :), 2, []))';
    
    % Preallocate: At most 4 cells can be occupied for one (s, d) tuple
    discreteCells = zeros(4*length(s), 4); 
    idx = 1;
    % Store index of previous iteration to get exit time, 
    % which can first be known in the next iteration
    idx_prev = []; 
    for i = 1:length(s)
        % Find row and column to given (s, d) tuple
        idxRow = (find(s(i)>=rows(:, 1) & s(i)<=rows(:, 2)))';
        idxColumn = (find(d(i)>=columns(:, 1) & d(i)<=columns(:, 2)))';
        
        % All combination are necessary to find the corrisponding discrete cells
        cells = (combvec(idxRow, idxColumn))';
        
        % Do not duplicate cells
        cells = cells(~ismember(cells, discreteCells(:, 1:2), 'rows'), :); 
                
        % Because at most 4 cells can be occupied, idx increases dynamically
        idx_next = idx + size(cells, 1);
        
        if ~isempty(cells)
            discreteCells(idx:idx_next-1, 1:2) = cells;
            discreteCells(idx:idx_next-1, 3) = time(i); % Entrance time
            if ~isempty(idx_prev)
                discreteCells(idx_prev, 4) = time(i); % Exit time is entrance time of next cell
            end
            idx_prev = idx:idx_next-1;
        end
        idx = idx_next;
    end
    
    discreteCells = discreteCells(1:idx-1, :); % [idxRow(cell1), idxColumn(cell1); idxRow(cell2), ...]
    discreteCells(idx_prev, 4) = time(end); 
end