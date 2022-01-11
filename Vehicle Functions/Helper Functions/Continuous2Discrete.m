function discreteCells = Continuous2Discrete(spaceDiscretisationMatrix, s, d, time)
% Maps continuous input to discrete rectangle cell(s) and stores the entrance and exit time to the cell(s)
% discreteCell: [idxRow, idxColumn, time_enter, time_exit]
    
    % Method using matrix structure makes it twice as fast
    rows = spaceDiscretisationMatrix(1:2:end, 1:2); 
    columns = (reshape(spaceDiscretisationMatrix(2, :), 2, []))';
    
    discreteCells = zeros(4*length(s), 4); % Preallocate: At most 4 cells can be occupied for one (s, d) tuple
    idx = 1;
    idx_prev = [];
    for i = 1:length(s)
        idxRow = (find(s(i)>=rows(:, 1) & s(i)<=rows(:, 2)))';
        idxColumn = (find(d(i)>=columns(:, 1) & d(i)<=columns(:, 2)))';
        
        cells = (combvec(idxRow, idxColumn))';
        cells = cells(~ismember(cells, discreteCells(:, 1:2), 'rows'), :); % Do not duplicate cells
                
        idx_next = idx + size(cells, 1);
        
        if ~isempty(cells)
            discreteCells(idx:idx_next-1, 1:2) = cells;
            discreteCells(idx:idx_next-1, 3) = time(i); % Entrance time
            if ~isempty(idx_prev)
                discreteCells(idx_prev, 4) = time(i); % Exit time
            end
            idx_prev = idx:idx_next-1;
        end
        idx = idx_next;
    end
    
    discreteCells = discreteCells(1:idx-1, :); % [idxRow(cell1), idxColumn(cell1); idxRow(cell2), ...]
    discreteCells(idx_prev, 4) = time(end); 
end