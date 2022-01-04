function discreteCells = Continuous2Discrete(spaceDiscretisationMatrix, s, d)
% Maps continuous input to discrete rectangle cell(s)
    
    % Method using matrix structure makes it twice as fast
    rows = spaceDiscretisationMatrix(1:2:end, 1:2); 
    columns = (reshape(spaceDiscretisationMatrix(2, :), 2, []))';
    
    discreteCells = zeros(4*length(s), 2); % Preallocate: At most 4 cells can be occupied for one (s, d) tuple
    idx = 1;
    for i = 1:length(s)
        idxRow = (find(s(i)>=rows(:, 1) & s(i)<=rows(:, 2)))';
        idxColumn = (find(d(i)>=columns(:, 1) & d(i)<=columns(:, 2)))';
        
        cells = (combvec(idxRow, idxColumn))';
        cells = cells(~ismember(cells, discreteCells, 'rows'), :); % Do not duplicate cells
                
        idx_next = idx + size(cells, 1);

        discreteCells(idx:idx_next-1, :) = cells;
        idx = idx_next;
    end
    
    discreteCells = discreteCells(1:idx-1, :); % [idxRow(cell1), idxColumn(cell1); idxRow(cell2), ...]
end