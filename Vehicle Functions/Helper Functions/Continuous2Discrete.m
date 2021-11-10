function discreteCells = Continuous2Discrete(spaceDiscretisationMatrix, s, d)
% Maps continuous input to discrete rectangle cell(s)
    
    % Method using matrix structure makes it twice as fast
    rows = spaceDiscretisationMatrix(1:2:end, 1:2); 
    columns = (reshape(spaceDiscretisationMatrix(2, :), 2, []))';
    idxRow = (find(s>=rows(:, 1) & s<=rows(:, 2)))';
    idxColumn = (find(d>=columns(:, 1) & d<=columns(:, 2)))';

%     % Method using cell array
%     idxRow = findIdx(spaceDiscretisation, s, 1);
%     idxColumn = findIdx(spaceDiscretisation, d, 2);
    
    if isempty(idxRow) || isempty(idxColumn)
        discreteCells = []; % s,d not in space discretisation
        return 
    end
    discreteCells = (combvec(idxRow, idxColumn))'; % [idxRow(cell1), idxColumn(cell1); idxRow(cell2), ...]
end

function idx = findIdx(spaceDiscretisation, value, dim)
% Find correct row(s)(dim == 1)/colmumn(s)(dim == 2) for given s/d value 

    for counter = 1:size(spaceDiscretisation, dim)
        switch dim
            case 1
                cell = spaceDiscretisation{counter, 1}; % s/d boundries are the same in each row/column
            case 2
                cell = spaceDiscretisation{1, counter};
        end
        
        if value < cell(dim, 1) || value > cell(dim, 2) % Incorrect row/column
            idx = [];
        elseif (counter ~= size(spaceDiscretisation, dim)) && (value == cell(dim, 2)) % s in between current and next row
            idx = [counter, counter+1];
            return
        else % s/d must be in this row/column exactly
            idx = counter;
            return
        end
    end
end