function continuousLimits = Discrete2Continuous(spaceDiscretisationMatrix, idxRow, idxColumn)
% Maps discrete cell to continuous space
    
    if (2*idxRow > size(spaceDiscretisationMatrix, 1)) || (2*idxColumn > size(spaceDiscretisationMatrix, 2))
        continuousLimits = []; % idxRow, idxColumn not in space discretisation
        return
    end
    
    % Index shift because of matrix structure
    continuousLimits = spaceDiscretisationMatrix(2*idxRow-1:2*idxRow, 2*idxColumn-1:2*idxColumn); % [s_min, s_max; d_min, d_max] 
end

