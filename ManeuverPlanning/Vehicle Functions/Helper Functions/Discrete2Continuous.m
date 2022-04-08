function continuousLimits = Discrete2Continuous(spaceDiscretisation, idxRow, idxColumn)
% Maps discrete cell to continuous space
    
    if (idxRow > size(spaceDiscretisation, 1)) || (idxColumn > size(spaceDiscretisation, 2))
        continuousLimits = []; % cell(idxRow, idxColumn) not in space discretisation
        return
    end
    
    continuousLimits = spaceDiscretisation{idxRow, idxColumn}; % [s_min, s_max; d_min, d_max] 
end

