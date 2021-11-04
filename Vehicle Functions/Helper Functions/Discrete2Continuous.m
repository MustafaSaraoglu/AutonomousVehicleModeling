function continuousLimits = Discrete2Continuous(spaceDiscretisation, idxRow, idxColumn)
% Maps discrete cell to continuous space
    
    continuousLimits = spaceDiscretisation{idxRow, idxColumn}; % [s_min, s_max; d_min, d_max] 
end

