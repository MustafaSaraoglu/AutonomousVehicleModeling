function lineNr = getFunctionLineNr(functionName)
% Simple containers.Map to keep the line numbers to the functions used in
% files up to date to use them as a reference

    entries = {...
        % LocalTrajectoryPlanner.m
        'calculateStraightTrajectory', 245;
        'updateCurrentTrajectory', 110;
        'calculateLaneChangingTrajectory', 165;
        'divideLaneChangingTrajectory', 95;
        'calculateTrajectoryToAdd', 129;
        'getCurrentTrajectoryCartesian', 256;
        'getNextFrenetTrajectoryWaypoints', 264;
        'calculateTrajectoryError', 279
        };
    
    referenceFunctions = containers.Map(entries(:, 1)', [entries{:, 2}]);
        
    lineNr = referenceFunctions(functionName);
end