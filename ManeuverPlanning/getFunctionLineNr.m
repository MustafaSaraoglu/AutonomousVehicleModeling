function lineNr = getFunctionLineNr(functionName)
% Simple containers.Map to keep the line numbers to the functions used in
% files up to date to use them as a reference

    entries = {...
        % VehicleFollowingInit.m
        'discretiseContinuousSpace', 105;
    
        % LocalTrajectoryPlanner.m
        'planTrajectory', 74;
        'divideLaneChangingTrajectory', 95;
        'updateCurrentTrajectory', 110;
        'calculateTrajectoryToAdd', 142;
        'calculateLaneChangingTrajectory', 178;
        'calculate_s_trajectory', 227;
        'calculateStraightTrajectory', 258;
        'getCurrentTrajectoryCartesian', 270;
        'getNextFrenetTrajectoryWaypoints', 279;
        'calculateTrajectoryError', 298;
        
        % StanleyPoseGenerator.m
        'getClosestPointOnTrajectory', 5;
        'getReferencePoseStanley', 38;
        
        % PlotDrivingScenario.m
        'plotVehicle', 23;
        'deletePreviousPlots', 85;
        'plotRoadLine', 101;
        'plotDiscreteSpace', 110;
        
        % ReachabilityAnalysis.m
        'calculateSteeringReachability', 60;
        };
    
    referenceFunctions = containers.Map(entries(:, 1)', [entries{:, 2}]);
        
    lineNr = referenceFunctions(functionName);
end