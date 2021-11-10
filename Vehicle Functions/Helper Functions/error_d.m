function ISE_d = error_d(paramsToBeOptimized)
% Use genetic algorithm to optimize parameters
% paramsToBeOptimized = [Kp, Ki, Kd, forwardMotionGain]

	% Prepare simulation
    mdl = 'VehicleFollowing';
    assignin('base', 'Kp', paramsToBeOptimized(1));
    assignin('base', 'Ki', paramsToBeOptimized(2));
    assignin('base', 'Kd', paramsToBeOptimized(3));
    assignin('base', 'forwardMotionGain', paramsToBeOptimized(4));
    
    % Run simulation
    out = sim(mdl);

    ISE_d = max(out.ISE_d);
end

