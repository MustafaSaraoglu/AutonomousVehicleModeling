function ISE_d = error_d(paramsToBeOptimized)
% Use genetic algorithm to optimize parameters
% paramsToBeOptimized = [Kp, Ki, Kd, forwardMotionGain] / only Kp

	% Prepare simulation
    mdl = 'ManeuverPlanning';
    
    %% With PID
%     assignin('base', 'Kp', paramsToBeOptimized(1));
%     assignin('base', 'Ki', paramsToBeOptimized(2));
%     assignin('base', 'Kd', paramsToBeOptimized(3));
%     assignin('base', 'forwardMotionGain', paramsToBeOptimized(4));
    %% Without PID
    assignin('base', 'forwardMotionGain', paramsToBeOptimized);
    %%
    
    % Run simulation
    out = sim(mdl);

    ISE_d = max(out.ISE_d);
end

