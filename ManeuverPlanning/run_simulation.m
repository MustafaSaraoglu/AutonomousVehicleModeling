function out = run_simulation(options)
% Run simulation

    arguments
        options.simTime (1,1) string = '10'; % Simulation time
    end
    
    out = sim('ManeuverPlanning', 'StartTime', '0', 'StopTime', options.simTime);
end

