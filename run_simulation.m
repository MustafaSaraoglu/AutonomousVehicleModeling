function run_simulation(options)
% Run simulation

    arguments
        options.simTime (1,1) string = '45'; % Simulation time
    end
    
    sim('VehicleFollowing', 'StartTime', '0', 'StopTime', options.simTime);
end

