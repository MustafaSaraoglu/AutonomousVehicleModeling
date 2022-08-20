%% Run automated tests with varying initial conditions
% number of other vehicles 
num_other_vehicles = 4;
% simulation time
t_sim = 30;
% number of iteration
num_iter = 50;
if num_other_vehicles == 4
    % 4 other vehicles
    ego_s0 = 140;
    ego_d0 = 0;
    ego_v0 = 15;
    ego_vref = 25;
    
    other1_s0 = 120;
    other2_s0 = 140;
    other3_s0 = 240;
    other4_s0 = 0;
    
    other1_d0 = 0;
    other2_d0 = 3.7;
    other3_d0 = 0;
    other4_d0 = 3.7;
    
    other1_v0 = 20;
    other2_v0 = 25;
    other3_v0 = 20;
    other4_v0 = 15;
    
    other1_vref = 15;
    other2_vref = 10;
    other3_vref = 20;
    other4_vref = 30;
    
elseif num_other_vehicles == 3
    % 3 other vehicles
    ego_s0 = 140;
    ego_d0 = 0;
    ego_v0 = 15;
    ego_vref = 15;

    other1_s0 = 185;
    other2_s0 = 0;
    other3_s0 = 200;

    other1_d0 = 0;
    other2_d0 = 3.7;
    other3_d0 = 3.7;

    other1_v0 = 13;
    other2_v0 = 25;
    other3_v0 = 14;

    other1_vref = 13;
    other2_vref = 25;
    other3_vref = 14;
    
elseif num_other_vehicles == 2
    % 2 other vehicles
    ego_s0 = 50;
    ego_d0 = 0;
    ego_v0 = 15;
    ego_vref = 15;

    other1_s0 = 62;
    other2_s0 = 50;

    other1_d0 = 0;
    other2_d0 = 3.7;

    other1_v0 = 8;
    other2_v0 = 30;

    other1_vref = 8;
    other2_vref = 0;
    
    elseif num_other_vehicles == 1
    % 1 other vehicle
    ego_s0 = 140;
    ego_d0 = 0;
    ego_v0 = 15;
    ego_vref = 15;

    other1_s0 = 185;

    other1_d0 = 0;

    other1_v0 = 13;

    other1_vref = 13;

end
%% MATLAB table
time_step = 0.05;
tao = 5; 
% define the MATLAB table, scn0 is for specifying the header
T = table;
scn_name = 'scn';
space_length = length(num2str(num_iter));
scn_name(4:4+space_length) = ' ';
T.scenario = scn_name;
T.reachedDis_RB = 0;
T.minTTC_RB = 0;
T.meanTTC_RB = 0;
T.TET_RB = 0;
T.executionTime_RB = 0;
T.reachedDis_POMDP = 0;
T.minTTC_POMDP = 0;
T.meanTTC_POMDP = 0;
T.TET_POMDP = 0;
T.executionTime_POMDP = 0;
T.reachedDis_MINIMAX = 0;
T.minTTC_MINIMAX = 0;
T.meanTTC_MINIMAX = 0;
T.TET_MINIMAX = 0;
T.executionTime_MINIMAX = 0;
T.numOtherVehicles = 0;
T1 = table;
%% simulation
if num_other_vehicles == 4
    for k =1:num_iter % For 4 vehicles
        
        disp(['Experiment no: ',num2str(k),' out of ', num2str(num_iter)]);
        
        disp(['Simulation for -> Ego_s0: ',num2str(ego_s0), '/ Ego_v0: ', num2str(ego_v0),...
            '/ Ego_vref: ', num2str(ego_vref)]);
        
        disp(['Simulation for -> other1_s0: ',num2str(other1_s0), '/ other1_v0: ', num2str(other1_v0),...
            '/ other1_vref: ', num2str(other1_vref)]);
        
        disp(['Simulation for -> other2_s0: ',num2str(other2_s0), '/ other2_v0: ', num2str(other2_v0),...
            '/ other2_vref: ', num2str(other2_vref)]);
        
        disp(['Simulation for -> other3_s0: ',num2str(other3_s0), '/ other3_v0: ', num2str(other3_v0),...
            '/ other3_vref: ', num2str(other3_vref)]);
        
        disp(['Simulation for -> other4_s0: ',num2str(other4_s0), '/ other4_v0: ', num2str(other4_v0),...
            '/ other4_vref: ', num2str(other4_vref)]);
        
        disp('------------------------------------------')
        disp('Rule-based Planner')
        tic;
        
        
        scn = ['scn', num2str(k)];
        space_length = length(scn_name) - length(scn); 
        scn(length(scn)+1:length(scn_name)) = ' ';
        T1.scenario = scn;
        try
            prepare_simulation('n_other', num_other_vehicles, 's_0', [ego_s0,other1_s0, other2_s0, other3_s0, other4_s0],...
                'd_0', [ego_d0, other1_d0, other2_d0, other3_d0, other4_d0], ...
                'v_0', [ego_v0, other1_v0, other2_v0, other3_v0, other4_v0], ...
                'v_ref', [ego_vref, other1_vref, other2_vref, other3_vref, other4_vref],...
                'planner', 'MANUAL');
            out = run_simulation('simTime', t_sim);
            disp(['s: ',num2str(out.s(end))]);
            disp(['d: ',num2str(out.d(end))]);
            % save the out.s and out.d for plot
            s_RB = out.s;
            d_RB = out.d;
            % get metrics
            [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
            % write the metrics and execution time of Rule-Based to table T1
            
            T1.reachedDis_RB = out.s(end);
            T1.minTTC_RB = min_TTC;
            T1.meanTTC_RB = meanTTC;
            T1.TET_RB = TET;
            T1.executionTime_RB = toc;
        catch
            % if bug, then set all values of Rule-Based to -1  
            T1.reachedDis_RB = -1;
            T1.minTTC_RB = -1;
            T1.meanTTC_RB = -1;
            T1.TET_RB = -1;
            T1.executionTime_RB = -1;
            toc;
            disp('***********scenario failed!!!*******')
        end    
        
        disp('------------------------------------------')
        disp('POMDP Planner')
        tic;
        try
            prepare_simulation('n_other', num_other_vehicles, 's_0', [ego_s0,other1_s0, other2_s0, other3_s0, other4_s0],...
                'd_0', [ego_d0, other1_d0, other2_d0, other3_d0, other4_d0], ...
                'v_0', [ego_v0, other1_v0, other2_v0, other3_v0, other4_v0], ...
                'v_ref', [ego_vref, other1_vref, other2_vref, other3_vref, other4_vref],...
                'planner', 'POMDP');
            out = run_simulation('simTime', t_sim);
            disp(['s: ',num2str(out.s(end))]);
            disp(['d: ',num2str(out.d(end))]);
            % save the out.s and out.d for plot
            s_POMDP = out.s;
            d_POMDP = out.d;
            % get metrics
            [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
            % write the metrics and execution time of POMDP to table T1
            T1.reachedDis_POMDP = out.s(end);
            T1.minTTC_POMDP = min_TTC;
            T1.meanTTC_POMDP = meanTTC;
            T1.TET_POMDP = TET;
            T1.executionTime_POMDP = toc;
        catch
            % if bug, then set all values of POMDP to -1  
            T1.reachedDis_POMDP = -1;
            T1.minTTC_POMDP = -1;
            T1.meanTTC_POMDP = -1;
            T1.TET_POMDP = -1;
            T1.executionTime_POMDP = -1;
            toc;
            disp('***********scenario failed!!!*******')
        end
               
        disp('------------------------------------------')
        disp('Minimax Planner')
        tic;
        try
            prepare_simulation('n_other', num_other_vehicles, 's_0', [ego_s0,other1_s0, other2_s0, other3_s0, other4_s0],...
                'd_0', [ego_d0, other1_d0, other2_d0, other3_d0, other4_d0], ...
                'v_0', [ego_v0, other1_v0, other2_v0, other3_v0, other4_v0], ...
                'v_ref', [ego_vref, other1_vref, other2_vref, other3_vref, other4_vref],...
                'planner', 'NEWPLANNER');
            out = run_simulation('simTime', t_sim);
            disp(['s: ',num2str(out.s(end))]);
            disp(['d: ',num2str(out.d(end))]);
            % save the out.s and out.d for plot
            s_MINIMAX = out.s;
            d_MINIMAX = out.d;
            % get metrics
            [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
            % write the metrics and execution time of MINIMAX to table T1
            T1.reachedDis_MINIMAX = out.s(end);
            T1.minTTC_MINIMAX = min_TTC;
            T1.meanTTC_MINIMAX = meanTTC;
            T1.TET_MINIMAX = TET;
            T1.executionTime_MINIMAX = toc;
            T1.numOtherVehicles = 4;
            
        catch
            % if bug set all values of MINIMAX to -1 except number of other vehicles  
            T1.reachedDis_MINIMAX = -1;
            T1.minTTC_MINIMAX = -1;
            T1.meanTTC_MINIMAX = -1;
            T1.TET_MINIMAX = -1;
            T1.executionTime_MINIMAX = -1;
            T1.numOtherVehicles = 4;
            toc;
            disp('***********scenario failed!!!*******')
                     
        end
        % concatenate two tables   
        T = [T;T1];
        disp('------------------------------------------')
        % plot the figures and save
        plotScn(k, s_RB, d_RB, s_POMDP, d_POMDP, s_MINIMAX, d_MINIMAX);
        
       %% Change initial parameters randomly
        other1_s0 = 400*rand(1,1);
        other2_s0 = 400*rand(1,1);
        other3_s0 = 400*rand(1,1);
        other4_s0 = 400*rand(1,1);
        
        other1_v0 = 30*rand(1,1);
        other2_v0 = 30*rand(1,1);
        other3_v0 = 30*rand(1,1);
        other4_v0 = 30*rand(1,1);
        
        other1_vref = 30*rand(1,1);
        other2_vref = 30*rand(1,1);
        other3_vref = 30*rand(1,1);
        other4_vref = 30*rand(1,1);
        
        ego_s0 = 300*rand(1,1);
        ego_v0 = 30*rand(1,1);
        ego_vref = 30*rand(1,1);
%         other1_s0 = 120 + (-20 + (20+20)*rand(1,1));
%         other2_s0 = 140 + (-20 + (20+20)*rand(1,1));
%         other3_s0 = 240 + (-20 + (20+20)*rand(1,1));
%         other4_s0 = 0 + (-20 + (20+20)*rand(1,1));
%         
%         other1_v0 = 20 + (-3 + (3+3)*rand(1,1));
%         other2_v0 = 25 + (-3 + (3+3)*rand(1,1));
%         other3_v0 = 20 + (-3 + (3+3)*rand(1,1));
%         other4_v0 = 15 + (-3 + (3+3)*rand(1,1));
%         
%         other1_vref = 15 + (-5 + (5+5)*rand(1,1));
%         other2_vref = 10 + (-5 + (5+5)*rand(1,1));
%         other3_vref = 20 + (-5 + (5+5)*rand(1,1));
%         other4_vref = 30 + (-5 + (5+5)*rand(1,1));
%         
%         ego_s0 = 140 + (-20 + (20+20)*rand(1,1));
%         ego_v0 = 15 + (-3 + (3+3)*rand(1,1));
%         ego_vref = 25 + (-5 + (5+5)*rand(1,1));
        writetable(T, 'testResults.csv');
    end
elseif num_other_vehicles == 3
    for k =1:num_iter % For three other vehicles

        disp(['Experiment no: ',num2str(k),' out of ', num2str(num_iter)]);
        
        disp(['Simulation for -> Ego_s0: ',num2str(ego_s0), '/ Ego_v0: ', num2str(ego_v0),...
            '/ Ego_vref: ', num2str(ego_vref)]);

        disp(['Simulation for -> other1_s0: ',num2str(other1_s0), '/ other1_v0: ', num2str(other1_v0),...
            '/ other1_vref: ', num2str(other1_vref)]);

        disp(['Simulation for -> other2_s0: ',num2str(other2_s0), '/ other2_v0: ', num2str(other2_v0),...
            '/ other2_vref: ', num2str(other2_vref)]);

        disp(['Simulation for -> other3_s0: ',num2str(other3_s0), '/ other3_v0: ', num2str(other3_v0),...
            '/ other3_vref: ', num2str(other3_vref)]);


        disp('------------------------------------------')
        disp('Rule-based Planner')
        tic;
        scn = ['scn', num2str(k)];
        space_length = length(scn_name) - length(scn); 
        scn(length(scn)+1:length(scn_name)) = ' ';
        T1.scenario = scn;
        try
            prepare_simulation('n_other', 3, 's_0', [ego_s0,other1_s0, other2_s0, other3_s0],...
                'd_0', [ego_d0, other1_d0, other2_d0, other3_d0], ...
                'v_0', [ego_v0, other1_v0, other2_v0, other3_v0], ...
                'v_ref', [ego_vref, other1_vref, other2_vref, other3_vref],...
                'planner', 'MANUAL');
            out = run_simulation('simTime', t_sim);
            disp(['s: ',num2str(out.s(end))]);
            disp(['d: ',num2str(out.d(end))]);
            % save the out.s and out.d for plot
            s_RB = out.s;
            d_RB = out.d;
            % get metrics
            [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
            % write the metrics and execution time of Rule-Based to table T1
            T1.reachedDis_RB = out.s(end);
            T1.minTTC_RB = min_TTC;
            T1.meanTTC_RB = meanTTC;
            T1.TET_RB = TET;
            T1.executionTime_RB = toc;
        catch
            % if bug set all values of Rule-Based to -1
            T1.reachedDis_RB = -1;
            T1.minTTC_RB = -1;
            T1.meanTTC_RB = -1;
            T1.TET_RB = -1;
            T1.executionTime_RB = -1;
            toc;
            disp('***********scenario failed!!!*******')
        end
        

        disp('------------------------------------------')
        disp('POMDP Planner')
        tic;
        try
        prepare_simulation('n_other', 3, 's_0', [ego_s0,other1_s0, other2_s0, other3_s0],...
            'd_0', [ego_d0, other1_d0, other2_d0, other3_d0], ...
            'v_0', [ego_v0, other1_v0, other2_v0, other3_v0], ...
            'v_ref', [ego_vref, other1_vref, other2_vref, other3_vref],...
            'planner', 'POMDP');
        out = run_simulation('simTime', t_sim);
        disp(['s: ',num2str(out.s(end))]);
        disp(['d: ',num2str(out.d(end))]);
        % save the out.s and out.d for plot
        s_POMDP = out.s;
        d_POMDP = out.d;
        % get metrics
        [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
        % write the metrics and execution time of POMDP to table T1
        T1.reachedDis_POMDP = out.s(end);
        T1.minTTC_POMDP = min_TTC;
        T1.meanTTC_POMDP = meanTTC;
        T1.TET_POMDP = TET;
        T1.executionTime_POMDP = toc;
        catch
            T1.reachedDis_POMDP = -1;
            T1.minTTC_POMDP = -1;
            T1.meanTTC_POMDP = -1;
            T1.TET_POMDP = -1;
            T1.executionTime_POMDP = -1;
            toc;
            disp('***********scenario failed!!!*******')
        end

        disp('------------------------------------------')
        disp('Minimax Planner')
        tic;
        try
            prepare_simulation('n_other', 3, 's_0', [ego_s0,other1_s0, other2_s0, other3_s0],...
            'd_0', [ego_d0, other1_d0, other2_d0, other3_d0], ...
            'v_0', [ego_v0, other1_v0, other2_v0, other3_v0], ...
            'v_ref', [ego_vref, other1_vref, other2_vref, other3_vref],...
            'planner', 'NEWPLANNER');
        out = run_simulation('simTime', t_sim);
        disp(['s: ',num2str(out.s(end))]);
        disp(['d: ',num2str(out.d(end))]);
        % save the out.s and out.d for plot
        s_MINIMAX = out.s;
        d_MINIMAX = out.d;
        % get metrics
        [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
        % write the metrics and execution time of MINIMAX to table T1
        T1.reachedDis_MINIMAX = out.s(end);
        T1.minTTC_MINIMAX = min_TTC;
        T1.meanTTC_MINIMAX = meanTTC;
        T1.TET_MINIMAX = TET;
        T1.executionTime_MINIMAX = toc;
        T1.numOtherVehicles = 3;
        % concatenate two tables
        
        catch
            % if bug set all values of MINIMAX to -1 except the number of other vehicles
            T1.reachedDis_MINIMAX = -1;
            T1.minTTC_MINIMAX = -1;
            T1.meanTTC_MINIMAX = -1;
            T1.TET_MINIMAX = -1;
            T1.executionTime_MINIMAX = -1;
            T1.numOtherVehicles = 3;
            toc;
            % concatenate two tables
            
            disp('***********scenario failed!!!*******')
        end
        T = [T;T1];
        disp('------------------------------------------')
        % plot the figures and save
        plotScn(k, s_RB, d_RB, s_POMDP, d_POMDP, s_MINIMAX, d_MINIMAX);
        

        %% Change initial parameters randomly
        
        other1_s0 = 400*rand(1,1);
        other2_s0 = 400*rand(1,1);
        other3_s0 = 400*rand(1,1);
       
        other1_v0 = 30*rand(1,1);
        other2_v0 = 30*rand(1,1);
        other3_v0 = 30*rand(1,1);

        other1_vref = 30*rand(1,1);
        other2_vref = 30*rand(1,1);
        other3_vref = 30*rand(1,1);
        
        ego_s0 = 300*rand(1,1);
        ego_v0 = 30*rand(1,1);
        ego_vref = 30*rand(1,1);
%         other1_s0 = 185 + (-20 + (20+20)*rand(1,1));
%         other2_s0 = 0 + (0 + (20+20)*rand(1,1));
%         other3_s0 = 200 + (-20 + (20+20)*rand(1,1));
% 
%         other1_v0 = 13 + (-3 + (3+3)*rand(1,1));
%         other2_v0 = 25 + (-3 + (3+3)*rand(1,1));
%         other3_v0 = 14 + (-3 + (3+3)*rand(1,1));
% 
%         other1_vref = 13 + (-5 + (5+5)*rand(1,1));
%         other2_vref = 25 + (-5 + (5+5)*rand(1,1));
%         other3_vref = 14 + (-5 + (5+5)*rand(1,1));
% 
%         ego_s0 = 140 + (-20 + (20+20)*rand(1,1));
%         ego_v0 = 15 + (-3 + (3+3)*rand(1,1));
%         ego_vref = 15 + (-5 + (5+5)*rand(1,1));
        writetable(T, 'testResults.csv');
    end
elseif num_other_vehicles == 2
    for k =1:num_iter % For two other vehicles

        disp(['Experiment no: ',num2str(k),' out of ', num2str(num_iter)]);
        
        disp(['Simulation for -> Ego_s0: ',num2str(ego_s0), '/ Ego_v0: ', num2str(ego_v0),...
            '/ Ego_vref: ', num2str(ego_vref)]);

        disp(['Simulation for -> other1_s0: ',num2str(other1_s0), '/ other1_v0: ', num2str(other1_v0),...
            '/ other1_vref: ', num2str(other1_vref)]);

        disp(['Simulation for -> other2_s0: ',num2str(other2_s0), '/ other2_v0: ', num2str(other2_v0),...
            '/ other2_vref: ', num2str(other2_vref)]);


        disp('------------------------------------------')
        disp('Rule-based Planner')
        tic;
        scn = ['scn', num2str(k)];
        space_length = length(scn_name) - length(scn); 
        scn(length(scn)+1:length(scn_name)) = ' ';
        T1.scenario = scn;
        try
            prepare_simulation('n_other', 2, 's_0', [ego_s0, other1_s0, other2_s0],...
                'd_0', [ego_d0, other1_d0, other2_d0], ...
                'v_0', [ego_v0, other1_v0, other2_v0], ...
                'v_ref', [ego_vref, other1_vref, other2_vref],...
                'planner', 'MANUAL');
            out = run_simulation('simTime', t_sim);
            disp(['s: ',num2str(out.s(end))]);
            disp(['d: ',num2str(out.d(end))]);
            % save the out.s and out.d for plot
            s_RB = out.s;
            d_RB = out.d;
            % get metrics
            [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
            % write the metrics and execution time of Rule-Based to table T1
            T1.reachedDis_RB = out.s(end);
            T1.minTTC_RB = min_TTC;
            T1.meanTTC_RB = meanTTC;
            T1.TET_RB = TET;
            T1.executionTime_RB = toc;
        catch
            % if bug set all values of Rule-Based to -1
            T1.reachedDis_RB = -1;
            T1.minTTC_RB = -1;
            T1.meanTTC_RB = -1;
            T1.TET_RB = -1;
            T1.executionTime_RB = -1;
            toc;
            disp('***********scenario failed!!!*******')
        end

        disp('------------------------------------------')
        disp('POMDP Planner')
        tic;
        try
        prepare_simulation('n_other', 2, 's_0', [ego_s0, other1_s0, other2_s0],...
            'd_0', [ego_d0, other1_d0, other2_d0], ...
            'v_0', [ego_v0, other1_v0, other2_v0], ...
            'v_ref', [ego_vref, other1_vref, other2_vref],...
            'planner', 'POMDP');
        out = run_simulation('simTime', t_sim);
        disp(['s: ',num2str(out.s(end))]);
        disp(['d: ',num2str(out.d(end))]);
        % save the out.s and out.d for plot
        s_POMDP = out.s;
        d_POMDP = out.d;
        % get metrics
        [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
        % write the metrics and execution time of POMDP to table T1
        T1.reachedDis_POMDP = out.s(end);
        T1.minTTC_POMDP = min_TTC;
        T1.meanTTC_POMDP = meanTTC;
        T1.TET_POMDP = TET;
        T1.executionTime_POMDP = toc;
        catch
            T1.reachedDis_POMDP = -1;
            T1.minTTC_POMDP = -1;
            T1.meanTTC_POMDP = -1;
            T1.TET_POMDP = -1;
            T1.executionTime_POMDP = -1;
            toc;
            disp('***********scenario failed!!!*******')
        end

        disp('------------------------------------------')
        disp('Minimax Planner')
        tic;
        try
            prepare_simulation('n_other', 2, 's_0', [ego_s0, other1_s0, other2_s0],...
            'd_0', [ego_d0, other1_d0, other2_d0], ...
            'v_0', [ego_v0, other1_v0, other2_v0], ...
            'v_ref', [ego_vref, other1_vref, other2_vref],...
            'planner', 'NEWPLANNER');
        out = run_simulation('simTime', t_sim);
        disp(['s: ',num2str(out.s(end))]);
        disp(['d: ',num2str(out.d(end))]);
        % save the out.s and out.d for plot
        s_MINIMAX = out.s;
        d_MINIMAX = out.d;
        % get metrics
        [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
        % write the metrics and execution time of MINIMAX to table T1
        T1.reachedDis_MINIMAX = out.s(end);
        T1.minTTC_MINIMAX = min_TTC;
        T1.meanTTC_MINIMAX = meanTTC;
        T1.TET_MINIMAX = TET;
        T1.executionTime_MINIMAX = toc;
        T1.numOtherVehicles = 2;
        % concatenate two tables
        T = [T;T1];
        catch
            % if bug set all values of MINIMAX to -1 except the number of other vehicles
            T1.reachedDis_MINIMAX = -1;
            T1.minTTC_MINIMAX = -1;
            T1.meanTTC_MINIMAX = -1;
            T1.TET_MINIMAX = -1;
            T1.executionTime_MINIMAX = -1;
            T1.numOtherVehicles = 2;
            toc;
            % concatenate two tables
            T = [T;T1];
            disp('***********scenario failed!!!*******')
        end

        disp('------------------------------------------')
        % plot the figures and save
        plotScn(k, s_RB, d_RB, s_POMDP, d_POMDP, s_MINIMAX, d_MINIMAX);


        %% Change initial parameters randomly
        other1_s0 = 400*rand(1,1);
        other2_s0 = 400*rand(1,1);

        other1_v0 = 30*rand(1,1);
        other2_v0 = 30*rand(1,1);

        other1_vref = 30*rand(1,1);
        other2_vref = 30*rand(1,1);

        ego_s0 = 300*rand(1,1);
        ego_v0 = 30*rand(1,1);
        ego_vref = 30*rand(1,1);
        writetable(T, 'testResults.csv');
    end
elseif num_other_vehicles == 1
    for k = 1:num_iter % For one other vehicle

        disp(['Experiment no: ',num2str(k),' out of ', num2str(num_iter)]);
        
        disp(['Simulation for -> Ego_s0: ',num2str(ego_s0), '/ Ego_v0: ', num2str(ego_v0),...
            '/ Ego_vref: ', num2str(ego_vref)]);

        disp(['Simulation for -> other1_s0: ',num2str(other1_s0), '/ other1_v0: ', num2str(other1_v0),...
            '/ other1_vref: ', num2str(other1_vref)]);


        disp('------------------------------------------')
        disp('Rule-based Planner')
        tic;
        scn = ['scn', num2str(k)];
        space_length = length(scn_name) - length(scn); 
        scn(length(scn)+1:length(scn_name)) = ' ';
        T1.scenario = scn;
        try
            prepare_simulation('n_other', 1, 's_0', [ego_s0,other1_s0],...
                'd_0', [ego_d0, other1_d0], ...
                'v_0', [ego_v0, other1_v0], ...
                'v_ref', [ego_vref, other1_vref],...
                'planner', 'MANUAL');
            out = run_simulation('simTime', t_sim);
            disp(['s: ',num2str(out.s(end))]);
            disp(['d: ',num2str(out.d(end))]);
            % save the out.s and out.d for plot
            s_RB = out.s;
            d_RB = out.d;
            % get metrics
            [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
            % write the metrics and execution time of Rule-Based to table T1
            T1.reachedDis_RB = out.s(end);
            T1.minTTC_RB = min_TTC;
            T1.meanTTC_RB = meanTTC;
            T1.TET_RB = TET;
            T1.executionTime_RB = toc;
        catch
            % if bug set all values of Rule-Based to -1
            T1.reachedDis_RB = -1;
            T1.minTTC_RB = -1;
            T1.meanTTC_RB = -1;
            T1.TET_RB = -1;
            T1.executionTime_RB = -1;
            toc;
            disp('***********scenario failed!!!*******')
        end
        

        disp('------------------------------------------')
        disp('POMDP Planner')
        tic;
        try
        prepare_simulation('n_other', 1, 's_0', [ego_s0,other1_s0],...
            'd_0', [ego_d0, other1_d0], ...
            'v_0', [ego_v0, other1_v0], ...
            'v_ref', [ego_vref, other1_vref],...
            'planner', 'POMDP');
        out = run_simulation('simTime', t_sim);
        disp(['s: ',num2str(out.s(end))]);
        disp(['d: ',num2str(out.d(end))]);
        % save the out.s and out.d for plot
        s_POMDP = out.s;
        d_POMDP = out.d;
        % get metrics
        [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
        % write the metrics and execution time of POMDP to table T1
        T1.reachedDis_POMDP = out.s(end);
        T1.minTTC_POMDP = min_TTC;
        T1.meanTTC_POMDP = meanTTC;
        T1.TET_POMDP = TET;
        T1.executionTime_POMDP = toc;
        catch
            T1.reachedDis_POMDP = -1;
            T1.minTTC_POMDP = -1;
            T1.meanTTC_POMDP = -1;
            T1.TET_POMDP = -1;
            T1.executionTime_POMDP = -1;
            toc;
            disp('***********scenario failed!!!*******')
        end

        disp('------------------------------------------')
        disp('Minimax Planner')
        tic;
        try
            prepare_simulation('n_other', 1, 's_0', [ego_s0,other1_s0],...
            'd_0', [ego_d0, other1_d0], ...
            'v_0', [ego_v0, other1_v0], ...
            'v_ref', [ego_vref, other1_vref],...
            'planner', 'NEWPLANNER');
        out = run_simulation('simTime', t_sim);
        disp(['s: ',num2str(out.s(end))]);
        disp(['d: ',num2str(out.d(end))]);
        % save the out.s and out.d for plot
        s_MINIMAX = out.s;
        d_MINIMAX = out.d;
        % get metrics
        [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, out.d, out.d_other, out.s, out.s_other, out.v, out.v_other);
        % write the metrics and execution time of MINIMAX to table T1
        T1.reachedDis_MINIMAX = out.s(end);
        T1.minTTC_MINIMAX = min_TTC;
        T1.meanTTC_MINIMAX = meanTTC;
        T1.TET_MINIMAX = TET;
        T1.executionTime_MINIMAX = toc;
        T1.numOtherVehicles = 1;
        % concatenate two tables
        T = [T;T1];
        catch
            % if bug set all values of MINIMAX to -1 except the number of other vehicles
            T1.reachedDis_MINIMAX = -1;
            T1.minTTC_MINIMAX = -1;
            T1.meanTTC_MINIMAX = -1;
            T1.TET_MINIMAX = -1;
            T1.executionTime_MINIMAX = -1;
            T1.numOtherVehicles = 1;
            toc;
            % concatenate two tables
            T = [T;T1];
            disp('***********scenario failed!!!*******')
        end

        disp('------------------------------------------')
        % plot the figures and save
        plotScn(k, s_RB, d_RB, s_POMDP, d_POMDP, s_MINIMAX, d_MINIMAX);

        %% Change initial parameters randomly
       
        other1_s0 = 400*rand(1,1);

        other1_v0 = 30*rand(1,1);

        other1_vref = 30*rand(1,1);

        ego_s0 = 300*rand(1,1);
        ego_v0 = 30*rand(1,1);
        ego_vref = 30*rand(1,1);
        writetable(T, 'testResults.csv');
    end    
end
% %% save the MATLAB table 
% writetable(T, 'testResults.csv');
        
%% functions to plot and calculate metrics
%% plot 
function plotScn(scn_idx, s_RB, d_RB, s_POMDP, d_POMDP, s_MINIMAX, d_MINIMAX)
% figure('visible','off');
% set the size of the figure
scrsz = get(0,'ScreenSize');
figure('Position',[0, 30, scrsz(3), scrsz(4)-95]);
hold on;
p1 = plot(s_RB, d_RB, 'color', [1,140/255,0]);
p1(1).LineWidth = 3;
p2 = plot(s_POMDP, d_POMDP, 'b');
p2(1).LineWidth = 3;
p3 = plot(s_MINIMAX, d_MINIMAX, 'color', [34/255, 177/255, 76/255]);
p3(1).LineWidth = 3;
% mark the end longitudinal position with diamond marker
plot(s_RB(end),0,'color', [1,140/255,0],'Marker','d','MarkerSize',16, 'MarkerFaceColor', [1,140/255,0]);
plot(s_POMDP(end), 0, 'color', 'b', 'Marker','d','MarkerSize',13,'MarkerFaceColor','b');
plot(s_MINIMAX(end),0,'color', [34/255, 177/255, 76/255],'Marker','d','MarkerSize',10, 'MarkerFaceColor',[34/255, 177/255, 76/255]);
grid on;
title(['Scenario ', num2str(scn_idx)], 'FontSize', 25);
% mark the initial position with asterisk marker
fig = plot(s_POMDP(1), d_POMDP(1), 'color', 'k', 'Marker','*','MarkerSize', 15);
legend('Rule-Based','POMDP','MINIMAX');
xlabel('Longitudinal Distance s (m)', 'FontSize', 25);
ylabel('Lateral Offset d (m)', 'FontSize', 25);
set(gca, 'FontSize', 25);
ylim([0 4]);
hold off;
set(gcf,'visible','off')
% save the figure
image_name = ['./figures/scenario', num2str(scn_idx),'.png'];
saveas(fig, image_name);
end
%% get metrics
function [min_TTC, meanTTC, totalTimeTTCunderTao, TET] = getMetric(time_step, tao, d, d_other, s, s_other, v, v_other)
% TTC
TTC = [TTCright(d, d_other, s, s_other, v, v_other);...
       TTCleft(d, d_other, s, s_other, v, v_other)];
% min TTC
min_TTC = min(TTC(TTC ~= inf));
if isempty(min_TTC)
    min_TTC = 0;
end
disp(['min TTC: ', num2str(min_TTC)])
% mean TTC under tao
meanTTC = mean(TTC(TTC < tao));
if isnan(meanTTC)
    meanTTC = 0;
end
disp(['mean TTC under ', num2str(tao), ': ', num2str(meanTTC)])
% total time TTC under tao
totalTimeTTCunderTao = totalTimeTTC(TTC, tao, time_step);
disp(['total time TTC under ', num2str(tao), ': ', num2str(totalTimeTTCunderTao)])
% TET
TET = sum(getTET(TTC, time_step, tao));
disp(['TET under ', num2str(tao), ' :', num2str(TET)])
end
%% calculate total time TTC under tao
function totalTimeTTCunderTao = totalTimeTTC(TTC, tao, time_step)
TTCunderTao = TTC(TTC < tao);
totalTimeTTCunderTao = length(TTCunderTao)*time_step;
end
%% calculate left lane TTC
function TTC_left = TTCleft(d, d_other, s, s_other, v, v_other)
% find the index of other vehicle on the left lane
left = d_other >= 2.85;
idx_left_lane = find(left == 1);
% the longitudinal displacement of the other vehicles on the left
% lane
s_other = s_other(:, idx_left_lane);
% the index of s of ego vehicle on the left lane
idx_ego_left_lane = d >= 2.85 ;
% the longitudinal displacement of ego vehicle on the left lane
s_ego_left = s.*idx_ego_left_lane;
% the longitudinal displacement of other vehicles on the left lane
s_other_left = s_other.*idx_ego_left_lane;
% index of leader vehicle on the same lane as ego
idx_leader_s = (s_other_left - s_ego_left) > 0.01;
% the speed of ego vehicle on the left lane
v_ego_left = v.*idx_ego_left_lane;
% the speed of other vehicles on the same lane 
v_other_left = v_other(:, idx_left_lane).*idx_ego_left_lane;
idx_slow_v = (v_ego_left - v_other_left)>0.01;
% calculate the longitudinal displacement and speed differenz at the same
% time step
s_left = (s_other_left - s_ego_left).*idx_leader_s.*idx_slow_v;
v_left = (v_ego_left - v_other_left).*idx_leader_s.*idx_slow_v;
TTC_left = s_left./v_left;
TTC_left = TTC_left';
% assign inf if the component of TTC is NaN
TTC_left(isnan(TTC_left)) = inf;
end
%% calculate right lane TTC
function TTC_right = TTCright(d, d_other, s, s_other, v, v_other)
% find the index of other vehicle on the right lane
right = d_other <= 0.1;
idx_right_lane = find(right == 1);
% the longitudinal displacement of the other vehicles on the right lane
s_other = s_other(:, idx_right_lane);
% the index of s of ego vehicle on the right lane
idx_ego_right_lane = d <= 0.1;
% the longitudinal displacement of ego vehicle on the right lane
s_ego_right = s.*idx_ego_right_lane;
% the longitudinal displacement of other vehicles on the right lane
s_other_right = s_other.*idx_ego_right_lane;
% index of leader vehicle on the same lane as ego
idx_leader_s = (s_other_right - s_ego_right) > 0.01; %exclude zero values
% the speed of ego vehicle on the right lane
v_ego_right = v.*idx_ego_right_lane;
% the speed of other vehicles on the same lane 
v_other_right = v_other(:, idx_right_lane).*idx_ego_right_lane;
idx_slow_v = (v_ego_right - v_other_right)>0.01;
% calculate the longitudinal displacement and speed differenz at the same
% time step
s_right = (s_other_right - s_ego_right).*idx_leader_s.*idx_slow_v;
v_right = (v_ego_right - v_other_right).*idx_leader_s.*idx_slow_v;
% TTC
TTC_right = s_right./v_right;
TTC_right = TTC_right';
% assign inf if the component of TTC is NaN
TTC_right(isnan(TTC_right)) = inf;
end
%% calculate TET for one other vehicle
function TET = TETcal(TTC, idx_vehicle, time_step, tao)
% TET calculation
% index of TET 
idx_tet = 1;
TET = zeros(1,15);
% find the index of noninfinity components
nonzeroIdx = find(TTC(idx_vehicle, :) ~= Inf);
for i=1:length(nonzeroIdx)
    % if i is the length of nonzeroIdx or the differenz between (i+1)th and
    % ith componnent is 1, then add the nonzeroIdx(i)th component of TTC to
    % beta(t)
    if i == length(nonzeroIdx)|| nonzeroIdx(i + 1) - nonzeroIdx(i) == 1
        if TTC(idx_vehicle, nonzeroIdx(i)) < tao
           
            TET(idx_tet) = TET(idx_tet) + TTC(idx_vehicle, nonzeroIdx(i));
        end
    else
        % otherwise increase t and add the nonzeroIdx(i)th component of TTC
        % to beta(t)
        if TTC(idx_vehicle, nonzeroIdx(i)) < tao
            TET(idx_tet) = TET(idx_tet) + TTC(idx_vehicle, nonzeroIdx(i));
        end
        idx_tet = idx_tet+1;
    end
end
TET = TET*time_step;
end
%% calculate TET for all other vehicles and sum them up
function TETTC = getTET(TTC, time_step, tao)
TETTC = [];
i = 0;
% for each other vehicle calculate the TET and store in TETTC
for idx_vehicle = 1:size(TTC, 1) 
    %TETTC = TETTC + sum(TETcal(TTC, idx_vehicle, time_step));
    TET = TETcal(TTC, idx_vehicle, time_step, tao);
    TETnonzero = TET(TET ~= 0);
    for j = 1:size(TETnonzero, 2)
        TETTC(i + j) = TETnonzero(j);
    end
    i = i+size(TETnonzero, 2);
end
end