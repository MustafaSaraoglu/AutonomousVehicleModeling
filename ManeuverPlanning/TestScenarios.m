%% Scenario 1: Pure Pursuit vs Stanley______________________________________________________________
%% Pure Pursuit
prepare_simulation('n_other', 1, 's_0', [0, 40], 'd_0', [0, 0], ...
    'v_0', [20, 10], 'v_ref', [20, 10], 'planner', 'Minimax-Dev', 'lateral', 'PURE_PURSUIT'); % With uncertainty
out = run_simulation('simTime', 12);

E_max = max(abs(out.d-out.d_ref))
ISE = max(out.ISE_d)

figure;
p = plot(out.tout, out.d, 'b', out.tout, out.d_ref, 'g');
p(1).LineWidth = 7;
p(2).LineWidth = 5;
grid on;
title('Tracking Pure Pursuit', 'FontSize', 50);
xlabel('Time t [s]', 'FontSize', 50);
ylabel('Lateral Offset d [m]', 'FontSize', 50);
legend('d(t)', 'd_{ref}(t)', 'FontSize', 50);
set(gca, 'FontSize', 50);

%% Stanley
prepare_simulation('n_other', 1, 's_0', [0, 40], 'd_0', [0, 0], ...
    'v_0', [20, 10], 'v_ref', [20, 10], 'planner', 'Minimax-Dev', 'lateral', 'Stanley'); % With uncertainty
out = run_simulation('simTime', 12);

E_max = max(abs(out.d-out.d_ref));
ISE = max(out.ISE_d)

figure;
p = plot(out.tout, out.d, 'b', out.tout, out.d_ref, 'g');
p(1).LineWidth = 7;
p(2).LineWidth = 5;
grid on;
title('Tracking Lateral Stanley Controller', 'FontSize', 50);
xlabel('Time t [s]', 'FontSize', 50);
ylabel('Lateral Offset d [m]', 'FontSize', 50);
legend('d(t)', 'd_{ref}(t)', 'FontSize', 50);
set(gca, 'FontSize', 50);

%% Curved
road_geometry = [0      0       0; 
                 1000   0   -1000; 
                 1.5708 0   -1000;
                 -1    -1      -1]; 
prepare_simulation('road', road_geometry, 'n_other', 1, 's_0', [0, 40], 'd_0', [0, 0], ...
    'v_0', [20, 10], 'v_ref', [20, 10], 'planner', 'Minimax-Dev'); % With uncertainty
out = run_simulation('simTime', 12);

E_max = max(abs(out.d-out.d_ref))
ISE = max(out.ISE_d)

figure;
p = plot(out.tout, out.d, 'b', out.tout, out.d_ref, 'g');
p(1).LineWidth = 7;
p(2).LineWidth = 5;
grid on;
title('Tracking Lateral Stanley Controller Curved Road', 'FontSize', 50);
xlabel('Time t [s]', 'FontSize', 50);
ylabel('Lateral Offset d [m]', 'FontSize', 50);
legend('d(t)', 'd_{ref}(t)', 'FontSize', 50);
set(gca, 'FontSize', 50);

%% Scenario 2: Rule-Based vs Minimax Speed Performance_______________________________________________
%% Rule-Based
t_sim = 14;
prepare_simulation('s_0', [0, 12, 0], 'd_0', [0, 0, 3.7], ...
    'v_0', [15, 8, 30], 'v_ref', [15, 8, 0], 'planner', 'RuleBased'); % With uncertainty
out = run_simulation('simTime', t_sim);

figure;
n_points = length(out.tout);
grid on;
hold on;
plot(out.s, out.d, 'Color', 'r', 'LineWidth', 5);
%plot(out.s_other(:, 1), ones(n_points, 1)*out.d_other(:, 1), 'Color', 'b', 'LineWidth', 5);
%plot(out.s_other(:, 2), ones(n_points, 1)*out.d_other(:, 2), 'Color', 'c', 'LineWidth', 5);
%legend('d_{1}(s)', 'd_{2}(s)', 'd_{3}(s)', 'FontSize', 50);

plot(out.s_other(1:round(n_points/t_sim):end, 1), out.d_other(1:round(n_points/t_sim):end, 1), ...
     'Color', 'b', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
plot(out.s_other(1:round(n_points/t_sim):end, 2), out.d_other(1:round(n_points/t_sim):end, 2),...
    'Color', 'c', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
plot(out.s(1:round(n_points/t_sim):end), out.d(1:round(n_points/t_sim):end), ...
     'Color', 'r', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
hold off;
title('Rule-Based Planner', 'FontSize', 50);
xlabel('Longitudinal distance s [m]', 'FontSize', 50);
ylabel('Lateral Offset d [m]', 'FontSize', 50);
set(gca, 'FontSize', 50);

%% Minimax
t_sim = 14;
prepare_simulation('s_0', [0, 12, 0], 'd_0', [0, 0, 3.7], ...
    'v_0', [15, 8, 30], 'v_ref', [15, 8, 0], 'planner', 'Minimax-Dev'); % With uncertainty
out = run_simulation('simTime', t_sim);

figure;
n_points = length(out.tout);
grid on;
hold on;
plot(out.s, out.d, 'Color', 'r', 'LineWidth', 5);
%plot(out.s_other(:, 1), ones(n_points, 1)*out.d_other(:, 1), 'Color', 'b', 'LineWidth', 5);
%plot(out.s_other(:, 2), ones(n_points, 1)*out.d_other(:, 2), 'Color', 'c', 'LineWidth', 5);
%legend('d_{1}(s)', 'd_{2}(s)', 'd_{3}(s)', 'FontSize', 50);

plot(out.s_other(1:round(n_points/t_sim):end, 1), out.d_other(1:round(n_points/t_sim):end, 1), ...
     'Color', 'b', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
plot(out.s_other(1:round(n_points/t_sim):end, 2), out.d_other(1:round(n_points/t_sim):end, 2),...
     'Color', 'c', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
plot(out.s(1:round(n_points/t_sim):end), out.d(1:round(n_points/t_sim):end), ...
     'Color', 'r', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
hold off;
title('Minimax Planner', 'FontSize', 50);
xlabel('Longitudinal distance s [m]', 'FontSize', 50);
ylabel('Lateral Offset d [m]', 'FontSize', 50);
set(gca, 'FontSize', 50);

%% Scenario 3: Rule-Based vs Minimax Safety Performance______________________________________________
%% Rule-Based
t_sim = 17;
prepare_simulation('n_other', 3, 's_0', [140, 185, 0, 200], 'd_0', [0, 0, 3.7, 3.7], ...
    'v_0', [15, 13, 25, 14], 'v_ref', [15, 13, 25, 14], 'planner', 'RuleBased'); % With uncertainty
out = run_simulation('simTime', 17);

figure;
n_points = length(out.tout);
grid on;
hold on;
plot(out.s, out.d, 'Color', 'r', 'LineWidth', 5);
%plot(out.s_other(:, 1), ones(n_points, 1)*out.d_other(:, 1), 'Color', 'b', 'LineWidth', 5);
%plot(out.s_other(:, 2), ones(n_points, 1)*out.d_other(:, 2), 'Color', 'c', 'LineWidth', 5);
%plot(out.s_other(:, 3), ones(n_points, 1)*out.d_other(:, 3), 'Color', 'g', 'LineWidth', 5);
%legend('d_{1}(s)', 'd_{2}(s)', 'd_{3}(s)', 'd_{4}(s)', 'FontSize', 50);

plot(out.s_other(1:round(n_points/t_sim):end, 1), out.d_other(1:round(n_points/t_sim):end, 1), ...
     'Color', 'b', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
plot(out.s_other(1:round(n_points/t_sim):end, 2), out.d_other(1:round(n_points/t_sim):end, 2), ...
     'Color', 'c', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
plot(out.s_other(1:round(n_points/t_sim):end, 3), out.d_other(1:round(n_points/t_sim):end, 3), ...
     'Color', 'g', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
plot(out.s(1:round(n_points/t_sim):end), out.d(1:round(n_points/t_sim):end), ...
     'Color', 'r', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
hold off;
title('Rule-Based Planner', 'FontSize', 50);
xlabel('Longitudinal distance s [m]', 'FontSize', 50);
ylabel('Lateral Offset d [m]', 'FontSize', 50);
set(gca, 'FontSize', 50);

%% Minimax
t_sim = 17;
prepare_simulation('n_other', 3, 's_0', [140, 185, 0, 200], 'd_0', [0, 0, 3.7, 3.7], ...
    'v_0', [15, 13, 25, 14], 'v_ref', [15, 13, 25, 14], 'planner', 'Minimax-Dev'); % With uncertainty
out = run_simulation('simTime', t_sim);

figure;
n_points = length(out.tout);
grid on;
hold on;
plot(out.s, out.d, 'Color', 'r', 'LineWidth', 5);
%plot(out.s_other(:, 1), ones(n_points, 1)*out.d_other(:, 1), 'Color', 'b', 'LineWidth', 5);
%plot(out.s_other(:, 2), ones(n_points, 1)*out.d_other(:, 2), 'Color', 'c', 'LineWidth', 5);
%plot(out.s_other(:, 3), ones(n_points, 1)*out.d_other(:, 3), 'Color', 'g', 'LineWidth', 5);
%legend('d_{1}(s)', 'd_{2}(s)', 'd_{3}(s)', 'd_{4}(s)', 'FontSize', 50);

plot(out.s_other(1:round(n_points/t_sim):end, 1), out.d_other(1:round(n_points/t_sim):end, 1), ...
     'Color', 'b', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
plot(out.s_other(1:round(n_points/t_sim):end, 2), out.d_other(1:round(n_points/t_sim):end, 2), ...
     'Color', 'c', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
plot(out.s_other(1:round(n_points/t_sim):end, 3), out.d_other(1:round(n_points/t_sim):end, 3), ...
     'Color', 'g', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
plot(out.s(1:round(n_points/t_sim):end), out.d(1:round(n_points/t_sim):end), ...
     'Color', 'r', 'Marker', 'o', 'LineStyle', 'none', 'LineWidth', 5);
hold off;
title('Minimax Planner', 'FontSize', 50);
xlabel('Longitudinal distance s [m]', 'FontSize', 50);
ylabel('Lateral Offset d [m]', 'FontSize', 50);
set(gca, 'FontSize', 50);