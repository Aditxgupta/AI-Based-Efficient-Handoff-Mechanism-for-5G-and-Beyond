% run_traditional_load_aware.m
% =========================================================================
% TRADITIONAL ALGORITHM BASELINE (A3 Event w/ Hysteresis)
% 1. Replicates the exact physics/load dynamics of your HandoffEnvironment.
% 2. Uses standard logic: Switch if Neighbor > Serving + 3dB.
% 3. IGNORES Load (Blindly enters congestion), acting as a baseline.
% =========================================================================

clear; clc; close all;

% --- 1. SETUP PARAMETERS (Matching your RL Environment) ---
ISD = 1000; 
gNB_positions = [0, 0]; 
theta = deg2rad(0:60:300); 
for i = 1:length(theta)
    x = ISD * cos(theta(i));
    y = ISD * sin(theta(i));
    gNB_positions = [gNB_positions; x, y];
end

% Simulation Settings
maxSteps = 2500;
time_step = 0.1;
hysteresis_dB = 3.0; % Standard hysteresis to prevent ping-pong
Tx_Power_dBm = 30;   % Matches your env
grid_radius = 1500;

% --- 2. INITIALIZATION ---
% Random Start
theta_start = 2 * pi * rand;
r_start = grid_radius * sqrt(rand);
ue_pos = [r_start * cos(theta_start), r_start * sin(theta_start)];
ue_vel = (rand(1,2)-0.5) * 10;

% Initialize Loads (Random like env)
gNB_loads = rand(7, 1);

% Connect to strongest initial cell
dists = vecnorm(gNB_positions - ue_pos, 2, 2);
pl = 128.1 + 37.6 * log10(max(dists, 10)/1000);
rsrps = Tx_Power_dBm - pl;
[~, serving_cell] = max(rsrps);

% Logging
history.time = zeros(maxSteps, 1);
history.serving_cell = zeros(maxSteps, 1);
history.serving_rsrp = zeros(maxSteps, 1);
history.serving_load = zeros(maxSteps, 1);
history.handoff_event = zeros(maxSteps, 1);

disp('Running Traditional Handoff Simulation...');

% --- 3. SIMULATION LOOP ---
for i = 1:maxSteps
    % A. Update Load (Fluctuation)
    load_noise = (rand(7, 1) - 0.5) * 0.05; 
    gNB_loads = gNB_loads + load_noise;
    gNB_loads(gNB_loads > 1) = 1; gNB_loads(gNB_loads < 0) = 0;
    
    % B. Update Physics (Random Walk)
    accel = (rand(1,2) - 0.5) * 4; % 2 * acceleration_noise
    ue_vel = ue_vel + accel;
    speed = norm(ue_vel);
    if speed > 25, ue_vel = (ue_vel/speed)*25; end
    if speed < 5,  ue_vel = (ue_vel/speed)*5; end
    
    if norm(ue_pos) > grid_radius
        ue_vel = ue_vel + (-ue_pos/norm(ue_pos) * 2.0);
    end
    ue_pos = ue_pos + ue_vel * time_step;
    
    % C. Calculate RSRP
    dists = vecnorm(gNB_positions - ue_pos, 2, 2);
    pl = 128.1 + 37.6 * log10(max(dists, 10)/1000);
    current_rsrps = Tx_Power_dBm - pl;
    
    % --- D. TRADITIONAL LOGIC (A3 Event) ---
    serving_rsrp = current_rsrps(serving_cell);
    
    % Find strongest neighbor
    neighbor_idxs = find((1:7)' ~= serving_cell);
    [best_neigh_rsrp, best_local_idx] = max(current_rsrps(neighbor_idxs));
    best_neigh_idx = neighbor_idxs(best_local_idx);
    
    % Logic: If Neighbor is stronger than Serving + Hysteresis
    % NOTICE: It does NOT check gNB_loads (Blind to congestion)
    if best_neigh_rsrp > (serving_rsrp + hysteresis_dB)
        serving_cell = best_neigh_idx;
        history.handoff_event(i) = 1;
    end
    
    % E. Log Data
    history.time(i) = (i-1)*time_step;
    history.serving_cell(i) = serving_cell;
    history.serving_rsrp(i) = current_rsrps(serving_cell);
    history.serving_load(i) = gNB_loads(serving_cell);
end

% --- 4. METRICS ---
num_ho = sum(history.handoff_event);
avg_load = mean(history.serving_load);
congestion_pct = sum(history.serving_load > 0.8) / maxSteps * 100;

fprintf('\n=== TRADITIONAL ALGORITHM RESULTS ===\n');
fprintf('Logic: Connect to Strongest Signal (A3)\n');
fprintf('Total Handovers: %d\n', num_ho);
fprintf('Avg Serving Load: %.2f%%\n', avg_load * 100);
fprintf('Time in Congestion: %.2f%% (Likely High!)\n', congestion_pct);

% --- 5. PLOTTING ---
figure('Name', 'Traditional Baseline Performance', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

% Plot 1: RSRP
subplot(3,1,1); hold on; grid on;
plot(history.time, history.serving_rsrp, 'b-', 'LineWidth', 1.5);
yline(-110, 'r--', 'Drop Threshold');
title('1. Signal Strength (Traditional)'); ylabel('RSRP (dBm)');
legend('Serving Cell');

% Plot 2: Load (The Critical Plot)
subplot(3,1,2); hold on; grid on;
plot(history.time, history.serving_load, 'm-', 'LineWidth', 1.5);
yline(0.8, 'r--', 'Congestion Threshold', 'LineWidth', 2);
title('2. Traffic Load (Traditional ignores this!)'); ylabel('Load (0-1)');
ylim([0 1]);
legend('Serving Load', 'Congestion Limit');

% Plot 3: Cell ID
subplot(3,1,3); hold on; grid on;
plot(history.time, history.serving_cell, 'k-', 'LineWidth', 1.5);
ho_idx = find(history.handoff_event);
plot(history.time(ho_idx), history.serving_cell(ho_idx), 'rx', 'MarkerSize', 8, 'LineWidth', 2);
title('3. Serving Cell ID'); ylabel('Cell ID'); xlabel('Time (s)');
ylim([0 8]);