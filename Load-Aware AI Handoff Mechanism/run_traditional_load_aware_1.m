% run_traditional_load_aware_1.m
% =========================================================================
% TRADITIONAL ALGORITHM IN LOAD-AWARE ENVIRONMENT
% 1. Instantiates the exact HandoffEnvironment used by the AI.
% 2. Uses traditional A3 logic (Neighbor > Serving + 3dB) to make decisions.
% 3. Evaluates how badly the traditional logic suffers from congestion.
% =========================================================================

clear; clc; close all;

% --- 1. SETUP SIMULATION PARAMETERS ---
ISD = 1000; % Inter-Site Distance (meters)
gNB_positions = [0, 0]; 
theta = deg2rad(0:60:300); 
for i = 1:length(theta)
    x = ISD * cos(theta(i));
    y = ISD * sin(theta(i));
    gNB_positions = [gNB_positions; x, y];
end

params.gNB_positions = gNB_positions;
params.UE_velocity_vector = [10, 5]; 
params.UE_trajectory_start = [0, 0]; 
params.time_step = 0.1; 

% Traditional Logic Parameters
hysteresis_dB = 3.0; 
Tx_Power_dBm = 30;

% --- 2. CREATE ENVIRONMENT ---
obsInfo = rlNumericSpec([14 1], 'LowerLimit', 0, 'UpperLimit', 1, ...
    'Name', 'HandoffObs', 'Description', 'Normalized_RSRPs_and_Loads');
actInfo = rlFiniteSetSpec([1 2], 'Name', 'HandoffAction');

env = HandoffEnvironment(params, obsInfo, actInfo);

% --- 3. RUN TRADITIONAL SIMULATION ---
disp('Running Traditional Handoff Simulation...');
maxSteps = 2500;

% Logging arrays
hist.time = zeros(maxSteps, 1);
hist.cell = zeros(maxSteps, 1);
hist.rsrp = zeros(maxSteps, 1);
hist.load = zeros(maxSteps, 1);
hist.handoff_event = zeros(maxSteps, 1);

% Reset environment to get a random starting state
obs = reset(env);

for i = 1:maxSteps
    % --- A. GET GROUND TRUTH FROM ENVIRONMENT ---
    ue_pos = env.UE_position;
    serving_cell = env.serving_gNB;
    
    % --- B. CALCULATE ALL RSRPs FOR TRADITIONAL LOGIC ---
    dists = vecnorm(params.gNB_positions - ue_pos, 2, 2);
    pl = 128.1 + 37.6 * log10(max(dists, 10)/1000);
    all_rsrps = Tx_Power_dBm - pl;
    
    srv_rsrp = all_rsrps(serving_cell);
    
    % --- C. TRADITIONAL A3 LOGIC ---
    neigh_idxs = find((1:7)' ~= serving_cell);
    [best_neigh_rsrp, local_idx] = max(all_rsrps(neigh_idxs));
    best_neigh_id = neigh_idxs(local_idx);
    
    % If a neighbor is stronger by the Hysteresis margin, switch to it
    if best_neigh_rsrp > (srv_rsrp + hysteresis_dB)
        env.serving_gNB = best_neigh_id; % Force the environment to switch
        hist.handoff_event(i) = 1;
    end
    
    % --- D. STEP ENVIRONMENT PHYSICS ---
    % Send Action 1 ("Stay") to the environment. 
    % We already handled the switch manually above, so we just want the 
    % environment to advance time, move the UE, and fluctuate the loads.
    [nextObs, reward, isDone, loggedData] = step(env, 1);
    
    % --- E. LOG DATA ---
    hist.time(i) = (i-1) * params.time_step;
    hist.cell(i) = loggedData.Serving_gNB;
    
    % Denormalize RSRP from observation for plotting: (Norm * 100) - 140
    hist.rsrp(i) = (nextObs(1) * 100) - 140; 
    
    % Load is at index 8 in the observation vector
    hist.load(i) = nextObs(8); 
    
    obs = nextObs;
    if isDone, break; end
end

% Trim Data
actualSteps = i;
fields = fieldnames(hist);
for k=1:numel(fields)
    hist.(fields{k}) = hist.(fields{k})(1:actualSteps);
end

% --- 4. METRICS & ANALYSIS ---
num_ho = sum(hist.handoff_event);
avg_load = mean(hist.load);
congestion_pct = sum(hist.load > 0.8) / actualSteps * 100;

fprintf('\n=== TRADITIONAL ALGORITHM RESULTS ===\n');
fprintf('Logic: A3 Event (Hysteresis = %.1f dB)\n', hysteresis_dB);
fprintf('Total Handovers: %d\n', num_ho);
fprintf('Avg Serving Load: %.2f%%\n', avg_load * 100);
fprintf('Time in Congestion: %.2f%% (Notice how high this is!)\n', congestion_pct);
fprintf('=====================================\n');

% --- 5. VISUALIZATION ---
figure('Name', 'Traditional Load-Aware Performance', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

% 1. RSRP Plot
subplot(3,1,1); hold on; grid on;
plot(hist.time, hist.rsrp, 'b-', 'LineWidth', 1.5);
yline(-110, 'r--', 'Drop Threshold', 'LineWidth', 1.5);
ylabel('RSRP (dBm)'); title('1. Signal Quality (Traditional focuses ONLY on this)');
legend('Serving Cell');

% 2. Load Plot (The Problem Area)
subplot(3,1,2); hold on; grid on;
plot(hist.time, hist.load, 'm-', 'LineWidth', 1.5);
yline(0.8, 'r--', 'Congestion Threshold (80%)', 'LineWidth', 2);
ylabel('Load (0-1)'); title('2. Traffic Load (Traditional is BLIND to this)');
ylim([0 1]);
legend('Serving Load', 'Congestion Limit');

% 3. Cell ID Plot
subplot(3,1,3); hold on; grid on;
plot(hist.time, hist.cell, 'k-', 'LineWidth', 1.5);
ho_idx = find(hist.handoff_event);
plot(hist.time(ho_idx), hist.cell(ho_idx), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
ylabel('Cell ID'); xlabel('Time (s)'); title('3. Serving Cell ID & Handoffs');
yticks(1:7); ylim([0 8]);

disp('Simulation Complete. Check the "Traffic Load" plot to see the congestion spikes.');