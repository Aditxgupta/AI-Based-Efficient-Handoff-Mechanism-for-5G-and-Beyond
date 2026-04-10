% run_unified_comparison_load.m
% =========================================================================
% UNIFIED COMPARISON: AI Agent vs. Traditional (Robust Version)
% 1. Instantiates the ACTUAL HandoffEnvironment class (No manual physics).
% 2. Runs the AI Agent inside this environment.
% 3. Extracts "Ground Truth" (Position, Load) to run Traditional Logic in parallel.
% =========================================================================

clear; clc; close all;

% --- 1. SETUP ---
if ~isfile('trainedHexAgent.mat')
    error('File "trainedHexAgent.mat" not found. Train the agent first!');
end
load('trainedHexAgent.mat', 'agent', 'params');

% Force Deterministic Mode
agent.AgentOptions.EpsilonGreedyExploration.Epsilon = 0;

% Recreate Environment (Ensures exact observation logic match)
obsInfo = agent.getObservationInfo;
actInfo = agent.getActionInfo;
env = HandoffEnvironment(params, obsInfo, actInfo);

% Simulation Params
maxSteps = 2500;
hysteresis_dB = 3.0; 
Tx_Power_dBm = 30;

% --- 2. INITIALIZATION ---
disp('Initializing Shared Simulation...');
obs = reset(env);

% Initialize Traditional Algorithm to start at the same cell as AI
tr_serving = env.serving_gNB;

% Logging
hist.time = zeros(maxSteps, 1);
hist.ai_cell = zeros(maxSteps, 1);
hist.tr_cell = zeros(maxSteps, 1);
hist.ai_rsrp = zeros(maxSteps, 1);
hist.tr_rsrp = zeros(maxSteps, 1);
hist.ai_load = zeros(maxSteps, 1);
hist.tr_load = zeros(maxSteps, 1);
hist.ai_ho = zeros(maxSteps, 1);
hist.tr_ho = zeros(maxSteps, 1);

disp('Running Side-by-Side Simulation...');

for i = 1:maxSteps
    % --- A. AI AGENT STEP ---
    % 1. Get Action from Agent (using the Environment's perfect observation)
    action = getAction(agent, {obs});
    
    % 2. Advance Environment (Moves UE, Fluctuates Load, AI Switches)
    % Note: env.step() handles the AI's handoff logic internally
    prev_ai_cell = env.serving_gNB;
    [nextObs, ~, isDone, loggedData] = step(env, action{1});
    
    % 3. Extract AI Status
    ai_serving = loggedData.Serving_gNB;
    
    % Denormalize RSRP for plotting: (NormVal * 100) - 140
    ai_rsrp_val = (nextObs(1) * 100) - 140; 
    ai_load_val = nextObs(8); % Load is index 8 in observation

    % --- B. TRADITIONAL LOGIC STEP ---
    % 1. Get "Ground Truth" Physics from Environment
    ue_pos = loggedData.UE_Position;
    % Access public property gNB_loads directly
    current_loads = env.gNB_loads; 
    
    % 2. Calculate RSRPs for Traditional Algo
    dists = vecnorm(params.gNB_positions - ue_pos, 2, 2);
    pl = 128.1 + 37.6 * log10(max(dists, 10)/1000);
    all_rsrps = Tx_Power_dBm - pl;
    
    % 3. A3 Event Logic (Neighbor > Serving + Hysteresis)
    srv_rsrp = all_rsrps(tr_serving);
    
    % Identify neighbors
    neigh_idxs = find((1:7)' ~= tr_serving);
    [best_neigh_rsrp, local_idx] = max(all_rsrps(neigh_idxs));
    best_neigh_id = neigh_idxs(local_idx);
    
    prev_tr_cell = tr_serving;
    
    % Check Handoff Condition
    if best_neigh_rsrp > (srv_rsrp + hysteresis_dB)
        tr_serving = best_neigh_id;
    end

    % --- C. LOGGING ---
    hist.time(i) = (i-1) * params.time_step;
    
    hist.ai_cell(i) = ai_serving;
    hist.tr_cell(i) = tr_serving;
    
    hist.ai_rsrp(i) = ai_rsrp_val;
    hist.tr_rsrp(i) = all_rsrps(tr_serving);
    
    hist.ai_load(i) = ai_load_val;
    hist.tr_load(i) = current_loads(tr_serving);
    
    % Record Handoff Events
    if i > 1
        if hist.ai_cell(i) ~= hist.ai_cell(i-1), hist.ai_ho(i) = 1; end
        if hist.tr_cell(i) ~= hist.tr_cell(i-1), hist.tr_ho(i) = 1; end
    end
    
    obs = nextObs;
    if isDone, break; end
end

% Trim Data
actualSteps = i;
fields = fieldnames(hist);
for k=1:numel(fields)
    hist.(fields{k}) = hist.(fields{k})(1:actualSteps);
end

% --- D. PLOTTING & METRICS ---
ai_cong_pct = sum(hist.ai_load > 0.8)/actualSteps * 100;
tr_cong_pct = sum(hist.tr_load > 0.8)/actualSteps * 100;
ai_ho_tot = sum(hist.ai_ho);
tr_ho_tot = sum(hist.tr_ho);

fprintf('\n=== COMPARISON RESULTS ===\n');
fprintf('METRIC          | AI AGENT | TRADITIONAL\n');
fprintf('----------------|----------|------------\n');
fprintf('Congestion Time | %5.2f%%   | %5.2f%%\n', ai_cong_pct, tr_cong_pct);
fprintf('Total Handovers | %5d    | %5d\n', ai_ho_tot, tr_ho_tot);
fprintf('Avg Signal      | %5.1f dB | %5.1f dB\n', mean(hist.ai_rsrp), mean(hist.tr_rsrp));

figure('Name', 'Unified Comparison', 'Color', 'w', 'Position', [50, 50, 1200, 800]);

% 1. RSRP
subplot(3,1,1); hold on; grid on;
plot(hist.time, hist.ai_rsrp, 'b-', 'LineWidth', 1.5);
plot(hist.time, hist.tr_rsrp, 'r--', 'LineWidth', 1.5);
yline(-110, 'k:', 'Drop Threshold');
ylabel('RSRP (dBm)'); title('1. Signal Quality Comparison');
legend('AI Agent', 'Traditional (A3)');

% 2. LOAD (Key Plot)
subplot(3,1,2); hold on; grid on;
plot(hist.time, hist.ai_load, 'b-', 'LineWidth', 1.5);
plot(hist.time, hist.tr_load, 'r--', 'LineWidth', 1.2);
yline(0.8, 'k--', 'Congestion (80%)', 'LineWidth', 2);
ylabel('Load (0-1)'); title('2. Traffic Load Comparison (Lower is Better)');
legend('AI Agent (Smart)', 'Traditional (Blind)');
ylim([0 1]);

% 3. Cell ID
subplot(3,1,3); hold on; grid on;
plot(hist.time, hist.ai_cell, 'b', 'LineWidth', 2);
plot(hist.time, hist.tr_cell, 'r:', 'LineWidth', 2);
ylabel('Cell ID'); xlabel('Time (s)'); title('3. Cell Selection');
legend('AI Selected', 'Traditional Selected');
ylim([0 8]);  