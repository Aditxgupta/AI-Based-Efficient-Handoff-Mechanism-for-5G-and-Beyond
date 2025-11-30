% % analyze_hex_results.m
% =========================================================================
% PHASE 2 ANALYSIS: Metrics & Multi-Plot Visualization
% Calculates: Ping-Pongs, Handover Count, Call Drops, and RSRP Trends
% =========================================================================

clear; clc; close all;

% --- 1. LOAD AGENT & SETUP ---
if ~isfile('trainedHexAgent.mat')
    error('File "trainedHexAgent.mat" not found. Please run training first.');
end
load('trainedHexAgent.mat'); % Loads 'agent' and 'params'

% Force Deterministic Mode (Exploitation only)
agent.AgentOptions.EpsilonGreedyExploration.Epsilon = 0;

% Recreate Environment
obsInfo = agent.getObservationInfo;
actInfo = agent.getActionInfo;
env = HandoffEnvironment(params, obsInfo, actInfo);

% --- 2. RUN INFERENCE SIMULATION ---
disp('Running simulation to gather data...');
maxSteps = 2500;

% Data Logging Arrays
history.time = zeros(maxSteps, 1);
history.ue_pos = zeros(maxSteps, 2);
history.serving_cell = zeros(maxSteps, 1);
history.serving_rsrp = zeros(maxSteps, 1);
history.all_rsrps = zeros(maxSteps, 7); % Store RSRP for ALL 7 gNBs
history.call_dropped = false(maxSteps, 1);

obs = reset(env);
total_reward = 0;

for i = 1:maxSteps
    % 2.1 Get Action from Agent
    action = getAction(agent, {obs});
    
    % 2.2 Step Environment
    [nextObs, reward, isDone, loggedData] = step(env, action{1});
    total_reward = total_reward + reward;
    
    % 2.3 Log Data
    current_time = (i-1) * params.time_step;
    history.time(i) = current_time;
    history.ue_pos(i, :) = loggedData.UE_Position;
    history.serving_cell(i) = loggedData.Serving_gNB;
    history.serving_rsrp(i) = nextObs(1); % 1st element is Serving RSRP
    
    % RE-CALCULATE RSRPs for ALL gNBs (for plotting purposes)
    % (This logic mirrors the environment's private method)
    for k = 1:7
        d = norm(loggedData.UE_Position - params.gNB_positions(k,:));
        if d==0, d=1; end
        pl = 128.1 + 37.6 * log10(d/1000);
        history.all_rsrps(i, k) = 46 - pl;
    end
    
    % Check for Call Drop (-110 dBm threshold)
    if history.serving_rsrp(i) < -110
        history.call_dropped(i) = true;
    end
    
    obs = nextObs;
    if isDone, break; end
end

% Trim arrays to actual duration
actualSteps = i;
history.time = history.time(1:actualSteps);
history.ue_pos = history.ue_pos(1:actualSteps, :);
history.serving_cell = history.serving_cell(1:actualSteps);
history.serving_rsrp = history.serving_rsrp(1:actualSteps);
history.all_rsrps = history.all_rsrps(1:actualSteps, :);
history.call_dropped = history.call_dropped(1:actualSteps);

% --- 3. CALCULATE METRICS ---
handoff_indices = find(diff(history.serving_cell) ~= 0);
num_handovers = length(handoff_indices);
num_drops = sum(history.call_dropped);

% Ping-Pong Detection Algorithm
% Definition: A HO from Cell A -> B, followed by B -> A within 'T_pp' seconds
T_pp = 2.0; % Time window in seconds (e.g., 20 steps)
steps_pp = T_pp / params.time_step;
num_pingpongs = 0;

for k = 1:length(handoff_indices)-1
    idx_1 = handoff_indices(k);     % Time of 1st HO
    idx_2 = handoff_indices(k+1);   % Time of 2nd HO
    
    % Check time difference
    if (idx_2 - idx_1) <= steps_pp
        cell_A = history.serving_cell(idx_1);     % Cell before 1st HO
        cell_B = history.serving_cell(idx_1 + 1); % Cell after 1st HO
        cell_C = history.serving_cell(idx_2 + 1); % Cell after 2nd HO
        
        if cell_A == cell_C
            num_pingpongs = num_pingpongs + 1;
        end
    end
end

avg_rsrp = mean(history.serving_rsrp);

% --- 4. PRINT METRICS REPORT ---
fprintf('\n========================================\n');
fprintf('       PERFORMANCE METRICS REPORT       \n');
fprintf('========================================\n');
fprintf('Total Simulation Time: %.2f seconds\n', history.time(end));
fprintf('Total Handovers:       %d\n', num_handovers);
fprintf('Ping-Pong Events:      %d (Window: %.1fs)\n', num_pingpongs, T_pp);
fprintf('Call Drops:            %d\n', num_drops);
fprintf('Average Serving RSRP:  %.2f dBm\n', avg_rsrp);
fprintf('========================================\n');

% --- 5. GENERATE PLOTS ---

% FIGURE 1: Trajectory on Hex Grid
figure('Name', '1. Hexagonal Grid Trajectory', 'Color', 'w', 'Position', [100, 100, 600, 500]);
hold on; axis equal; grid on;
gNBs = params.gNB_positions;

% Draw Cells
viscircles(gNBs, 600 * ones(7,1), 'Color', [0.8 0.8 0.8], 'LineStyle', ':');
% Draw gNBs
plot(gNBs(:,1), gNBs(:,2), 'k^', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'gNBs');
text(gNBs(:,1)+50, gNBs(:,2)+50, string(1:7), 'FontSize', 12, 'FontWeight', 'bold');

% Draw Path with Color Mapping for Serving Cell
scatter(history.ue_pos(:,1), history.ue_pos(:,2), 15, history.serving_cell, 'filled');
colormap(jet(7)); c = colorbar; c.Label.String = 'Serving Cell ID';
caxis([1 7]);

% Mark Handoffs
if ~isempty(handoff_indices)
    ho_x = history.ue_pos(handoff_indices, 1);
    ho_y = history.ue_pos(handoff_indices, 2);
    plot(ho_x, ho_y, 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Handoff');
end
title('UE Trajectory & Serving Cell (Hex Grid)');
xlabel('X Position (m)'); ylabel('Y Position (m)');
legend('Location','best');

% FIGURE 2: Time Series Analysis
figure('Name', '2. Signal Analysis', 'Color', 'w', 'Position', [750, 100, 800, 600]);

% Subplot 2.1: RSRP Levels
subplot(2,1,1); hold on; grid on;
colors = lines(7);
% Plot all neighbor signals faintly
for k = 1:7
    plot(history.time, history.all_rsrps(:,k), 'Color', [colors(k,:) 0.3], 'LineWidth', 1, 'HandleVisibility', 'off');
end
% Plot Serving Cell RSRP boldly
plot(history.time, history.serving_rsrp, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Serving Cell RSRP');

% Mark Handoffs vertical lines
for k = 1:length(handoff_indices)
    xline(history.time(handoff_indices(k)), 'r:', 'LineWidth', 1.5);
end
yline(-110, 'r--', 'Drop Threshold', 'LabelHorizontalAlignment', 'left');

ylabel('RSRP (dBm)');
title('RSRP Levels Over Time');
legend('Location', 'best');
ylim([-130 -50]);

% Subplot 2.2: Serving Cell ID
subplot(2,1,2); hold on; grid on;
plot(history.time, history.serving_cell, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Cell ID');
title('Serving Cell ID (Stability Check)');
yticks(1:7);
ylim([0 8]);

% Add Text Box with Metrics to the Plot
dim = [0.15 0.15 0.3 0.3];
str = {sprintf('Handovers: %d', num_handovers), ...
       sprintf('Ping-Pongs: %d', num_pingpongs), ...
       sprintf('Drops: %d', num_drops)};
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', 'BackgroundColor', 'w');

% disp('Analysis Complete. See Figures and Console Output.');