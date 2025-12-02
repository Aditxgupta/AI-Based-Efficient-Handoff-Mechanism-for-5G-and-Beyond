% run_traditional_handoff.m
% =========================================================================
% TRADITIONAL HANDOFF SIMULATION (A3 Event Logic)
% 1. Uses the same Hexagonal Environment physics as the AI.
% 2. Implements standard A3 Event logic with Hysteresis (3dB).
% 3. Generates FULL PLOTS (Trajectory & Multi-signal Time Series).
% 4. Exports 'simulation_log_traditional.csv'.
% =========================================================================

clear; clc; close all;

% --- 1. SETUP PARAMETERS (Must match AI simulation) ---
ISD = 1000; % Inter-Site Distance
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
hysteresis_dB = 3.0; % Standard hysteresis margin (e.g., 3dB)

% --- 2. INITIALIZE STATE ---
% Random Start
grid_radius = 1500;
theta_start = 2 * pi * rand;
r_start = grid_radius * sqrt(rand);
ue_pos = [r_start * cos(theta_start), r_start * sin(theta_start)];

% Random Velocity
ue_velocity = (rand(1,2)-0.5) * 10;
max_speed = 25; min_speed = 5;

% Connect to best initial cell
d_init = vecnorm(gNB_positions - ue_pos, 2, 2);
pl_init = 128.1 + 37.6 * log10(max(d_init, 10)/1000);
rsrp_init = 30 - pl_init; % 30 dBm Tx Power
[~, serving_cell] = max(rsrp_init); % Initial Serving Cell

% Data Logging
history.time = zeros(maxSteps, 1);
history.ue_pos_x = zeros(maxSteps, 1);
history.ue_pos_y = zeros(maxSteps, 1);
history.serving_cell = zeros(maxSteps, 1);
history.serving_rsrp = zeros(maxSteps, 1);
history.all_rsrps = zeros(maxSteps, 7); % Store ALL neighbor RSRPs
history.handoff_event = zeros(maxSteps, 1);
history.call_dropped = false(maxSteps, 1);

disp('Running Traditional Handoff Simulation...');

% --- 3. MAIN LOOP ---
for i = 1:maxSteps
    % A. Calculate RSRP for all cells
    dists = vecnorm(gNB_positions - ue_pos, 2, 2);
    dists(dists < 10) = 10; % Clamp min distance
    path_losses = 128.1 + 37.6 * log10(dists/1000);
    current_rsrps = 30 - path_losses;
    
    % B. Traditional Handoff Logic (Event A3)
    % Get current serving RSRP based on the 'serving_cell' variable
    serving_rsrp = current_rsrps(serving_cell);
    
    % Find strongest neighbor
    [best_neigh_rsrp, best_neigh_idx] = max(current_rsrps);
    
    % Check Condition: Neighbor > Serving + Hysteresis
    if best_neigh_idx ~= serving_cell
        if best_neigh_rsrp > (serving_rsrp + hysteresis_dB)
            % Execute Handoff
            serving_cell = best_neigh_idx;
            history.handoff_event(i) = 1;
        end
    end
    
    % C. Update Physics (Random Walk)
    accel = (rand(1,2) - 0.5) * 4; 
    ue_velocity = ue_velocity + accel;
    speed = norm(ue_velocity);
    if speed > max_speed, ue_velocity = (ue_velocity/speed)*max_speed; end
    if speed < min_speed, ue_velocity = (ue_velocity/speed)*min_speed; end
    
    % Boundary check
    if norm(ue_pos) > 1800
        ue_velocity = ue_velocity + (-ue_pos/norm(ue_pos) * 2);
    end
    
    ue_pos = ue_pos + ue_velocity * time_step;
    
    % D. Log Data
    history.time(i) = (i-1)*time_step;
    history.ue_pos_x(i) = ue_pos(1);
    history.ue_pos_y(i) = ue_pos(2);
    history.serving_cell(i) = serving_cell;
    history.serving_rsrp(i) = serving_rsrp;
    history.all_rsrps(i, :) = current_rsrps';
    
    if serving_rsrp < -110
        history.call_dropped(i) = true;
    end
end

% --- 4. CALCULATE METRICS ---
num_handovers = sum(history.handoff_event);
num_drops = sum(history.call_dropped);

% Ping-Pong Detection (Window 2.0s)
T_pp = 2.0; 
steps_pp = T_pp / time_step;
num_pingpongs = 0;
handoff_indices = find(history.handoff_event);

for k = 1:length(handoff_indices)-1
    idx1 = handoff_indices(k);
    idx2 = handoff_indices(k+1);
    if (idx2 - idx1) <= steps_pp
        prev_cell = history.serving_cell(idx1-1);
        next_next_cell = history.serving_cell(idx2);
        if prev_cell == next_next_cell
            num_pingpongs = num_pingpongs + 1;
        end
    end
end

% --- 5. EXPORT DATA ---
disp('Exporting Traditional Data to CSV...');
data_table = table(history.time, history.ue_pos_x, history.ue_pos_y, ...
                   history.serving_cell, history.serving_rsrp, ...
                   history.handoff_event, ...
                   'VariableNames', {'Time_s', 'UE_X_m', 'UE_Y_m', 'Serving_Cell_ID', 'Serving_RSRP_dBm', 'Handoff_Occurred'});

filename = 'simulation_log_traditional.csv';
writetable(data_table, filename);

% --- 6. GENERATE PLOTS (Matches AI Visualization) ---

% FIGURE 1: Trajectory Map
figure('Name', '1. Traditional Trajectory (Hex Grid)', 'Color', 'w', 'Position', [100, 100, 600, 500]);
hold on; axis equal; grid on;
viscircles(gNB_positions, 600 * ones(7,1), 'Color', [0.8 0.8 0.8], 'LineStyle', ':');
plot(gNB_positions(:,1), gNB_positions(:,2), 'k^', 'MarkerSize', 10, 'LineWidth', 2);
text(gNB_positions(:,1)+50, gNB_positions(:,2)+50, string(1:7), 'FontSize', 12, 'FontWeight', 'bold');

scatter(history.ue_pos_x, history.ue_pos_y, 15, history.serving_cell, 'filled');
colormap(jet(7)); c = colorbar; c.Label.String = 'Serving Cell ID'; caxis([1 7]);

if ~isempty(handoff_indices)
    plot(history.ue_pos_x(handoff_indices), history.ue_pos_y(handoff_indices), 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Handoff');
end
title(['Traditional Handover (A3 Hysteresis ' num2str(hysteresis_dB) 'dB)']);
xlabel('X Position (m)'); ylabel('Y Position (m)');

% FIGURE 2: Detailed Signal Analysis (The one you requested)
figure('Name', '2. Traditional Signal Analysis', 'Color', 'w', 'Position', [750, 100, 800, 600]);

% Subplot 2.1: RSRP Levels (Showing ALL Neighbors)
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
title(['RSRP Levels (Traditional A3, Hyst: ' num2str(hysteresis_dB) 'dB)']);
legend('Location', 'best');
ylim([-130 -50]);

% Subplot 2.2: Serving Cell ID
subplot(2,1,2); hold on; grid on;
plot(history.time, history.serving_cell, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Cell ID');
title('Serving Cell ID (Traditional)');
yticks(1:7);
ylim([0 8]);

% Add Text Box with Metrics
dim = [0.15 0.15 0.3 0.3];
str = {sprintf('Total Handovers: %d', num_handovers), ...
       sprintf('Ping-Pongs: %d', num_pingpongs), ...
       sprintf('Call Drops: %d', num_drops)};
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', 'BackgroundColor', 'w');

% Console Report
fprintf('\n=== TRADITIONAL METHOD RESULTS ===\n');
fprintf('Total Handovers: %d\n', num_handovers);
fprintf('Ping-Pongs:      %d\n', num_pingpongs);
fprintf('Call Drops:      %d\n', num_drops);
fprintf('Data saved to:   %s\n', filename);
fprintf('==================================\n');
