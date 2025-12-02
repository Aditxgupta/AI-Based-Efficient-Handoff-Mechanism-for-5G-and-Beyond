function run_full_comparison()
% =========================================================================
% UNIFIED SIMULATION: AI Agent vs. Traditional Algorithm (FUNCTION ENV VERSION)
% 1. Creates an RL environment using FUNCTION HANDLES (No classdef required).
% 2. Loads the trained agent.
% 3. Runs comparison simulations.
% =========================================================================

    % --- 1. SETUP & LOAD ---
    clearvars -except params agent; % Clear workspace but keep safe vars if reloading
    clc; close all;

    if ~isfile('trainedHexAgent.mat')
        error('File "trainedHexAgent.mat" not found. Please train the agent first.');
    end
    
    % Load agent and parameters
    loadedData = load('trainedHexAgent.mat', 'agent', 'params');
    agent = loadedData.agent;
    params = loadedData.params;

    % Force Deterministic AI for Inference
    agent.AgentOptions.EpsilonGreedyExploration.Epsilon = 0;
    agent.AgentOptions.EpsilonGreedyExploration.EpsilonMin = 0;
    
    % --- 2. CREATE FUNCTION-BASED ENVIRONMENT ---
    % We use rlFunctionEnv to avoid 'classdef' issues in single files.
    % We pass 'params' into the step/reset functions using function handles.
    
    obsInfo = agent.getObservationInfo;
    actInfo = agent.getActionInfo;
    
    % Define handles that wrap the local functions with the parameters
    ResetHandle = @() envReset(params);
    StepHandle = @(Action, LoggedSignals) envStep(Action, LoggedSignals, params);
    
    % Create the environment object
    env = rlFunctionEnv(obsInfo, actInfo, StepHandle, ResetHandle);
    
    % --- 3. GENERATE TRAJECTORY ---
    disp('Generating random user trajectory...');
    
    % Reset environment to get random start state
    [~, initialState] = reset(env);
    
    maxSteps = 3000;
    history.time = zeros(maxSteps, 1);
    history.ue_pos = zeros(maxSteps, 2);
    history.all_rsrps = zeros(maxSteps, 7);
    
    % Simulation Loop to record physics trajectory (Agent stays passive)
    % We use the environment's internal state tracking
    currentState = initialState;
    
    for i = 1:maxSteps
        % Action 1 = Stay (Passive physics update)
        [~, ~, isDone, nextState] = step(env, 1);
        
        history.time(i) = (i-1) * params.time_step;
        history.ue_pos(i, :) = nextState.UE_Position;
        
        % Calculate RSRPs for all 7 gNBs (for plotting/comparison)
        history.all_rsrps(i, :) = calculateAllRSRPs(nextState.UE_Position, params.gNB_positions);
        
        if isDone, break; end
        % IMPORTANT: Update state for next step (rlFunctionEnv is stateless)
        % But since we are using the env object, it manages the call, 
        % however, our envStep function updates the struct.
        % Actually, we need to manually pass the state in this custom loop context?
        % No, the 'env' object in MATLAB RL Toolbox maintains the state internally 
        % between step() calls if it's an object. But rlFunctionEnv relies on the
        % LoggedSignals being passed back and forth.
        
        % Workaround for trajectory generation: We just used the step() function 
        % which updates the internal state of the 'env' wrapper.
    end
    
    actualSteps = i;
    % Trim data
    history.time = history.time(1:actualSteps);
    history.ue_pos = history.ue_pos(1:actualSteps, :);
    history.all_rsrps = history.all_rsrps(1:actualSteps, :);
    
    disp(['Trajectory ready. Duration: ' num2str(history.time(end)) 's']);

    % --- 4. RUN AI SIMULATION ---
    disp('Simulating AI Agent...');
    ai_serving_cell = zeros(actualSteps, 1);
    ai_serving_rsrp = zeros(actualSteps, 1);
    
    % Init AI: Connect to strongest at start
    [~, best_start_cell] = max(history.all_rsrps(1, :));
    curr_ai_cell = best_start_cell;
    
    for i = 1:actualSteps
        % 1. Construct Observation
        rsrps = history.all_rsrps(i, :);
        serving_rsrp = rsrps(curr_ai_cell);
        
        % Create sorted neighbor list for observation (Match training logic)
        neigh_idxs = find((1:7) ~= curr_ai_cell);
        neigh_vals = sort(rsrps(neigh_idxs), 'descend');
        if length(neigh_vals) < 6
            neigh_vals = [neigh_vals, -150 * ones(1, 6-length(neigh_vals))];
        end
        observation = [serving_rsrp; neigh_vals(:)];
        
        % 2. Get Action from Agent
        action = getAction(agent, {observation});
        
        % 3. Execute Logic
        if action{1} == 2 % Handoff
            [~, best_idx] = max(rsrps);
            curr_ai_cell = best_idx;
        end
        
        ai_serving_cell(i) = curr_ai_cell;
        ai_serving_rsrp(i) = rsrps(curr_ai_cell);
    end
    
    % --- 5. RUN TRADITIONAL SIMULATION ---
    disp('Simulating Traditional Algorithm...');
    trad_serving_cell = zeros(actualSteps, 1);
    trad_serving_rsrp = zeros(actualSteps, 1);
    
    % Init Traditional
    [~, best_start_trad] = max(history.all_rsrps(1, :));
    curr_trad_cell = best_start_trad;
    hysteresis = 3.0; % dB
    
    for i = 1:actualSteps
        rsrps = history.all_rsrps(i, :);
        srv_rsrp = rsrps(curr_trad_cell);
        [best_neigh_rsrp, best_neigh_idx] = max(rsrps);
        
        % A3 Event Logic
        if best_neigh_idx ~= curr_trad_cell
            if best_neigh_rsrp > (srv_rsrp + hysteresis)
                curr_trad_cell = best_neigh_idx;
            end
        end
        
        trad_serving_cell(i) = curr_trad_cell;
        trad_serving_rsrp(i) = rsrps(curr_trad_cell);
    end
    
    % --- 6. CALCULATE METRICS ---
    % Handoff Counts
    ai_ho = sum(diff(ai_serving_cell) ~= 0);
    trad_ho = sum(diff(trad_serving_cell) ~= 0);
    
    % Ping-Pongs
    ai_pp = count_pingpongs(ai_serving_cell, params.time_step);
    trad_pp = count_pingpongs(trad_serving_cell, params.time_step);
    
    % Drops (-110 dBm threshold)
    drop_thresh = -110;
    ai_drops = sum(ai_serving_rsrp < drop_thresh);
    trad_drops = sum(trad_serving_rsrp < drop_thresh);
    
    % Average RSRP
    ai_avg = mean(ai_serving_rsrp);
    trad_avg = mean(trad_serving_rsrp);
    
    % --- 7. PLOTTING ---
    figure('Name', 'Unified Comparison Dashboard', 'Color', 'w', 'Position', [50, 50, 1200, 800]);
    
    % Plot A: Trajectory
    subplot(2, 3, [1 4]); 
    hold on; axis equal; grid on;
    viscircles(params.gNB_positions, 600 * ones(7,1), 'Color', [0.8 0.8 0.8], 'LineStyle', ':');
    plot(params.gNB_positions(:,1), params.gNB_positions(:,2), 'k^', 'MarkerSize', 8, 'LineWidth', 2);
    % UE Path
    scatter(history.ue_pos(:,1), history.ue_pos(:,2), 15, ai_serving_cell, 'filled');
    colormap(jet(7)); c = colorbar; c.Label.String = 'AI Serving Cell'; caxis([1 7]);
    title('UE Trajectory & AI Decisions'); xlabel('X (m)'); ylabel('Y (m)');
    
    % Plot B: RSRP Comparison
    subplot(2, 3, [2 3]); 
    hold on; grid on;
    % Background faint lines
    colors = lines(7);
    for k=1:7, plot(history.time, history.all_rsrps(:,k), 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5); end
    % Main lines
    plot(history.time, ai_serving_rsrp, 'b-', 'LineWidth', 2, 'DisplayName', 'AI Agent');
    plot(history.time, trad_serving_rsrp, 'r--', 'LineWidth', 2, 'DisplayName', 'Traditional');
    yline(drop_thresh, 'k:', 'Drop Threshold');
    legend('Location','southwest'); title('Signal Quality (RSRP) Comparison'); ylabel('RSRP (dBm)'); ylim([-130 -40]);
    
    % Plot C: Bar Charts (Metrics)
    subplot(2, 3, 5);
    b = bar([ai_ho, trad_ho]);
    xticklabels({'AI', 'Traditional'}); title('Total Handovers'); ylabel('Count');
    b.FaceColor = 'flat'; b.CData(1,:) = [0 0 1]; b.CData(2,:) = [1 0 0];
    text(1:2, [ai_ho, trad_ho], string([ai_ho, trad_ho]), 'vert', 'bottom', 'horiz', 'center');
    
    subplot(2, 3, 6);
    b2 = bar([ai_pp, trad_pp]);
    xticklabels({'AI', 'Traditional'}); title('Ping-Pong Events'); ylabel('Count');
    b2.FaceColor = 'flat'; b2.CData(1,:) = [0 0 1]; b2.CData(2,:) = [1 0 0];
    text(1:2, [ai_pp, trad_pp], string([ai_pp, trad_pp]), 'vert', 'bottom', 'horiz', 'center');
    
    % --- 8. CONSOLE TABLE ---
    fprintf('\n======================================================\n');
    fprintf('             PERFORMANCE COMPARISON TABLE             \n');
    fprintf('======================================================\n');
    fprintf('| %-15s | %-12s | %-12s |\n', 'METRIC', 'AI AGENT', 'TRADITIONAL');
    fprintf('|-----------------|--------------|--------------|\n');
    fprintf('| Total Handovers | %-12d | %-12d |\n', ai_ho, trad_ho);
    fprintf('| Ping-Pongs      | %-12d | %-12d |\n', ai_pp, trad_pp);
    fprintf('| Call Drops      | %-12d | %-12d |\n', ai_drops, trad_drops);
    fprintf('| Avg RSRP (dBm)  | %-12.2f | %-12.2f |\n', ai_avg, trad_avg);
    fprintf('======================================================\n');

end

% =========================================================================
% LOCAL HELPER FUNCTIONS (Environment Dynamics)
% =========================================================================

% --- 1. Environment Step Function ---
function [NextObs, Reward, IsDone, NextState] = envStep(Action, State, params)
    % Unpack State
    ue_pos = State.UE_Position;
    ue_vel = State.UE_velocity_vector;
    serving_gNB = State.Serving_gNB;
    curr_time = State.Current_Time;
    
    % 1. Process Action
    handoff_penalty = 0;
    if Action == 2 % Handoff Triggered
        handoff_penalty = -0.2; % Stability cost
        % Switch to strongest
        all_rsrp = calculateAllRSRPs(ue_pos, params.gNB_positions);
        neighbor_idxs = find((1:7)' ~= serving_gNB);
        [~, best_local] = max(all_rsrp(neighbor_idxs));
        serving_gNB = neighbor_idxs(best_local);
    end
    
    % 2. Physics Update (Random Walk)
    curr_time = curr_time + params.time_step;
    % Add acceleration noise
    accel = (rand(1,2)-0.5) * 4; % 2 * acceleration_noise
    ue_vel = ue_vel + accel;
    % Speed Clamp
    speed = norm(ue_vel);
    max_speed = 25; min_speed = 5;
    if speed > max_speed, ue_vel = (ue_vel/speed)*max_speed; end
    if speed < min_speed, ue_vel = (ue_vel/speed)*min_speed; end
    % Boundary Logic (Steer back to center if > 1500m)
    if norm(ue_pos) > 1500
        ue_vel = ue_vel + (-ue_pos/norm(ue_pos) * 2.0);
    end
    ue_pos = ue_pos + ue_vel * params.time_step;
    
    % 3. Calculate Reward
    all_rsrp = calculateAllRSRPs(ue_pos, params.gNB_positions);
    serving_rsrp = all_rsrp(serving_gNB);
    
    IsDone = false;
    if serving_rsrp < -110
        Reward = -20; IsDone = true; % Drop
    elseif serving_rsrp < -95
        Reward = -1 + handoff_penalty; % Weak signal penalty
    else
        Reward = 1 + handoff_penalty; % Good signal reward
    end
    
    if curr_time > 400, IsDone = true; end
    
    % 4. Pack Next Observation
    NextObs = getObservation(all_rsrp, serving_gNB);
    
    % 5. Pack Next State (LoggedSignals)
    NextState.UE_Position = ue_pos;
    NextState.UE_velocity_vector = ue_vel;
    NextState.Serving_gNB = serving_gNB;
    NextState.Current_Time = curr_time;
end

% --- 2. Environment Reset Function ---
function [InitialObs, InitialState] = envReset(params)
    % Random Start Position
    theta = 2*pi*rand;
    r = 1500 * sqrt(rand);
    ue_pos = [r*cos(theta), r*sin(theta)];
    ue_vel = (rand(1,2)-0.5) * 10;
    
    % Connect to best cell
    all_rsrp = calculateAllRSRPs(ue_pos, params.gNB_positions);
    [~, serving_gNB] = max(all_rsrp);
    
    % Initial Observation
    InitialObs = getObservation(all_rsrp, serving_gNB);
    
    % Initial State Struct
    InitialState.UE_Position = ue_pos;
    InitialState.UE_velocity_vector = ue_vel;
    InitialState.Serving_gNB = serving_gNB;
    InitialState.Current_Time = 0;
end

% --- 3. Helper: RSRP Calculation ---
function all_rsrp = calculateAllRSRPs(ue_pos, gnb_pos)
    num_gNBs = size(gnb_pos, 1);
    all_rsrp = zeros(num_gNBs, 1);
    Tx_Power = 30; % dBm
    for i = 1:num_gNBs
        d = norm(ue_pos - gnb_pos(i,:));
        if d < 10, d = 10; end
        pl = 128.1 + 37.6 * log10(d/1000);
        all_rsrp(i) = Tx_Power - pl;
    end
end

% --- 4. Helper: Format Observation Vector ---
function obs = getObservation(all_rsrp, serving_idx)
    % [Serving, Neighbor1, Neighbor2, ..., Neighbor6] (Sorted)
    serving_val = all_rsrp(serving_idx);
    neigh_idxs = find((1:length(all_rsrp))' ~= serving_idx);
    neigh_vals = sort(all_rsrp(neigh_idxs), 'descend');
    
    % Pad to 6 neighbors if needed
    if length(neigh_vals) < 6
        neigh_vals = [neigh_vals; -150 * ones(6-length(neigh_vals), 1)];
    end
    obs = [serving_val; neigh_vals(1:6)];
end

% --- 5. Helper: Ping-Pong Counter ---
function pp = count_pingpongs(cell_log, dt)
    changes = find(diff(cell_log) ~= 0);
    pp = 0;
    window_steps = 2.0 / dt; % 2 seconds window
    for k = 1:length(changes)-1
        % If switched back to original cell within window
        if (changes(k+1) - changes(k)) < window_steps
            cell_A = cell_log(changes(k));
            cell_C = cell_log(changes(k+1)+1);
            if cell_A == cell_C
                pp = pp + 1;
            end
        end
    end
end
