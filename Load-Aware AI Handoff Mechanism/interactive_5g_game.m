function interactive_5g_game()
% =========================================================================
% INTERACTIVE 5G HANDOFF SIMULATOR (MINI-GAME)
% Control the UE with Arrow Keys. 
% Watch the AI or Traditional algorithm react in real-time.
% =========================================================================

    clc; clearvars -except agent params; close all;

    % --- 1. CHOOSE MODE ---
    disp('=======================================');
    disp('   5G INTERACTIVE HANDOFF SIMULATOR');
    disp('=======================================');
    disp('1: AI Agent (Smart Load Balancing)');
    disp('2: Traditional Algorithm (Blind to Load)');
    mode = input('Select Mode (1 or 2): ');
    
    if mode ~= 1 && mode ~= 2
        error('Invalid selection. Choose 1 or 2.');
    end

    % --- 2. LOAD AI IF NEEDED ---
    if mode == 1
        if ~isfile('trainedHexAgent.mat')
            error('trainedHexAgent.mat not found. Please train the AI first.');
        end
        load('trainedHexAgent.mat', 'agent');
        agent.AgentOptions.EpsilonGreedyExploration.Epsilon = 0; % Deterministic
        disp('AI Agent Loaded successfully.');
    end

    % --- 3. SETUP PHYSICS & PARAMETERS ---
    ISD = 1000; 
    gNB_positions = [0, 0]; 
    theta = deg2rad(0:60:300); 
    for i = 1:length(theta)
        gNB_positions = [gNB_positions; ISD * cos(theta(i)), ISD * sin(theta(i))];
    end
    
    Tx_Power_dBm = 30;
    hysteresis_dB = 3.0;
    time_step = 0.1;
    
    % Game State Variables
    ue_pos = [0, -500];
    ue_vel = [0, 0];
    global ue_accel is_running;
    ue_accel = [0, 0];
    is_running = true;
    
    gNB_loads = rand(7, 1);
    
    % Connect to initial cell
    dists = vecnorm(gNB_positions - ue_pos, 2, 2);
    pl = 128.1 + 37.6 * log10(max(dists, 10)/1000);
    [~, serving_cell] = max(Tx_Power_dBm - pl);

    % --- 4. SETUP GUI (THE "GAME" WINDOW) ---
    fig = figure('Name', '5G Handoff Game', 'Color', 'w', 'Position', [100, 100, 1200, 700], ...
                 'WindowKeyPressFcn', @keyDown, 'WindowKeyReleaseFcn', @keyUp);
    
    % Subplot 1: The Map
    ax_map = subplot(2, 2, [1 3]); hold on; axis equal; grid on;
    viscircles(gNB_positions, 600 * ones(7,1), 'Color', [0.8 0.8 0.8], 'LineStyle', ':');
    plot(gNB_positions(:,1), gNB_positions(:,2), 'k^', 'MarkerSize', 12, 'LineWidth', 2);
    text(gNB_positions(:,1)+50, gNB_positions(:,2)+50, string(1:7), 'FontSize', 14, 'FontWeight', 'bold');
    
    h_ue = plot(ue_pos(1), ue_pos(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    h_trail = plot(ue_pos(1), ue_pos(2), 'r-', 'LineWidth', 1); % Trail
    h_link = plot([ue_pos(1), gNB_positions(serving_cell, 1)], ...
                  [ue_pos(2), gNB_positions(serving_cell, 2)], 'b-', 'LineWidth', 2);
    
    title(ax_map, 'Drive UE with ARROW KEYS (Press "Q" to Quit)');
    xlim([-2000 2000]); ylim([-2000 2000]);
    
    % Subplot 2: RSRP Monitor
    ax_rsrp = subplot(2, 2, 2); hold on; grid on;
    h_rsrp = plot(0, 0, 'b-', 'LineWidth', 2);
    yline(-110, 'r--', 'Drop Threshold');
    title(ax_rsrp, 'Serving Cell RSRP'); ylabel('dBm');
    ylim([-130 -40]); xlim([0 100]);
    
    % Subplot 3: Load Monitor
    ax_load = subplot(2, 2, 4); hold on; grid on;
    h_load = plot(0, 0, 'm-', 'LineWidth', 2);
    yline(0.8, 'r--', 'Congestion (80%)', 'LineWidth', 2);
    title(ax_load, 'Serving Cell Load'); ylabel('Load (0-1)');
    ylim([0 1]); xlim([0 100]);

    % Dynamic Info Text
    info_text = annotation('textbox', [0.15 0.8 0.3 0.1], 'String', '', 'FitBoxToText', 'on', ...
        'BackgroundColor', 'w', 'FontSize', 12, 'FontWeight', 'bold');
    
    if mode == 1
        set(info_text, 'EdgeColor', 'b', 'Color', 'b');
    else
        set(info_text, 'EdgeColor', 'r', 'Color', 'r');
    end

    % History buffers for scrolling plots
    hist_len = 100;
    rsrp_hist = nan(1, hist_len);
    load_hist = nan(1, hist_len);
    trail_x = []; trail_y = [];

    disp('GAME STARTED! Click the figure window and use your Arrow Keys.');

    % --- 5. THE GAME LOOP ---
    step_count = 0;
    while is_running && ishandle(fig)
        step_count = step_count + 1;
        
        % 1. PHYSICS UPDATE (Player Control)
        % Apply keyboard acceleration to velocity
        ue_vel = ue_vel + ue_accel * 2.0; 
        
        % Friction / Speed limits
        speed = norm(ue_vel);
        if speed > 30, ue_vel = (ue_vel/speed)*30; end
        if speed > 0 && norm(ue_accel) == 0, ue_vel = ue_vel * 0.95; end % Natural braking
        
        ue_pos = ue_pos + ue_vel * time_step;
        
        % Trail update
        trail_x = [trail_x, ue_pos(1)]; trail_y = [trail_y, ue_pos(2)];
        if length(trail_x) > 200, trail_x(1)=[]; trail_y(1)=[]; end
        
        % 2. ENVIRONMENT UPDATE (Loads)
        gNB_loads = gNB_loads + (rand(7, 1) - 0.5) * 0.05;
        gNB_loads(gNB_loads > 1) = 1; gNB_loads(gNB_loads < 0) = 0;
        
        % 3. CALCULATE SIGNALS
        dists = vecnorm(gNB_positions - ue_pos, 2, 2);
        pl = 128.1 + 37.6 * log10(max(dists, 10)/1000);
        all_rsrps = Tx_Power_dBm - pl;
        srv_rsrp = all_rsrps(serving_cell);
        
        % 4. HANDOFF LOGIC
        if mode == 1 % AI AGENT LOGIC
            neigh_idxs = find((1:7)' ~= serving_cell);
            [sorted_rsrp, sort_idx] = sort(all_rsrps(neigh_idxs), 'descend');
            sorted_loads = gNB_loads(neigh_idxs(sort_idx));
            
            % Pad Observation
            if length(sorted_rsrp) < 6
                pad = 6 - length(sorted_rsrp);
                sorted_rsrp = [sorted_rsrp; -140*ones(pad,1)];
                sorted_loads = [sorted_loads; zeros(pad,1)];
            end
            
            % Normalize [0,1]
            norm_srv_rsrp = max(0, min(1, (srv_rsrp + 140)/100));
            norm_neigh_rsrp = max(0, min(1, (sorted_rsrp(1:6) + 140)/100));
            
            obs = [norm_srv_rsrp; norm_neigh_rsrp; gNB_loads(serving_cell); sorted_loads(1:6)];
            
            % Ask Agent
            action = getAction(agent, {obs});
            if action{1} == 2 % Switch
                cands = neigh_idxs(gNB_loads(neigh_idxs) < 0.8);
                if ~isempty(cands)
                    [~, best] = max(all_rsrps(cands));
                    serving_cell = cands(best);
                else
                    [~, best] = max(all_rsrps(neigh_idxs));
                    serving_cell = neigh_idxs(best);
                end
            end
            
        else % TRADITIONAL LOGIC (A3)
            neigh_idxs = find((1:7)' ~= serving_cell);
            [best_neigh_rsrp, local_idx] = max(all_rsrps(neigh_idxs));
            
            if best_neigh_rsrp > (srv_rsrp + hysteresis_dB)
                serving_cell = neigh_idxs(local_idx);
            end
        end
        
        % 5. UPDATE SCROLLING DATA
        rsrp_hist = [rsrp_hist(2:end), all_rsrps(serving_cell)];
        load_hist = [load_hist(2:end), gNB_loads(serving_cell)];
        
        % 6. UPDATE VISUALS
        set(h_ue, 'XData', ue_pos(1), 'YData', ue_pos(2));
        set(h_trail, 'XData', trail_x, 'YData', trail_y);
        set(h_link, 'XData', [ue_pos(1), gNB_positions(serving_cell, 1)], ...
                    'YData', [ue_pos(2), gNB_positions(serving_cell, 2)]);
                
        set(h_rsrp, 'YData', rsrp_hist, 'XData', 1:hist_len);
        set(h_load, 'YData', load_hist, 'XData', 1:hist_len);
        
        % Change load color if congested
        if load_hist(end) > 0.8, set(h_load, 'Color', 'r'); else, set(h_load, 'Color', 'm'); end
        
        % Update Info Box
        if mode == 1, mode_str = 'AI AGENT'; else, mode_str = 'TRADITIONAL'; end
        status = sprintf('MODE: %s\nServing Cell: %d\nRSRP: %.1f dBm\nLoad: %.0f%%', ...
            mode_str, serving_cell, all_rsrps(serving_cell), gNB_loads(serving_cell)*100);
        set(info_text, 'String', status);
        
        % Render frame (Target ~30 FPS)
        drawnow limitrate;
        pause(0.03); 
    end
    
    if ishandle(fig), close(fig); end
    disp('Simulation ended.');

    % --- KEYBOARD CALLBACK FUNCTIONS ---
    function keyDown(~, event)
        switch event.Key
            case 'uparrow',    ue_accel(2) = 5;
            case 'downarrow',  ue_accel(2) = -5;
            case 'leftarrow',  ue_accel(1) = -5;
            case 'rightarrow', ue_accel(1) = 5;
            case 'q',          is_running = false;
        end
    end

    function keyUp(~, event)
        switch event.Key
            case 'uparrow',    if ue_accel(2) > 0, ue_accel(2) = 0; end
            case 'downarrow',  if ue_accel(2) < 0, ue_accel(2) = 0; end
            case 'leftarrow',  if ue_accel(1) < 0, ue_accel(1) = 0; end
            case 'rightarrow', if ue_accel(1) > 0, ue_accel(1) = 0; end
        end
    end
end