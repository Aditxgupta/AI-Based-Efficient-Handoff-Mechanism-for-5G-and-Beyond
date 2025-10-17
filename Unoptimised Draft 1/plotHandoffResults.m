% plotHandoffResults.m
% =================================================================
% CORRECTED VERSION v2 (Fixes field name error)
% =================================================================

function plotHandoffResults(results)
    
    experience = results.experience;
    params = results.params;
    
    % =================================================================
    % THIS IS THE CORRECTED LINE
    % The field name comes from the 'Name' property of obsInfo.
    % 'Handoff Obs' becomes 'HandoffObs'.
    % =================================================================
    rsrp_log = squeeze(experience.Observation.HandoffObs.Data);
    numSteps = size(rsrp_log, 2);
    
    disp('Re-running simulation to log detailed data for plotting...');
    env = results.env;
    agent = results.agent;
    
    initialObs = reset(env);
    ue_positions_x = zeros(1, numSteps);
    serving_gnb_log = zeros(1, numSteps);
    
    ue_positions_x(1) = env.UE_position(1);
    serving_gnb_log(1) = env.serving_gNB;
    
    obs = initialObs;
    for i = 2:numSteps
        action = getAction(agent, {obs});
        [obs, ~, ~, ~] = step(env, action{1});
        ue_positions_x(i) = env.UE_position(1);
        serving_gnb_log(i) = env.serving_gNB;
    end
    disp('Data logging complete.');
    
    % --- Generate Plots (No changes below this line) ---
    
    figure('Name', 'UE Trajectory and Handoffs', 'Position', [50, 500, 700, 400]);
    hold on;
    gNB_positions = params.gNB_positions;
    plot(gNB_positions(:,1), gNB_positions(:,2), 'k^', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'gNBs');
    
    ue_y_pos = params.UE_trajectory_start(2);
    plot(ue_positions_x, repmat(ue_y_pos, 1, numSteps), 'b-', 'LineWidth', 1.5, 'DisplayName', 'UE Path');
    
    handoff_indices = find(diff(serving_gnb_log) ~= 0);
    handoff_x_positions = ue_positions_x(handoff_indices);
    
    if ~isempty(handoff_x_positions)
        scatter(handoff_x_positions(1), ue_y_pos, 120, 'rx', 'LineWidth', 2, 'DisplayName', 'Handoff Event');
        if length(handoff_x_positions) > 1
            scatter(handoff_x_positions(2:end), repmat(ue_y_pos, 1, length(handoff_x_positions)-1), 120, 'rx', 'LineWidth', 2, 'HandleVisibility', 'off');
        end
    end
    
    title('UE Trajectory and AI-Triggered Handoff Locations');
    xlabel('X-Position (m)');
    ylabel('Y-Position (m)');
    legend('show', 'Location', 'best');
    grid on;
    hold off;

    figure('Name', 'RSRP Levels and Serving Cell', 'Position', [760, 500, 700, 600]);
    
    ax1 = subplot(2,1,1);
    hold on;
    time_steps = (0:numSteps-1) * params.time_step;
    plot(time_steps, rsrp_log(1,:), 'LineWidth', 2, 'DisplayName', 'RSRP Serving');
    plot(time_steps, rsrp_log(2,:), '--', 'LineWidth', 1.5, 'DisplayName', 'RSRP Neighbor 1');
    plot(time_steps, rsrp_log(3,:), ':', 'LineWidth', 1.5, 'DisplayName', 'RSRP Neighbor 2');
    
    for i = 1:length(handoff_indices)
        xline(time_steps(handoff_indices(i)), 'r--', 'LineWidth', 1, 'Label', sprintf('HO to gNB-%d', serving_gnb_log(handoff_indices(i)+1)));
    end
    
    title('RSRP Levels Over Time');
    ylabel('RSRP (dBm)');
    legend('show');
    grid on;
    hold off;
    
    ax2 = subplot(2,1,2);
    plot(time_steps, serving_gnb_log, 'm-', 'LineWidth', 2);
    title('Serving gNB Over Time');
    xlabel('Time (s)');
    ylabel('Serving gNB ID');
    yticks(1:size(gNB_positions,1));
    ylim([0.5, size(gNB_positions,1) + 0.5]);
    grid on;

    linkaxes([ax1, ax2], 'x');
    xlim([0, time_steps(end)]);
end