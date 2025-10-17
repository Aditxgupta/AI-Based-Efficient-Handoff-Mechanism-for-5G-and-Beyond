% plotHandoffResults_Inference.m
% This function is for plotting only. It takes a struct containing
% all pre-logged data from a simulation run.

function plotHandoffResults_Inference(plotData)
    
    % --- Unpack the data from the input struct ---
    ue_positions_x = plotData.ue_positions_x;
    serving_gnb_log = plotData.serving_gnb_log;
    rsrp_log = plotData.rsrp_log;
    time_vector = plotData.time_vector;
    params = plotData.params;
    
    % --- Plot 1: UE Trajectory and Handoffs ---
    figure('Name', 'UE Trajectory and AI-Triggered Handoffs', 'Position', [50, 500, 700, 400]);
    hold on;
    gNB_positions = params.gNB_positions;
    plot(gNB_positions(:,1), gNB_positions(:,2), 'k^', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'gNBs');
    ue_y_pos = params.UE_trajectory_start(2);
    plot(ue_positions_x, repmat(ue_y_pos, 1, length(ue_positions_x)), 'b-', 'LineWidth', 1.5, 'DisplayName', 'UE Path');
    
    handoff_indices = find(diff(serving_gnb_log) ~= 0);
    handoff_x_positions = ue_positions_x(handoff_indices);
    if ~isempty(handoff_x_positions)
        scatter(handoff_x_positions, repmat(ue_y_pos, 1, length(handoff_x_positions)), 120, 'rx', 'LineWidth', 2, 'DisplayName', 'Handoff Event');
    end
    
    title('UE Trajectory and AI-Triggered Handoff Locations');
    xlabel('X-Position (m)');
    ylabel('Y-Position (m)');
    legend('show', 'Location', 'best');
    grid on;
    axis equal;
    hold off;

    % --- Plot 2: RSRP Levels and Serving Cell ---
    figure('Name', 'RSRP Levels and Serving Cell', 'Position', [760, 500, 700, 600]);
    
    ax1 = subplot(2,1,1);
    hold on;
    plot(time_vector, rsrp_log(1,:), 'LineWidth', 2, 'DisplayName', 'RSRP Serving');
    plot(time_vector, rsrp_log(2,:), '--', 'LineWidth', 1.5, 'DisplayName', 'RSRP Neighbor 1');
    plot(time_vector, rsrp_log(3,:), ':', 'LineWidth', 1.5, 'DisplayName', 'RSRP Neighbor 2');
    
    for i = 1:length(handoff_indices)
        xline(time_vector(handoff_indices(i)), 'r--', 'LineWidth', 1, 'Label', sprintf('HO to gNB-%d', serving_gnb_log(handoff_indices(i)+1)));
    end
    
    title('RSRP Levels Over Time');
    ylabel('RSRP (dBm)');
    legend('show');
    grid on;
    hold off;
    
    ax2 = subplot(2,1,2);
    plot(time_vector, serving_gnb_log, 'm-', 'LineWidth', 2);
    title('Serving gNB Over Time');
    xlabel('Time (s)');
    ylabel('Serving gNB ID');
    yticks(1:size(gNB_positions,1));
    ylim([0.5, size(gNB_positions,1) + 0.5]);
    grid on;

    linkaxes([ax1, ax2], 'x');
    xlim([0, time_vector(end)]);
end