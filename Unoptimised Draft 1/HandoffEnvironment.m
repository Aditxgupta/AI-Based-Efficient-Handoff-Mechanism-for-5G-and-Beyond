% HandoffEnvironment.m
% =========================================================================
% CORRECTED AND IMPROVED VERSION v2 (Fixes Constructor Error)
% =========================================================================

classdef HandoffEnvironment < rl.env.MATLABEnvironment
    properties
        gNB_positions
        UE_velocity
        UE_start_pos
        time_step
        UE_position
        serving_gNB
        current_time
        SINR_good_threshold = 10;
        RSRP_drop_threshold = -110;
    end
    
    methods
        % --- CONSTRUCTOR (Function to initialize the environment) ---
        function this = HandoffEnvironment(params, obsInfo, actInfo)
            % =================================================================
            % THIS IS THE CORRECTED SECTION
            % =================================================================
            % Call the parent class constructor
            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);
            
            % Store simulation parameters
            this.gNB_positions = params.gNB_positions;
            this.UE_velocity = params.UE_velocity;
            this.UE_start_pos = params.UE_trajectory_start;
            this.time_step = params.time_step;
        end
        
        % --- STEP FUNCTION (Called at every time step) ---
        function [nextObservation, reward, isDone, loggedSignals] = step(this, action)
            num_gNBs = size(this.gNB_positions, 1);
            handoff_penalty = 0;
            
            if action == 2 % Action is to Handoff
                handoff_penalty = -0.1;
                all_rsrp = this.getAllRSRPs();
                neighbor_indices = find((1:num_gNBs)' ~= this.serving_gNB);
                if ~isempty(neighbor_indices)
                    neighbor_rsrps = all_rsrp(neighbor_indices);
                    [~, best_local_idx] = max(neighbor_rsrps);
                    best_global_neighbor_idx = neighbor_indices(best_local_idx);
                    this.serving_gNB = best_global_neighbor_idx;
                end
            end
            
            this.current_time = this.current_time + this.time_step;
            this.UE_position(1) = this.UE_start_pos(1) + this.UE_velocity * this.current_time;
            
            all_rsrp = this.getAllRSRPs();
            serving_rsrp = all_rsrp(this.serving_gNB);
            sinr = this.calculateSINR(serving_rsrp);
            
            if serving_rsrp < this.RSRP_drop_threshold
                reward = -10;
                isDone = true;
            elseif sinr > this.SINR_good_threshold
                reward = 1 + handoff_penalty;
                isDone = false;
            else
                reward = -0.5 + handoff_penalty;
                isDone = false;
            end
            
            if this.UE_position(1) > 2200
                isDone = true;
            end
            
            nextObservation = this.getObservation(all_rsrp);
            
            loggedSignals.Serving_gNB = this.serving_gNB;
            loggedSignals.UE_Position = this.UE_position;
        end
        
        % --- RESET FUNCTION (Called at the beginning of each episode) ---
        function [initialObservation, loggedSignals] = reset(this)
            this.UE_position = this.UE_start_pos;
            this.current_time = 0;
            
            all_rsrp = this.getAllRSRPs();
            [~, this.serving_gNB] = max(all_rsrp);
            
            initialObservation = this.getObservation(all_rsrp);
            
            loggedSignals.Serving_gNB = this.serving_gNB;
            loggedSignals.UE_Position = this.UE_position;
        end
    end
    
    methods (Access = private)
        function all_rsrp = getAllRSRPs(this)
            num_gNBs = size(this.gNB_positions, 1);
            all_rsrp = zeros(num_gNBs, 1);
            for i = 1:num_gNBs
                distance = norm(this.UE_position - this.gNB_positions(i,:));
                if distance == 0; distance = 1; end
                path_loss_dB = 128.1 + 37.6 * log10(distance/1000); 
                all_rsrp(i) = 46 - path_loss_dB;
            end
        end

        function obs = getObservation(this, all_rsrp)
            rsrp_serving = all_rsrp(this.serving_gNB);
            neighbor_indices = find((1:length(all_rsrp))' ~= this.serving_gNB);
            neighbor_rsrps = sort(all_rsrp(neighbor_indices), 'descend');
            
            obs = zeros(3,1);
            obs(1) = rsrp_serving;
            if ~isempty(neighbor_rsrps)
                num_neighbors_to_report = min(length(neighbor_rsrps), 2);
                obs(2:1+num_neighbors_to_report) = neighbor_rsrps(1:num_neighbors_to_report);
            end
        end
        
        function sinr = calculateSINR(~, rsrp_dBm)
            signal_power_linear = 10^((rsrp_dBm-30)/10);
            noise_power_linear = 10^((-114-30)/10);
            interference_linear = 0.1 * noise_power_linear;
            sinr = 10*log10(signal_power_linear / (noise_power_linear + interference_linear));
        end
    end
end