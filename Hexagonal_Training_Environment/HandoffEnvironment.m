classdef HandoffEnvironment < rl.env.MATLABEnvironment
    properties
        gNB_positions       % Matrix [N x 2]
        UE_velocity_vector  % Current Vector [vx, vy]
        time_step
        UE_position         % Current [x, y]
        serving_gNB         % Index of serving gNB
        current_time
        
        % Mobility & Grid Parameters
        max_speed = 25;     
        min_speed = 5;      
        acceleration_noise = 2; 
        grid_radius = 1500; % Area where UE can spawn
        
        % Signal Physics
        % REDUCED Tx Power from 46 to 30 dBm to force handovers
        Tx_Power_dBm = 30; 
        
        % Thresholds
        SINR_good_threshold = 5;     % Lowered slightly
        RSRP_drop_threshold = -110;  % Call drops below this
        RSRP_weak_threshold = -95;   % "Danger zone" threshold
    end
    
    methods
        function this = HandoffEnvironment(params, obsInfo, actInfo)
            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);
            
            this.gNB_positions = params.gNB_positions;
            this.UE_velocity_vector = params.UE_velocity_vector;
            this.time_step = params.time_step;
        end
        
        function [nextObservation, reward, isDone, loggedSignals] = step(this, action)
            num_gNBs = size(this.gNB_positions, 1);
            handoff_penalty = 0;
            
            % --- 1. ACTION LOGIC ---
            if action == 2 % Trigger Handoff
                handoff_penalty = -0.2; % Slightly higher cost to prevent jitter
                
                all_rsrp = this.getAllRSRPs();
                neighbor_indices = find((1:num_gNBs)' ~= this.serving_gNB);
                [~, best_local_idx] = max(all_rsrp(neighbor_indices));
                this.serving_gNB = neighbor_indices(best_local_idx);
            end
            
            % --- 2. PHYSICS UPDATE (Random Walk) ---
            this.current_time = this.current_time + this.time_step;
            
            % Add Random Noise (Inertia-based movement)
            accel = (rand(1,2) - 0.5) * 2 * this.acceleration_noise; 
            this.UE_velocity_vector = this.UE_velocity_vector + accel;
            
            % Speed Limits
            current_speed = norm(this.UE_velocity_vector);
            if current_speed > this.max_speed
                this.UE_velocity_vector = (this.UE_velocity_vector / current_speed) * this.max_speed;
            elseif current_speed < this.min_speed
                this.UE_velocity_vector = (this.UE_velocity_vector / current_speed) * this.min_speed;
            end
            
            % Boundary Check: If too far, steer back towards center
            if norm(this.UE_position) > this.grid_radius
                bias_to_center = -this.UE_position / norm(this.UE_position);
                this.UE_velocity_vector = this.UE_velocity_vector + (bias_to_center * 2.0);
            end

            % Move UE
            this.UE_position = this.UE_position + this.UE_velocity_vector * this.time_step;
            
            % --- 3. REWARD CALCULATION ---
            all_rsrp = this.getAllRSRPs();
            serving_rsrp = all_rsrp(this.serving_gNB);
            sinr = this.calculateSINR(serving_rsrp);
            
            isDone = false;
            
            if serving_rsrp < this.RSRP_drop_threshold
                % CALL DROP: Massive Penalty
                reward = -20; 
                isDone = true; 
            elseif serving_rsrp < this.RSRP_weak_threshold
                % DANGER ZONE: Penalty for staying on weak signal
                % This forces agent to look for a better neighbor
                reward = -1 + handoff_penalty; 
            else
                % GOOD SIGNAL: Positive Reward
                reward = 1 + handoff_penalty;
            end
            
            % Stop if simulation runs too long
            if this.current_time > 400
                isDone = true;
            end
            
            nextObservation = this.getObservation(all_rsrp);
            
            % Logging
            loggedSignals.Serving_gNB = this.serving_gNB;
            loggedSignals.UE_Position = this.UE_position;
        end
        
        function [initialObservation, loggedSignals] = reset(this)
            % --- RANDOM START POSITION ---
            % Pick a random angle and random radius within grid limits
            theta = 2 * pi * rand;
            r = this.grid_radius * sqrt(rand); % sqrt for uniform distribution
            this.UE_position = [r * cos(theta), r * sin(theta)];
            
            % Random initial velocity
            this.UE_velocity_vector = (rand(1,2)-0.5) * 10;
            this.current_time = 0;
            
            % Connect to best gNB at this random start location
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
                if distance < 10; distance = 10; end % Min distance clamp
                
                % 3GPP Path Loss (Urban)
                path_loss_dB = 128.1 + 37.6 * log10(distance/1000); 
                
                % REDUCED Tx Power creates "holes" in coverage
                all_rsrp(i) = this.Tx_Power_dBm - path_loss_dB; 
            end
        end

        function obs = getObservation(this, all_rsrp)
            rsrp_serving = all_rsrp(this.serving_gNB);
            neighbor_indices = find((1:length(all_rsrp))' ~= this.serving_gNB);
            neighbor_rsrps = sort(all_rsrp(neighbor_indices), 'descend');
            
            % Pad if neighbors < 6
            num_neighbors_expected = 6; 
            if length(neighbor_rsrps) < num_neighbors_expected
                padding = -150 * ones(num_neighbors_expected - length(neighbor_rsrps), 1);
                neighbor_rsrps = [neighbor_rsrps; padding];
            end
            
            obs = zeros(1 + num_neighbors_expected, 1);
            obs(1) = rsrp_serving;
            obs(2:end) = neighbor_rsrps(1:num_neighbors_expected);
        end
        
        function sinr = calculateSINR(~, rsrp_dBm)
            signal = 10^((rsrp_dBm-30)/10);
            noise = 10^((-114-30)/10);
            interference = 0.5 * noise; % Increased interference slightly
            sinr = 10*log10(signal / (noise + interference));
        end
    end
end
