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
        SINR_good_threshold = 5;     
        RSRP_drop_threshold = -110;  % Call drops below this
        RSRP_weak_threshold = -95;   % "Danger zone" threshold

        % --- NEW: LOAD PARAMETERS ---
        gNB_loads           % Vector [7x1] storing load (0.0 to 1.0)
        congestion_threshold = 0.8; % 80% load is "Congested"
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
            
            % --- 1. LOAD FLUCTUATION ---
            % Randomly fluctuate loads slightly so they aren't static
            % (Simulates users joining/leaving cells)
            load_noise = (rand(num_gNBs, 1) - 0.5) * 0.05; 
            this.gNB_loads = this.gNB_loads + load_noise;
            
            % Clamp load between 0.0 and 1.0
            this.gNB_loads(this.gNB_loads > 1) = 1;
            this.gNB_loads(this.gNB_loads < 0) = 0;

            % --- 2. ACTION LOGIC ---
            if action == 2 % Trigger Handoff
                all_rsrp = this.getAllRSRPs();
                neighbor_indices = find((1:num_gNBs)' ~= this.serving_gNB);
                
                % Find best neighbor based on SIGNAL only (Traditional logic start)
                [~, best_local_idx] = max(all_rsrp(neighbor_indices));
                target_cell = neighbor_indices(best_local_idx);
                
                % NEW: Check if Target is Congested
                target_load = this.gNB_loads(target_cell);
                
                if target_load > this.congestion_threshold
                    % HEAVY PENALTY: Attempted handoff to a congested cell
                    handoff_penalty = -5; 
                    
                    % We allow the switch, but the agent learns it was a bad move.
                    % (In real 5G, admission control would likely block this).
                    this.serving_gNB = target_cell; 
                else
                    % OK: Normal Handoff cost
                    handoff_penalty = -2.0; 
                    this.serving_gNB = target_cell;
                end
            end
            
            % --- 3. PHYSICS UPDATE (Random Walk) ---
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
            
            % --- 4. REWARD CALCULATION ---
            all_rsrp = this.getAllRSRPs();
            serving_rsrp = all_rsrp(this.serving_gNB);
            
            isDone = false;
            
            if serving_rsrp < this.RSRP_drop_threshold
                % CALL DROP: Massive Penalty
                reward = -20; 
                isDone = true; 
            elseif serving_rsrp < this.RSRP_weak_threshold
                % DANGER ZONE: Penalty for staying on weak signal
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
            theta = 2 * pi * rand;
            r = this.grid_radius * sqrt(rand); 
            this.UE_position = [r * cos(theta), r * sin(theta)];
            
            % Random initial velocity
            this.UE_velocity_vector = (rand(1,2)-0.5) * 10;
            this.current_time = 0;
            
            % --- NEW: RANDOMIZE LOADS ---
            % Assign random load (0.0 to 1.0) to all 7 cells at start of episode
            this.gNB_loads = rand(size(this.gNB_positions, 1), 1);
            
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
            % 1. Get Serving Data
            rsrp_serving = all_rsrp(this.serving_gNB);
            load_serving = this.gNB_loads(this.serving_gNB);
            
            % 2. Identify Neighbors
            neighbor_idxs = find((1:length(all_rsrp))' ~= this.serving_gNB);
            
            % 3. Sort Neighbors by RSRP (Strongest First)
            [sorted_neigh_rsrps, sort_order] = sort(all_rsrp(neighbor_idxs), 'descend');
            
            % 4. Get Loads for those sorted neighbors (Crucial!)
            % We must pick the load of the *same* cell we just sorted by RSRP
            sorted_neigh_idxs = neighbor_idxs(sort_order);
            sorted_neigh_loads = this.gNB_loads(sorted_neigh_idxs);
            
            % 5. Pad if neighbors < 6
            num_neighbors_expected = 6; 
            if length(sorted_neigh_rsrps) < num_neighbors_expected
                pad_len = num_neighbors_expected - length(sorted_neigh_rsrps);
                
                % Pad RSRP with -150 (Silence)
                sorted_neigh_rsrps = [sorted_neigh_rsrps; -150 * ones(pad_len, 1)];
                
                % Pad Load with 0 (Empty)
                sorted_neigh_loads = [sorted_neigh_loads; zeros(pad_len, 1)];
            end
            
            % 6. Construct Final Observation Vector (Size 14)
            % [Serving_RSRP; Neighbor_RSRPs(6); Serving_Load; Neighbor_Loads(6)]
            obs = zeros(1 + num_neighbors_expected + 1 + num_neighbors_expected, 1);
            
            % RSRPs (Indices 1 to 7)
            obs(1) = rsrp_serving;
            obs(2:7) = sorted_neigh_rsrps(1:6);
            
            % Loads (Indices 8 to 14)
            obs(8) = load_serving;
            obs(9:14) = sorted_neigh_loads(1:6);
        end
        
        function sinr = calculateSINR(~, rsrp_dBm)
            signal = 10^((rsrp_dBm-30)/10);
            noise = 10^((-114-30)/10);
            interference = 0.5 * noise; 
            sinr = 10*log10(signal / (noise + interference));
        end
    end
end