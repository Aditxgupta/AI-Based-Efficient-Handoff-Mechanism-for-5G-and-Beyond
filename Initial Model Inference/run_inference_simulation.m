% run_inference_simulation.m
% =================================================================
% Loads a PRE-TRAINED reinforcement learning agent and runs a single,
% deterministic simulation to visualize its handoff policy.
%
% INSTRUCTIONS:
% 1. Ensure 'trainedHandoffAgent.mat' exists by running the main
%    training script and saving the 'agent' and 'params' variables.
% 2. Ensure 'HandoffEnvironment.m' and 'plotHandoffResults_Inference.m'
%    are in the MATLAB path.
% =================================================================

% --- 1. SETUP ---
clear; clc; close all;

disp('--- AI-Based Handoff Simulation (Inference-Only Mode) ---');

% --- 2. LOAD THE PRE-TRAINED AGENT AND PARAMETERS ---
filename = 'trainedHandoffAgent.mat';
if ~exist(filename, 'file')
    error(['Trained agent file not found: ''', filename, '''. ' ...
        'Please run your original training script and then save the ''agent'' ' ...
        'and ''params'' variables to this file using the command: ' ...
        'save(''' filename ''', ''agent'', ''params'');']);
end
disp(['Loading trained agent and parameters from ''', filename, '''...']);
load(filename, 'agent', 'params');

% --- 3. RECREATE THE ENVIRONMENT AND CONFIGURE AGENT FOR INFERENCE ---
% The observation and action info are stored inside the agent object
obsInfo = agent.getObservationInfo;
actInfo = agent.getActionInfo;

% Create a fresh instance of the environment using the loaded parameters
env = HandoffEnvironment(params, obsInfo, actInfo);

% CRITICAL: Set the agent to deterministic mode for consistent results.
% This turns off all random exploration (e.g., epsilon-greedy).
agent.AgentOptions.EpsilonGreedyExploration.Epsilon = 0;
disp('Agent configured for deterministic inference (Exploration is OFF).');

% --- 4. RUN A SINGLE, COMPLETE SIMULATION AND LOG ALL DATA ---
disp('Starting deterministic simulation...');

maxSteps = 2500; % A safe upper limit for steps in one episode
% Pre-allocate arrays to store the history of the simulation
ue_positions_x = zeros(1, maxSteps);
serving_gnb_log = zeros(1, maxSteps);
rsrp_log = zeros(obsInfo.Dimension(1), maxSteps);
time_vector = (0:maxSteps-1) * params.time_step;

% Start the episode
obs = reset(env);

% Log the initial state (t=0)
ue_positions_x(1) = env.UE_position(1);
serving_gnb_log(1) = env.serving_gNB;
rsrp_log(:, 1) = obs;

% Run the simulation step-by-step
for i = 2:maxSteps
    % Get the best action from the trained agent (no randomness)
    action = getAction(agent, {obs});
    
    % Apply action to the environment
    [nextObs, ~, isDone, ~] = step(env, action{1});
    
    % Log the results of this step
    ue_positions_x(i) = env.UE_position(1);
    serving_gnb_log(i) = env.serving_gNB;
    rsrp_log(:, i) = nextObs;
    
    % Prepare for the next step
    obs = nextObs;
    
    % If the episode is finished (e.g., UE reached end of path), stop the loop
    if isDone
        break;
    end
end
disp('Simulation finished.');

% Trim logs to the actual number of steps taken in the episode
actualSteps = i;
ue_positions_x = ue_positions_x(1:actualSteps);
serving_gnb_log = serving_gnb_log(1:actualSteps);
rsrp_log = rsrp_log(:, 1:actualSteps);
time_vector = time_vector(1:actualSteps);

% --- 5. VISUALIZE THE RESULTS ---
% Package all logged data into a single struct to pass to the plotting function
plotData.ue_positions_x = ue_positions_x;
plotData.serving_gnb_log = serving_gnb_log;
plotData.rsrp_log = rsrp_log;
plotData.time_vector = time_vector;
plotData.params = params; % Pass params for gNB locations, etc.

% Create and call a new, clean plotting function
disp('Generating plots...');
plotHandoffResults_Inference(plotData);

disp('--- Script complete. ---');