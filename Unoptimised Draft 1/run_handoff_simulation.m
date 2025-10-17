% AI-Based Efficient Handoff Mechanism for 5G and Beyond
% =======================================================
% Main script to configure, train, and evaluate the RL agent.

% --- 1. SETUP ---
clear; clc; close all;

% Define simulation parameters
params.gNB_positions = [0, 0; 1000, 0; 2000, 0]; % In meters
params.UE_velocity = 15; % m/s (54 km/h)
params.UE_trajectory_start = [-200, 50];
params.time_step = 0.1; % Agent makes a decision every 100 ms

% --- 2. CREATE 5G HANDOFF ENVIRONMENT ---
% Define observation and action spaces
obsInfo = rlNumericSpec([3 1], 'LowerLimit', -140, 'UpperLimit', -44, ...
    'Name', 'Handoff Obs', 'Description', 'RSRP_serving, RSRP_neigh1, RSRP_neigh2');
actInfo = rlFiniteSetSpec([1 2], 'Name', 'Handoff Action');

% Create the environment object from the custom class file
env = HandoffEnvironment(params, obsInfo, actInfo);

% --- 3. CREATE RL AGENT (DQN) ---
statePath = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'state')
    fullyConnectedLayer(64, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(32, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(length(actInfo.Elements), 'Name', 'output')];

% =========================================================================
% CORRECTED LINE: Use rlQValueRepresentation for older MATLAB versions
% =========================================================================
critic = rlQValueRepresentation(statePath, obsInfo, actInfo); 
% =========================================================================

agent = rlDQNAgent(critic);
agent.AgentOptions.EpsilonGreedyExploration.EpsilonDecay = 1e-4;

% --- 4. TRAIN THE AGENT ---
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 500, ...
    'MaxStepsPerEpisode', 2500, ...
    'ScoreAveragingWindowLength', 20, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 200, ... % Stop when reward is consistently high
    'Verbose', true, ...
    'Plots', 'training-progress');

trainingStats = train(agent, env, trainOpts);

% In run_handoff_simulation.m

% --- 5. SIMULATE AND VISUALIZE RESULTS ---
disp('Simulating trained agent to generate results...');
simOpts = rlSimulationOptions('MaxSteps', 2500);

% Disable exploration for the final simulation to see the learned policy
agent.AgentOptions.EpsilonGreedyExploration.Epsilon = 0;
experience = sim(env, agent, simOpts);

% Package all necessary data into a single struct for plotting
results.experience = experience;
results.params = params;
results.agent = agent;
results.env = env;

% Call the dedicated plotting function
plotHandoffResults(results);

disp('Simulation and plotting complete.');