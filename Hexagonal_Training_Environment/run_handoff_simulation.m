% AI-Based Efficient Handoff Mechanism for 5G and Beyond
% =======================================================
% PHASE 3: Optimized Training (Random Start & Weak Signals)
% =======================================================

clear; clc; close all;

% --- 1. SETUP SIMULATION PARAMETERS ---
% 1.1 Generate Hexagonal Grid for 7 gNBs
ISD = 1000; % Inter-Site Distance (meters)
gNB_positions = [0, 0]; 
theta = deg2rad(0:60:300); 
for i = 1:length(theta)
    x = ISD * cos(theta(i));
    y = ISD * sin(theta(i));
    gNB_positions = [gNB_positions; x, y];
end

params.gNB_positions = gNB_positions;

% Note: UE_trajectory_start is NO LONGER SET HERE.
% It is randomized inside HandoffEnvironment.m -> reset()

% Initial dummy velocity (will be randomized in env)
params.UE_velocity_vector = [10, 5]; 
params.UE_trajectory_start = [0, 0]; % Placeholder (ignored by reset)
params.time_step = 0.1; 

% --- 2. CREATE ENVIRONMENT ---
obsInfo = rlNumericSpec([7 1], 'LowerLimit', -140, 'UpperLimit', -30, ...
    'Name', 'HandoffObs', 'Description', 'RSRP_serving + 6 neighbors');
actInfo = rlFiniteSetSpec([1 2], 'Name', 'HandoffAction');

env = HandoffEnvironment(params, obsInfo, actInfo);

% --- 3. CREATE RL AGENT (DQN) ---
% Using a slightly deeper network to learn the complex boundary logic
statePath = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'none', 'Name', 'state')
    fullyConnectedLayer(256, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(128, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(64, 'Name', 'fc3')
    reluLayer('Name', 'relu3')
    fullyConnectedLayer(length(actInfo.Elements), 'Name', 'output')];

critic = rlQValueRepresentation(statePath, obsInfo, actInfo); 
agent = rlDQNAgent(critic);

% Exploration Strategy: Start high, decay slower to let it find the "holes"
agent.AgentOptions.EpsilonGreedyExploration.Epsilon = 1.0;
agent.AgentOptions.EpsilonGreedyExploration.EpsilonDecay = 5e-5;
agent.AgentOptions.EpsilonGreedyExploration.EpsilonMin = 0.01;
agent.AgentOptions.ExperienceBufferLength = 100000; 
agent.AgentOptions.MiniBatchSize = 128;

% --- 4. TRAIN THE AGENT ---
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 2000, ...      % More episodes for the random start logic
    'MaxStepsPerEpisode', 1500, ...
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1200, ... 
    'Verbose', true, ...
    'Plots', 'training-progress');

% Train the agent
trainingStats = train(agent, env, trainOpts);

% Save the trained agent
save('trainedHexAgent.mat', 'agent', 'params');

disp('Optimized Hexagonal Agent Trained & Saved.');