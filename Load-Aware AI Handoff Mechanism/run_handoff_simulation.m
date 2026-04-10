% AI-Based Efficient Handoff Mechanism for 5G and Beyond
% =======================================================
% PHASE 4: Optimized Training (Load Aware & Congestion Control)
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

% Initial dummy velocity (will be randomized in env)
params.UE_velocity_vector = [10, 5]; 
params.UE_trajectory_start = [0, 0]; 
params.time_step = 0.1; 

% --- 2. CREATE ENVIRONMENT ---
% UPDATED: Observation dimension is now 14 
% (7 RSRP values + 7 Load values)
obsInfo = rlNumericSpec([14 1], 'LowerLimit', -150, 'UpperLimit', 1, ...
    'Name', 'HandoffObs', 'Description', 'RSRPs_and_Loads');
actInfo = rlFiniteSetSpec([1 2], 'Name', 'HandoffAction');

env = HandoffEnvironment(params, obsInfo, actInfo);

% --- 3. CREATE RL AGENT (DQN) ---
% Using a Deep Q-Network to handle the complex state space
statePath = [
    % UPDATED: Input layer must match observation dimension (14)
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

% Exploration Strategy: Start high to explore load penalties
agent.AgentOptions.EpsilonGreedyExploration.Epsilon = 1.0;
agent.AgentOptions.EpsilonGreedyExploration.EpsilonDecay = 5e-5;
agent.AgentOptions.EpsilonGreedyExploration.EpsilonMin = 0.01;
agent.AgentOptions.ExperienceBufferLength = 100000; 
agent.AgentOptions.MiniBatchSize = 128;

% --- 4. TRAIN THE AGENT ---
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 3000, ...      % Increased episodes for harder task
    'MaxStepsPerEpisode', 1500, ...
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1100, ... 
    'Verbose', true, ...
    'Plots', 'training-progress');

% Train the agent
trainingStats = train(agent, env, trainOpts);

% Save the trained agent
save('trainedHexAgent.mat', 'agent', 'params');

disp('Load-Aware Agent Trained & Saved.');