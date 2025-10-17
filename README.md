# AI-Based Efficient Handoff Mechanism for 5G and Beyond

This repository contains the MATLAB implementation for the project "AI-Based Efficient Handoff Mechanism for 5G and Beyond". This project aims to develop an intelligent handoff system for 5G and 6G networks using Artificial Intelligence. Traditional handoff mechanisms, which are often based on static signal strength triggers, struggle with the complexity of modern network environments, leading to issues like dropped calls and unnecessary handoffs. This project addresses these challenges by training an AI agent to make smarter, more adaptive handoff decisions in real-time.

## 📝 Project Synopsis

The core objective of this project is to enhance handoff efficiency and Quality of Service (QoS) by integrating AI into the decision-making process. The proposed solution involves creating a simulated 5G/6G network environment to train a reinforcement learning agent. This agent is designed to learn from a variety of contextual features, such as signal quality (RSRP), user mobility, and cell load, to predict the optimal time and target for a handoff. By doing so, the project aims to minimize handoff failures and reduce the "ping-pong" effect, where a user is rapidly switched back and forth between two cells.

## 📂 Repository Structure

This repository is organized into two main folders, representing different stages of the project:

1.  **Unoptimised Draft 1**
    This folder contains the initial MATLAB scripts for training a Deep Q-Network (DQN) agent from scratch. The code in this directory is considered a first draft and has known optimization issues within the `HandoffEnvironment.m` file. As a result, the training process (`run_handoff_simulation.m`) may not consistently produce a perfectly optimized agent. This version of the model is trained using only the Reference Signal Received Power (RSRP) as the primary metric for making handoff decisions.

2.  **Initial Model Inference**
    This folder contains a pre-trained and saved DQN agent (`trainedHandoffAgent.mat`) that has successfully learned to perform handoffs as expected. Due to the optimization challenges in the training script, this stable version of the agent is provided to demonstrate the desired handoff behavior. The scripts in this folder are configured for inference only, meaning you can run simulations with the pre-trained agent but not retrain it.

## 🚀 Getting Started

To run the simulations, you will need MATLAB with the **Reinforcement Learning Toolbox**.

### Running the Pre-Trained Model (Inference)

To see the optimized handoff agent in action, follow these steps:

1.  Navigate to the `Initial Model Inference` directory.
2.  Open the `run_inference_simulation.m` script in MATLAB.
3.  Run the script. This will load the pre-trained agent and execute a deterministic simulation, visualizing the handoff policy and generating plots for analysis.

### Training the Model (Unoptimised)

If you wish to experiment with the training process, you can use the scripts in the `Unoptimised Draft 1` folder:

1.  Navigate to the `Unoptimised Draft 1` directory.
2.  Open the `run_handoff_simulation.m` script in MATLAB.
3.  Run the script. This will set up the environment and begin training a new DQN agent. **Note**: As mentioned, this version is unoptimized, and the training outcomes may vary.

## 🛠️ Methodology and Future Work

The current implementation uses a Deep Q-Network (DQN), a popular reinforcement learning algorithm, to train the handoff agent. The agent learns a policy by interacting with the simulated 5G environment, receiving rewards for maintaining good signal quality and penalties for poor performance.

As the project progresses, we plan to enhance the model by incorporating additional metrics beyond RSRP, as outlined in the project synopsis. Future work will involve integrating features such as:

* UE velocity and trajectory
* Cell load and congestion
* Quality of Service (QoS) indicators

These additions will enable the agent to make more sophisticated and context-aware decisions, further improving the efficiency and reliability of handoffs in 5G and beyond networks.

## 🧑‍💻 Authors

This project is being developed by:

* Aditya B Gupta (1DS22ET003)
* Asadulla MA Ansari (1DS22ET012)
* Shreya M R (1DS22ET085)
* Tanvi Patidar (1DS22ET097)
