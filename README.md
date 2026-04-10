# 5G Load-Aware AI Handoff Mechanism

This directory contains an advanced iteration of the AI-based handoff system. While the original model focused primarily on **Reference Signal Received Power (RSRP)** to maintain connectivity, this version introduces **Network Load Awareness**. The AI agent is trained to balance the trade-off between the strongest signal and the least congested cell to ensure optimal Quality of Service (QoS).

## 🌟 Key Enhancements

* **Traffic Load Integration**: The environment now simulates dynamic traffic loads for each gNodeB (0.0 to 1.0). High loads (above 80%) represent congested cells that can degrade user experience even if the signal is strong.
* **Congestion-Aware Reward Function**: The DQN agent receives a heavy penalty (-5) if it attempts to hand over to a congested cell, teaching it to "load balance" rather than just "signal hunt."
* **Expanded Observation Space**: The agent's input vector has been increased to 14 dimensions:
    * **Indices 1-7**: RSRP of the serving cell and the top 6 neighbors.
    * **Indices 8-14**: Current traffic load of the serving cell and those same 6 neighbors.
* **Interactive Mini-Game**: A new script allows users to manually control the UE via arrow keys to see how the AI reacts to real-time congestion and movement.

## 📂 Module Structure

* **`HandoffEnvironment.m`**: The core RL environment class updated with load fluctuation logic and congestion-based penalties.
* **`run_unified_comparison_load.m`**: A benchmarking script that runs the "Smart" AI against a "Blind" Traditional algorithm (which only sees signal strength) on the same trajectory.
* **`interactive_5g_game.m`**: A GUI-based simulator for manual testing of the handoff logic.
* **`trainedHexAgent.mat`**: The pre-trained weights for the load-aware DQN agent.

## 🚀 How to Use

### 1. Interactive Simulation (Game Mode)
Run `interactive_5g_game.m` to open the simulator. 
* **Controls**: Use **Arrow Keys** to move the UE.
* **Modes**: 
    1.  **AI Agent**: Switches cells based on both RSRP and Load.
    2.  **Traditional**: Switches based solely on the A3 event (strongest neighbor + hysteresis).

### 2. Performance Benchmarking
Run `run_unified_comparison_load.m` to generate a side-by-side performance analysis. The script provides:
* **Congestion Time %**: Total time spent connected to a cell over 80% load.
* **Handover Efficiency**: Total number of switches made.
* **Signal Quality**: Average RSRP maintained during the session.

## 📊 Comparative Results (Expected)

| Metric | Traditional (A3) | Load-Aware AI |
| :--- | :--- | :--- |
| **Congestion Awareness** | Blind (Connects to strongest gNB regardless of load) | Smart (Avoids gNBs > 80% load) |
| **Switching Logic** | Reactive (Threshold-based) | Proactive (Long-term reward maximization) |
| **User Experience** | Risk of high latency in crowded cells | Balanced throughput and signal stability |


---
**Authors**: Aditya B Gupta, Asadulla MA Ansari, Shreya M R, Tanvi Patidar.
**Department**: Electronics and Telecommunication Engineering, DSCE Bengaluru.
