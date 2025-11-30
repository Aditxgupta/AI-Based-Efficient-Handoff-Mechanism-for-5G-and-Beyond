# AI-Based Efficient Handoff Mechanism for 5G and Beyond

A Deep Reinforcement Learning based mobility optimization system for 5G Heterogeneous Networks. The project simulates a User Equipment moving through a realistic 7-cell hexagonal grid and uses a Deep Q-Network agent to make handover decisions that reduce ping-pong events and avoid call drops.

---

## 📝 Project Synopsis

Traditional handover algorithms such as A3 Events with Hysteresis depend on static thresholds. These thresholds often fail to adapt to fast-changing 5G environments, causing:

* Unnecessary frequent switches
* Late handovers leading to radio link failures

This project addresses the issue by:

* **Simulating a realistic 7-cell hexagonal network** with overlaps and weak-signal areas
* **Modeling dynamic UE mobility** using a Random Walk with Inertia approach
* **Training a DQN agent** to observe serving and neighbor cell RSRP values and decide the best handover timing based on long-term rewards

---

## ⚡ Key Features

* **Hexagonal Grid Topology**
  Seven gNodeBs arranged in a standard honeycomb pattern.

* **Random Walk Mobility**
  Includes speed variation, acceleration, and random direction changes for realistic movement.

* **Penalty-Based Reward Function**
  Penalizes unnecessary switching. Achieved a ping-pong rate of 0%.

* **Data Logging and Visualization**

  * Real-time UE trajectory plots
  * RSRP time-series graphs
  * CSV export (simulation_log.csv) for further analysis

---

## 📂 Repository Structure

```
├── Hexagonal_Training_Environment/
│   ├── HandoffEnvironment.m
│   ├── run_handoff_simulation.m
│   ├── analyze_hex_results.m
│   ├── analyze_hex_results_with_export.m
│   └── trainedHexAgent.mat
│
└── Initial Model Inference/
```

---

## 🚀 Getting Started

### Prerequisites

* MATLAB R2022a or newer
* Reinforcement Learning Toolbox
* Deep Learning Toolbox
* Optional: 5G Toolbox

---

### How to Run the Simulation

#### 1. Train the Agent (Optional)

To train from scratch:

1. Open `run_handoff_simulation.m`
2. Run the script
3. Training Manager will launch and stop after convergence
4. The agent will be saved as `trainedHexAgent.mat`

#### 2. Run Inference and Visualize Results (Recommended)

1. Ensure `trainedHexAgent.mat` is available in your path
2. Open `analyze_hex_results_with_export.m`
3. Run the script

---

## 📤 Output

The analysis script generates:

* **Console Report**
  Total handovers, ping-pong events, call drops

* **Visualization Map**
  UE path with marked handover locations

* **Signal Strength Plot**
  RSRP trends for all seven base stations

* **simulation_log.csv**
  Contains timestamp, coordinates, connected cell ID, and metrics for each time step

---

## 📊 Results Summary

| Metric         | Traditional A3 Algorithm | DQN Agent |
| -------------- | ------------------------ | --------- |
| Ping-Pong Rate | High (above 15 percent)  | 0 percent |
| Handover Count | High and unstable        | Optimized |
| Call Drop Rate | Low                      | Low       |

The agent learns to maintain the serving cell even if a neighbor is slightly stronger, switching only when required. This behavior resembles an adaptive hysteresis effect learned through experience.

---

## 👥 Authors

* **Aditya B Gupta (1DS22ET003)**
* **Asadulla MA Ansari (1DS22ET012)**
* **Shreya M R (1DS22ET085)**
* **Tanvi Patidar (1DS22ET097)**

Department of Electronics and Telecommunication Engineering
Dayananda Sagar College of Engineering, Bengaluru
