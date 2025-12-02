# AI-Based Efficient Handoff Mechanism for 5G and Beyond

A Deep Reinforcement Learning based mobility optimization system for 5G Heterogeneous Networks. The project simulates a User Equipment moving through a realistic 7-cell hexagonal grid and uses a Deep Q-Network agent to make handover decisions that reduce ping-pong events and avoid call drops.

---

## 📝 Project Synopsis

Traditional handover algorithms such as A3 Events with Hysteresis depend on static thresholds. These thresholds often fail to adapt to fast-changing 5G environments, causing:

* Unnecessary frequent switches (Ping-Pong effect)
* Late handovers leading to radio link failures

This project addresses the issue by:

* **Simulating a realistic 7-cell hexagonal network** with overlaps and weak-signal areas.
* **Modeling dynamic UE mobility** using a Random Walk with Inertia approach.
* **Training a DQN agent** to observe serving and neighbor cell RSRP values and decide the best handover timing based on long-term rewards.
* **Benchmarking** the AI agent directly against standard Traditional Handoff algorithms.

---

## ⚡ Key Features

* **Hexagonal Grid Topology**
  Seven gNodeBs arranged in a standard honeycomb pattern with reduced transmission power to create realistic coverage gaps.

* **Random Walk Mobility**
  Includes speed variation, acceleration, and random direction changes for realistic movement.

* **Penalty-Based Reward Function**
  Penalizes unnecessary switching and call drops. Achieved a ping-pong rate of 0%.

* **Unified Comparison Dashboard**
  A dedicated script to run the AI Agent and Traditional Algorithm side-by-side on the *exact same* user trajectory for fair performance evaluation.

* **Data Logging and Visualization**
  * Real-time UE trajectory plots
  * RSRP time-series graphs
  * CSV export (`simulation_log.csv`, `simulation_log_traditional.csv`) for further analysis

---

## 📂 Repository Structure

```

├── Hexagonal\_Training\_Environment/
│   ├── HandoffEnvironment.m          \# MATLAB RL Environment Class
│   ├── run\_handoff\_simulation.m      \# Main Training Script
│   ├── run\_traditional\_handoff.m     \# Traditional A3 Algorithm Simulation
│   ├── run\_full\_comparison.m         \# Unified AI vs Traditional Comparison Script
│   ├── analyze\_hex\_results.m         \# Inference & Visualization Script
│   ├── analyze\_hex\_results\_with\_export.m  \# Inference with CSV Export
│   └── trainedHexAgent.mat           \# Pre-trained DQN Agent
│
└── Initial Model Inference/          \# Legacy/Draft models

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

To train the DQN agent from scratch:

1. Open `Hexagonal_Training_Environment/run_handoff_simulation.m`
2. Run the script.
3. Training Manager will launch and stop after convergence.
4. The agent will be saved as `trainedHexAgent.mat`.

#### 2. Run Single Inference (Recommended for Analysis)

To visualize the AI agent's performance on a random path:

1. Ensure `trainedHexAgent.mat` is available in your path.
2. Open `analyze_hex_results_with_export.m`.
3. Run the script.
4. View the generated plots and check `simulation_log.csv` for data.

#### 3. Run Traditional Handoff Simulation

To simulate standard A3 Event logic (Hysteresis based):

1. Open `run_traditional_handoff.m`.
2. Run the script.
3. This will generate a trajectory map, signal plots, and export `simulation_log_traditional.csv`.

#### 4. Run Unified Comparison (AI vs Traditional)

To see a direct head-to-head comparison:

1. Open `run_full_comparison.m`.
2. Run the script.
3. The script will:
   * Generate a random user trajectory.
   * Run the **AI Agent** on that trajectory.
   * Run the **Traditional Algorithm** on the *same* trajectory.
   * Produce a **Unified Comparison Dashboard** and a console performance table.

---

## 📤 Output

The simulation scripts generate various outputs:

* **Unified Comparison Dashboard** (from `run_full_comparison.m`)
  * **UE Trajectory:** Shows the path and AI connection decisions.
  * **Signal Quality Comparison:** Overlays AI and Traditional RSRP levels.
  * **Metric Bar Charts:** Visual comparison of Handovers and Ping-Pongs.

* **Console Reports**
  * Total handovers, ping-pong events, call drops, and average RSRP.

* **CSV Logs**
  * `simulation_log.csv`: AI agent flight data.
  * `simulation_log_traditional.csv`: Traditional algorithm flight data.

---

## 📊 Results Summary

| Metric         | Traditional A3 Algorithm | DQN Agent |
| -------------- | ------------------------ | --------- |
| Ping-Pong Rate | High (>15%)              | 0%        |
| Handover Count | High and unstable        | Optimized |
| Call Drop Rate | Low                      | Low       |

The agent learns to maintain the serving cell even if a neighbor is slightly stronger, switching only when required. This behavior resembles an adaptive hysteresis effect learned purely through experience.

---

## 👥 Authors

* **Aditya B Gupta (1DS22ET003)**
* **Asadulla MA Ansari (1DS22ET012)**
* **Shreya M R (1DS22ET085)**
* **Tanvi Patidar (1DS22ET097)**

Department of Electronics and Telecommunication Engineering
Dayananda Sagar College of Engineering, Bengaluru
