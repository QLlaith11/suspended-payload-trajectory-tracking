# 15 Research Papers for Cable-Suspended Payload Control
## Comparative Study in MuJoCo Simulation

**Updated:** October 26, 2026  
**Status:** Simulation-Only Research  
**Researcher:** Haidar Saad  

---

## YOUR PROFILE

- **Researcher:** Final-year Robotics & Intelligent Systems student
- **Your Strengths:** MuJoCo expert | RL specialist (PPO/TD3) | Control theory | System identification
- **Project:** Comparative analysis of control methods for cable-suspended payloads in MuJoCo
- **Novel Contribution:** Benchmarking 4 control approaches with fair comparison methodology

---

## PHASE 1: FOUNDATION (Weeks 1-2)

### Paper 1: Geometric Control and Differential Flatness of a Quadrotor UAV with a Cable-Suspended Load
- **Authors:** Sreenath, Lee, Kumar (2013)
- **Venue:** IEEE CDC
- **Status:** FOUNDATIONAL
- **Description:** THE seminal paper establishing 8-DOF with 4 underactuated degrees and differential flatness proof. Essential baseline for your comparison study.
- **Access:** https://arxiv.org/pdf/1309.6717

### Paper 2: Recent Advances and Challenges in Controlling Quadrotors with Suspended Loads
- **Authors:** Survey (2022)
- **Venue:** ScienceDirect
- **Status:** OVERVIEW
- **Description:** Comprehensive survey of all control methods. Understand the landscape before implementing your 4 methods.
- **Access:** https://www.sciencedirect.com/science/article/pii/S1110016822005245

### Paper 3: Modeling and Control of a Multirotor UAV with a Cable-Suspended Rigid Body Payload: A Double-Pendulum Approach
- **Authors:** Recent (2025)
- **Venue:** Robotics and Autonomous Systems, Vol. 193
- **Status:** ESSENTIAL
- **Description:** 15-DOF nonlinear model with cable elasticity, slack/taut phases, and LQR control. Your MuJoCo model reference.
- **Access:** https://www.sciencedirect.com/science/article/pii/S0921889025001460

---

## PHASE 2: LEARNING-BASED METHODS (Weeks 2-3)

### Paper 4: FLARE: Agile Flights for Quadrotor Cable-Suspended Payload System via Reinforcement Learning
- **Authors:** Recent (2025)
- **Venue:** arXiv:2508.09797
- **Status:** RL | SIMULATION-VALIDATED
- **Description:** RL policy for cable-suspended payloads. Method 2 for your comparison: learn swing suppression with PPO/TD3.
- **Access:** https://arxiv.org/pdf/2508.09797

### Paper 5: ASTER: Attitude-aware Suspended-payload Quadrotor Traversal via Efficient Reinforcement Learning
- **Authors:** Recent (2026)
- **Venue:** arXiv:2603.10715
- **Status:** RL | SIMULATION-VALIDATED
- **Description:** Attitude-aware RL for constrained trajectory tracking. Alternative RL approach emphasizing attitude control efficiency.
- **Access:** https://arxiv.org/pdf/2603.10715

### Paper 6: Adaptive Outer-Loop Control of Quadrotors via Reinforcement Learning
- **Authors:** Recent (2025)
- **Venue:** arXiv:2605.16015
- **Status:** RL | SIMULATION-VALIDATED
- **Description:** Learns model for external forces/moments estimation. Handles payload disturbances in simulation validation.
- **Access:** https://arxiv.org/pdf/2605.16015

### Paper 7: Residual Policy Learning
- **Authors:** Silver et al. (2018)
- **Venue:** arXiv:1812.06298
- **Status:** METHODOLOGY
- **Description:** YOUR APPROACH! Improves geometric control with model-free RL. Method 3 for comparison: geometric baseline + learned residual.
- **Access:** https://arxiv.org/pdf/1812.06298v2

---

## PHASE 3: CONTROL THEORY & CONSTRAINTS (Weeks 3-4)

### Paper 8: A Review of Quadrotor: An Underactuated Mechanical System
- **Authors:** Review (2018)
- **Venue:** ScienceDirect
- **Status:** THEORY
- **Description:** Foundation on underactuation theory (8 DOF, 4 inputs). Explains nested control loops and adaptive methods for underactuated systems.
- **Access:** https://www.sciencedirect.com/science/article/abs/pii/S1367578818300932

### Paper 9: Anti-Swing Control for Quadrotor-Slung Load Transportation System with Underactuated State Constraints
- **Authors:** Recent (2023)
- **Venue:** Sensors Journal
- **Status:** CONSTRAINT HANDLING | SIMULATION-VALIDATED
- **Description:** Barrier Lyapunov functions for swing bounds. Method 1 for comparison: constraint-aware control keeping swing < ±20°.
- **Access:** https://doi.org/10.3390/s23218995

### Paper 10: Nonlinear Control of Unmanned Aerial Vehicles with Cable Suspended Payloads
- **Authors:** Multiple (2018)
- **Venue:** ScienceDirect
- **Status:** THEORY
- **Description:** Comprehensive nonlinear control: feedback linearization, backstepping, coupling dynamics. Theoretical foundation for multiple approaches.
- **Access:** https://www.sciencedirect.com/science/article/abs/pii/S1270963818325185

---

## PHASE 4: SYSTEM IDENTIFICATION & ADAPTATION (Week 4)

### Paper 11: RISE-Based Adaptive Control with Mass-Inertia Parameter Estimation for Aerial Transportation
- **Authors:** Recent (2022)
- **Venue:** arXiv:2209.08209
- **Status:** PARAMETER ESTIMATION | SIMULATION-VALIDATED
- **Description:** Method 4 for comparison: adaptive control with online mass-inertia estimation. YOUR strength area.
- **Access:** https://arxiv.org/pdf/2209.08209

### Paper 12: ESO-based Adaptive Neural Network Control for a Quadrotor Against Wind and Payload Disturbances
- **Authors:** Recent (2026)
- **Venue:** Nature Scientific Reports
- **Status:** DISTURBANCE REJECTION | SIMULATION-VALIDATED
- **Description:** Extended state observer + adaptive NN. Disturbance estimation and feedforward control for unknown payloads.
- **Access:** https://www.nature.com/articles/s41598-026-38931-8

### Paper 13: Control of an Aerial Manipulator Using On-line Parameter Estimator for an Unknown Payload
- **Authors:** Recent (2016)
- **Venue:** arXiv:1601.02204
- **Status:** PARAMETER ESTIMATION
- **Description:** Online payload mass estimation with passivity-based control. Handles unknown payload parameters during operation.
- **Access:** https://arxiv.org/pdf/1601.02204

---

## PHASE 5: MUJOCO IMPLEMENTATION & BENCHMARKING (Week 5)

### Paper 14: MuJoCo-Drones-Gym: A GPU-Accelerated Multi-Drone Simulator for Control and Reinforcement Learning
- **Authors:** Recent (2025)
- **Venue:** arXiv:2606.08039
- **Status:** ⭐ NEW | ESSENTIAL FOR YOU
- **Description:** PERFECT FRAMEWORK! Open-source MuJoCo environment for drone control. Gymnasium-compatible with Stable-Baselines3. Build your cable-suspended model on this.
- **Key Features:** GPU acceleration, modular physics models, PID baselines for comparison, wind/obstacle handling, curriculum learning support.
- **Access:** https://arxiv.org/pdf/2606.08039

### Paper 15: Leveling the Playing Field: Carefully Comparing Classical and Learned Controllers for Quadrotor Trajectory Tracking
- **Authors:** Recent (2025)
- **Venue:** arXiv:2506.17832
- **Status:** ⭐ NEW | BENCHMARKING METHODOLOGY
- **Description:** YOUR RESEARCH FRAMEWORK! Addresses fair comparison of control methods. Identifies common pitfalls in benchmarking. Use this to structure your 4-method comparison.
- **Key Insights:** Baseline tuning matters, dataset matching is critical, trajectory RMSE metrics, RL vs. classical controller comparison best practices.
- **Access:** https://arxiv.org/html/2506.17832

---

## YOUR RESEARCH PROJECT

### Title
**"Comparative Analysis of Control Methods for Cable-Suspended Payloads: A Comprehensive MuJoCo Simulation Study"**

### 4 Methods to Compare

1. **Barrier Lyapunov Control** (Paper 9)
   - Constraint-aware baseline
   - Explicit swing bounds, proven stability
   - Conservative approach

2. **Residual RL Policy** (Paper 7)
   - Learned swing suppression on geometric control
   - Hybrid approach, leverages your methodology
   - Interpretable control signals

3. **Pure RL Approach** (Papers 4-6)
   - End-to-end learning with PPO/TD3
   - Adaptive, learns disturbance rejection
   - High performance but black-box

4. **Adaptive System ID** (Paper 11)
   - Online parameter estimation + control
   - Handles unknown payloads
   - Your strength area

### Implementation Stack

- **Simulator:** MuJoCo-Drones-Gym (Paper 14)
- **RL Framework:** Stable-Baselines3
- **Benchmarking:** Fair comparison methodology (Paper 15)
- **Model:** 15-DOF cable-suspended payload (Paper 3)

### Validation Metrics

✅ Trajectory tracking RMSE (position error)  
✅ Swing angle suppression (max swing during maneuvers)  
✅ Computational efficiency (control loop time)  
✅ Robustness to payload mass variation (±50%)  
✅ Parameter sensitivity analysis  
✅ Convergence speed (settling time)  
✅ Fair benchmarking following Paper 15 methodology  
✅ Open-source MuJoCo implementation with reproducible results  
✅ Publishable comparative study paper

---

## 5-WEEK SIMULATION-BASED ROADMAP

### Week 1: Foundation Theory (12 hours)
- **Papers:** 1, 2, 3
- **Goal:** Learn differential flatness, 15-DOF modeling, understand the control landscape
- **Deliverable:** Notes on system dynamics and modeling requirements for MuJoCo

### Week 2: Learning-Based Methods (12 hours)
- **Papers:** 4, 5, 6, 7
- **Goal:** Master RL approaches (your expertise) and reference+residual methodology
- **Deliverable:** Design RL policies for Methods 2 & 3

### Week 3-4: Control Theory & Constraints (14 hours)
- **Papers:** 8, 9, 10
- **Goal:** Understand nonlinear control, underactuation, constraint-aware methods
- **Deliverable:** Implement Methods 1 & 4, understand their theoretical foundations

### Week 4: System ID & Robustness (8 hours)
- **Papers:** 11, 12, 13
- **Goal:** Master adaptive control and parameter estimation (your strength area)
- **Deliverable:** Add online parameter estimation to Method 4

### Week 5: MuJoCo Setup & Benchmarking (12 hours)
- **Papers:** 14, 15
- **Goal:** Set up MuJoCo-Drones-Gym, implement fair comparison framework
- **Deliverable:** Complete simulation setup, run initial benchmarks

### Weeks 6-8: Validation & Analysis (ongoing)
- Sensitivity analysis, parameter sweeps, robustness testing
- Compare all 4 methods across metrics
- Write comparative analysis paper

---

## SUCCESS CRITERIA

✅ Swing angles < ±20° during aggressive maneuvers for all methods  
✅ Position tracking RMSE < 0.5m for all methods  
✅ Computational cost comparison (control loop time in Hz)  
✅ Robustness to payload mass variation (±50%)  
✅ Sensitivity analysis on cable length and control parameters  
✅ Convergence speed comparison (settling time to steady state)  
✅ Fair benchmarking following Paper 15 methodology  
✅ Open-source MuJoCo implementation with reproducible results  
✅ Publishable comparative study paper  

---

**Last Updated:** October 26, 2026  
**Timeline:** 8 weeks from start to publishable paper  
**Focus:** Simulation-only (no hardware validation needed)  
**Novel Contribution:** Fair benchmarking of 4 control methods in MuJoCo