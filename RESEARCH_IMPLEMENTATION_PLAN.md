# Research & Implementation Plan
## Trajectory Tracking of Cable-Suspended Payloads with Autonomous Quadrotors

**Project Owner:** Haidar Saad  
**Advisor:** Prof. Andry  
**Start Date:** 2026-10-27  
**Current Status:** Phase 1 - Literature Review (Papers 1-2 Complete, Paper 3 In Progress)

---

## 1. RESEARCH GOAL & PROBLEM STATEMENT

### Primary Research Goal
Develop an **intelligent, adaptive control and trajectory planning system** for autonomous quadrotors carrying cable-suspended payloads that:
- Minimizes payload oscillation during dynamic flight
- Achieves precise trajectory tracking under real-world disturbances
- Adapts to unknown or varying payload mass
- Operates within hardware constraints (rotor speed, thrust limits)
- Combines classical control theory with reinforcement learning

### Problem Context
**Application Domain:** Autonomous aerial delivery, search-and-rescue, precision transport  
**Technical Challenge:** Underactuated system (4 rotor inputs) with 6+ degrees of freedom  
**Key Constraints:** Cable tension (can only pull), payload swinging dynamics, real-time computation

### Research Questions
1. **How can we combine flatness-based trajectory generation with RL-based control for optimal swing suppression?**
2. **What is the minimum set of state variables needed for effective swing angle estimation without additional sensors?**
3. **How does the choice of payload model (point mass vs. rigid body, elastic vs. inextensible cable) affect control performance?**
4. **Can we develop a unified framework that works across different payload masses through adaptive learning?**
5. **What is the computational and accuracy trade-off between simplified and full dynamic models in real-time control?**

---

## 2. RESEARCH METHODOLOGY: 15-PAPER STRUCTURED APPROACH

### Learning Strategy
Instead of reading papers randomly, we use a **scaffolded learning approach** where each paper builds knowledge toward a specific layer of the problem:

```
Papers 1-3:  Build theoretical foundation
     ↓
Papers 4-7:  Understand modern learning approaches
     ↓
Papers 8-10: Deep dive into control theory challenges
     ↓
Papers 11-13: Handle uncertainty & adaptation
     ↓
Papers 14-15: Implementation & validation
     ↓
Synthesis: Develop novel approach combining insights from all phases
```

---

## 3. LITERATURE PHASES & PAPER BREAKDOWN

### PHASE 1: Theoretical Foundations (Papers 1-3)
**Goal:** Understand the mathematical and physical foundations of the problem  
**Timeline:** Week 1-2  
**Status:** In Progress (2/3 complete)

#### Paper 1: Sreenath et al. (2013) - Differential Flatness ✅
**Type:** Foundational theory  
**Key Concepts:** 
- Differential flatness for underactuated systems
- Trajectory generation without solving full ODEs
- Flat output selection for quadrotor systems

**Learning Objectives:**
- [x] Understand what differential flatness means
- [x] Learn how it enables trajectory generation
- [x] See how it applies to quadrotor dynamics
- [ ] Understand limitations for suspended loads

**Contribution to Research:** Provides theoretical framework for trajectory design

---

#### Paper 2: Omar et al. (2022) - Control Landscape Survey ✅
**Type:** Comprehensive survey  
**Key Concepts:**
- Historical overview of control methods
- Hardware platforms and sensor technologies
- Testing methodologies and validation approaches

**Learning Objectives:**
- [x] Map the entire control methods landscape
- [x] Understand what's been tried and what works
- [x] Identify hardware-software integration challenges
- [x] Learn what gaps exist in current approaches

**Contribution to Research:** Provides context for where novel approach should fit

---

#### Paper 3: Omar et al. (2022) - System Dynamics & Modeling
**Type:** System identification & dynamic modeling  
**Key Concepts:**
- 15-DOF system representation
- Cable constraint modeling
- Multi-body dynamics formulation
- State variables and actuator constraints

**Learning Objectives:**
- [ ] Understand 15-DOF breakdown
- [ ] Learn cable constraint formulations
- [ ] Identify state variables for control
- [ ] Understand model simplifications trade-offs

**Contribution to Research:** Define the system we'll control

---

### PHASE 2: Learning-Based Control Methods (Papers 4-7)
**Goal:** Understand modern RL and data-driven approaches  
**Timeline:** Week 2-3  
**Status:** Not started

#### Papers 4-5: Deep Reinforcement Learning for Autonomous Flight
**Type:** Applied machine learning  
**Expected Concepts:**
- Policy gradient methods (PPO, TRPO)
- Value function approaches (DQN variants)
- Reward shaping for swing suppression
- State representation for suspended loads

**Learning Objectives:**
- [ ] Understand how RL can handle underactuated systems
- [ ] Learn reward design for cable-suspended systems
- [ ] Study curriculum learning for complex tasks
- [ ] Understand sample efficiency challenges

**Contribution to Research:** Provides adaptive control capability

---

#### Papers 6-7: Neural Networks & Trajectory Planning
**Type:** Applied deep learning  
**Expected Concepts:**
- Neural network-based controllers
- Recurrent networks for temporal dynamics
- Graph neural networks for multi-body systems
- Hybrid symbolic-neural approaches

**Learning Objectives:**
- [ ] Learn how neural networks capture dynamics
- [ ] Understand limitations and failure modes
- [ ] Study hybrid approaches combining symbolic + learning
- [ ] Evaluate interpretability vs. performance trade-off

**Contribution to Research:** Provides data-driven modular components

---

### PHASE 3: Advanced Control Theory (Papers 8-10)
**Goal:** Deep understanding of control challenges  
**Timeline:** Week 3-4  
**Status:** Not started

#### Paper 8: Nonlinear Control Theory
**Type:** Classical control theory  
**Expected Concepts:**
- Lyapunov stability
- Feedback linearization
- Backstepping control
- Underactuation and controllability

**Learning Objectives:**
- [ ] Understand nonlinear system stability
- [ ] Learn control design for underactuated systems
- [ ] Study cascade control architectures
- [ ] Understand theoretical guarantees

**Contribution to Research:** Provides stability-guaranteed frameworks

---

#### Papers 9-10: Robust & Adaptive Control
**Type:** Control under uncertainty  
**Expected Concepts:**
- Robust control (H-infinity, μ-synthesis)
- Adaptive control for unknown parameters
- Disturbance rejection techniques
- Constraint-aware control

**Learning Objectives:**
- [ ] Understand robustness margins
- [ ] Learn adaptation for unknown masses
- [ ] Study disturbance observer design
- [ ] Understand constraint handling

**Contribution to Research:** Ensures real-world performance

---

### PHASE 4: System Identification & Adaptation (Papers 11-13)
**Goal:** Handle real-world parameter variation  
**Timeline:** Week 4  
**Status:** Not started

#### Papers 11-13: Parameter Estimation & Online Learning
**Type:** System identification and adaptation  
**Expected Concepts:**
- Least-squares parameter estimation
- Extended Kalman filters
- Online learning algorithms
- Multi-model adaptive control

**Learning Objectives:**
- [ ] Learn to identify unknown payload mass
- [ ] Understand cable property estimation
- [ ] Study online adaptation algorithms
- [ ] Learn sensor fusion techniques

**Contribution to Research:** Enables real-world deployment with unknown loads

---

### PHASE 5: Implementation & Real-World (Papers 14-15)
**Goal:** Practical implementation and validation  
**Timeline:** Week 5  
**Status:** Not started

#### Papers 14-15: Simulation, Hardware, and Sim-to-Real
**Type:** Implementation and experimental validation  
**Expected Concepts:**
- Physics simulation (Gazebo, MuJoCo, PyBullet)
- Sim-to-real transfer techniques
- Hardware integration and real-time constraints
- Experimental validation methodologies

**Learning Objectives:**
- [ ] Learn MuJoCo simulation setup for suspended loads
- [ ] Understand domain randomization techniques
- [ ] Study real-time control implementation
- [ ] Learn experimental validation approaches

**Contribution to Research:** Validates proposed approach in simulation and practice

---

## 4. IMPLEMENTATION ARCHITECTURE

### Overall System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   RESEARCH SYSTEM PIPELINE                  │
└─────────────────────────────────────────────────────────────┘

LAYER 1: SIMULATION ENVIRONMENT (MuJoCo)
├─ Quadrotor dynamics (6 DOF)
├─ Cable-suspended payload (2 swing DOF + mass)
├─ Real-world disturbances (wind, measurement noise)
└─ Sensor models (IMU, motor feedback, swing angle sensors)

LAYER 2: CONTROL ARCHITECTURE
├─ Trajectory Planner
│  ├─ Flatness-based trajectory generator
│  ├─ Swing-free trajectory computation
│  └─ Collision avoidance (optional)
│
├─ State Estimator
│  ├─ Extended Kalman Filter (EKF)
│  ├─ Swing angle estimation (from IMU + dynamics)
│  └─ Payload mass estimation
│
└─ Controller (Multi-level)
   ├─ Low-Level: Motor command generation (4 thrusts)
   ├─ Mid-Level: Attitude controller (classic + adaptive)
   └─ High-Level: Swing suppression (RL-based + feedback)

LAYER 3: LEARNING COMPONENT
├─ RL Agent (Policy Network)
│  ├─ Observation: state variables (position, attitude, swing angles)
│  ├─ Action: thrust distribution or attitude commands
│  └─ Reward: trajectory tracking error + swing suppression
│
├─ Value Function Network (for comparison/ablation)
└─ Adaptation Module (online learning for mass estimation)

LAYER 4: ANALYSIS & VALIDATION
├─ Metrics computation (oscillation, tracking error, energy)
├─ Comparison with baselines (classic control, other RL methods)
└─ Sensitivity analysis (model variations, disturbances)
```

### Key Components to Implement

#### 4.1 MuJoCo Environment Setup
```python
# Simulate:
# - Quadrotor with 4 rotors (independent thrust control)
# - Rigid body dynamics with realistic inertia/mass
# - Cable as constraint (inextensible, fixed length)
# - Payload as point mass or small rigid body
# - Realistic actuator saturation (rotor speed limits)
# - Sensor noise (IMU, gyroscope drift)
# - Environmental disturbances (wind gusts)
```

**Why MuJoCo?**
- Fast, accurate physics simulation
- Easy constraint modeling (cable length)
- Good for RL training (fast step times)
- Can export to C++ for real-time control

---

#### 4.2 Trajectory Planning Module
```python
# Two approaches to combine:

# Approach A: Flatness-Based (from Paper 1)
# - Select flat outputs: (x, y, z, ψ)
# - Generate smooth trajectory avoiding singularities
# - Compute required swing angles automatically
# - No feedback needed in ideal case

# Approach B: Optimal Planning (from Papers 8-10)
# - Minimize energy while respecting constraints
# - Generate time-optimal trajectories
# - Account for rotor speed limits
```

---

#### 4.3 State Estimation
```python
# Estimate full state from limited measurements:
# Measured: Quadrotor position (GPS/VICON), attitude (IMU), motor RPM
# Estimated: Payload position, swing angles, payload mass

# Tools: Extended Kalman Filter + Online Adaptation
# Challenge: Observability of swing angles without direct measurement
```

---

#### 4.4 Control System (Hybrid Classical + Learning)
```
Control Hierarchy:

1. TRAJECTORY REFERENCE GENERATOR
   Input: Target destination, task type
   Output: Reference trajectory (x_ref, y_ref, z_ref, ψ_ref)

2. SWING-FREE TRAJECTORY PLANNER
   Input: Reference trajectory
   Output: Required swing angles θ_p_ref, φ_p_ref
   Method: Flatness-based (Paper 1) + optimization (Papers 8-10)

3. STATE ESTIMATOR
   Inputs: Sensors (IMU, position, motor feedback)
   Outputs: Current state estimate (position, attitude, swing angles, mass)
   Method: EKF + adaptive filters (Papers 11-13)

4. HIGH-LEVEL CONTROLLER (RL-BASED)
   Inputs: Reference trajectory + current state
   Outputs: Desired thrust distribution or attitude command
   Method: RL policy network trained with reward for:
           - Swing suppression (minimize θ_p, φ_p)
           - Trajectory tracking (minimize position error)
           - Energy efficiency

5. LOW-LEVEL ATTITUDE CONTROLLER (CLASSICAL)
   Inputs: Desired attitude (from RL or feedforward)
   Outputs: 4 motor thrusts
   Method: Cascaded PID or adaptive control

6. MOTOR COMMAND GENERATOR
   Inputs: 4 thrust commands
   Outputs: 4 rotor RPM setpoints
   Constraint: 0 ≤ RPM ≤ RPM_max
```

---

#### 4.5 Reinforcement Learning Agent
```python
# Policy Network Architecture
Input: [position (3D), velocity (3D), 
        attitude (3 angles), angular velocity (3D),
        swing angles (2D), swing rates (2D),
        payload mass estimate (1D)] = 17 dims

Output: [attitude commands or thrust distribution] = 4 dims

Training:
- Algorithm: PPO or SAC (from Papers 4-5)
- Reward: r = -w1*tracking_error - w2*swing_angle - w3*energy
- Environment: MuJoCo simulation with randomized payloads
- Training duration: Until convergence (typically 100k-1M steps)
```

---

### System Integration Points

| Component | Input From | Output To | Papers |
|-----------|-----------|-----------|---------|
| Trajectory Planner | Task, obstacles | Reference trajectory | 1, 8-10 |
| State Estimator | Sensors, dynamics | State estimate | 11-13 |
| RL Controller | State estimate, reference | Attitude commands | 4-7 |
| Attitude Controller | Attitude commands | Motor thrusts | 8-10 |
| Motor Generator | Motor commands | Rotor RPM | 14-15 |
| MuJoCo Simulator | Rotor RPM | Sensor readings | 14-15 |

---

## 5. RESEARCH CONTRIBUTIONS & INNOVATION STRATEGY

### Novel Contributions We'll Develop

#### Contribution 1: Hybrid Flatness + RL Framework
**What:** Combine theoretical guarantees of differential flatness with adaptability of RL  
**Why:** Flatness gives structure; RL handles real-world constraints and disturbances  
**How:** 
- Use flatness to generate nominal swing-free trajectories
- Train RL agent to follow trajectory while suppressing residual oscillations
- RL learns to adapt for unknown masses

**Papers involved:** 1 (flatness) + 4-7 (RL)

---

#### Contribution 2: Model-Free Swing Angle Estimation
**What:** Estimate swing angles without direct sensors using system dynamics  
**Why:** Reduces hardware cost and complexity  
**How:**
- Use Extended Kalman Filter with dynamics model
- Online learning of cable properties
- Validate observability mathematically

**Papers involved:** 3 (modeling) + 11-13 (estimation)

---

#### Contribution 3: Payload-Adaptive Controller
**What:** Single controller works for unknown/varying payload mass  
**Why:** Real systems have different payload masses  
**How:**
- RL policy trained with domain randomization (varied masses)
- Online mass estimation module
- Adaptive gain scheduling based on mass

**Papers involved:** 4-7 (RL) + 11-13 (adaptation)

---

#### Contribution 4: Constraint-Aware Trajectory Planning
**What:** Generate trajectories respecting cable tension and rotor limits  
**Why:** Current methods ignore practical constraints  
**How:**
- Extend flatness framework to include constraints
- Optimization-based planning with constraint encoding
- Real-time feasibility checking

**Papers involved:** 1 (flatness) + 8-10 (optimization)

---

### Comparison Baselines
We'll compare against:
1. **PID Control** (simple baseline)
2. **LQR Control** (classical optimal control)
3. **Sliding Mode Control** (robust nonlinear control)
4. **Standard RL** (without flatness integration)
5. **Neural Network Controller** (direct policy)

---

## 6. VALIDATION & TESTING APPROACH

### Validation Strategy (3-Tier)

#### Tier 1: Simulation Validation (MuJoCo)
**Metrics:**
- Payload swing angle (target: < 5° during maneuvers)
- Trajectory tracking error (target: < 0.1m)
- Settling time (target: < 5 seconds)
- Energy consumption

**Test Cases:**
```
1. Point-to-point navigation (varying distances)
2. Circular trajectory (constant speed)
3. Figure-8 trajectory (direction changes)
4. Step response (sudden velocity commands)
5. Disturbance rejection (wind gusts)
6. Unknown payload mass (5 kg to 15 kg)
7. Sensor noise robustness
8. Cable breakage detection
```

**Success Criteria:**
- All test cases pass with swing < 5°
- Tracking error < 0.1m for 95% of trajectory
- Robust to ±20% mass variation

---

#### Tier 2: Comparison & Ablation (Simulation)
**Ablation Studies:**
1. Remove RL component → test classical control alone
2. Remove swing estimation → test with direct sensor
3. Remove adaptive mass learning → test fixed mass
4. Simplify dynamics model → test with point mass vs. rigid body
5. Remove flatness trajectory → test with standard planning

**Expected Result:** RL + flatness + adaptation should outperform all ablations

---

#### Tier 3: Real-World Transfer (if hardware available)
**Prerequisites:**
1. Quadrotor platform (e.g., Crazyflie, DJI M300)
2. Payload sling setup
3. Motion capture system or VICON
4. Real-time onboard computer

**Transfer Steps:**
1. Domain randomization in simulation
2. System identification on real hardware
3. Policy finetuning with real data
4. Field testing in controlled environment
5. Performance comparison vs. simulation

---

### Metrics & Analysis

#### Primary Metrics
| Metric | Definition | Target |
|--------|-----------|--------|
| Max Swing Angle | max(\|θ_p\|, \|φ_p\|) during maneuver | < 5° |
| Mean Tracking Error | RMS(position error) | < 0.1m |
| Settling Time | Time to reach < 1° swing after maneuver | < 5s |
| Energy Per Distance | Total motor energy / distance traveled | Minimize |

#### Secondary Metrics
| Metric | Definition | Purpose |
|--------|-----------|---------|
| Robustness | Performance across parameter variations | Validate adaptability |
| Generalization | Performance on unseen trajectories | Validate learning |
| Computational Load | CPU usage during control | Real-time feasibility |
| Sensitivity | Performance vs. sensor noise levels | Practical applicability |

---

## 7. TIMELINE & DELIVERABLES

### Overall Timeline: 6-8 Weeks

```
WEEK 1-2: Literature Foundation
├─ Paper 1 (Flatness)
├─ Paper 2 (Survey)
├─ Paper 3 (Dynamics)
└─ Deliverable: RESEARCH_PLAN.md + KEY_INSIGHTS.md

WEEK 2-3: Modern Methods
├─ Papers 4-7 (RL + Learning)
└─ Deliverable: RL_FRAMEWORK.md + Algorithm Selection Report

WEEK 3-4: Control Theory Deep Dive
├─ Papers 8-10 (Control)
└─ Deliverable: CONTROL_DESIGN.md + Stability Analysis

WEEK 4: Adaptation & System ID
├─ Papers 11-13 (Adaptation)
└─ Deliverable: ESTIMATION_MODULE.md

WEEK 5: Implementation Planning
├─ Papers 14-15 (Real-world)
└─ Deliverable: IMPLEMENTATION_ARCHITECTURE.md

WEEK 5-6: Development & Simulation
├─ MuJoCo environment setup
├─ Control architecture implementation
├─ RL agent training
└─ Deliverable: Working simulation + results

WEEK 6-7: Validation & Optimization
├─ Ablation studies
├─ Baseline comparisons
├─ Performance tuning
└─ Deliverable: Experiment Report + Analysis

WEEK 7-8: Documentation & Finalization
├─ Research paper writeup
├─ Code documentation
├─ Presentation preparation
└─ Deliverable: Research Paper + Code Release
```

---

### Deliverables by Phase

#### Phase 1: Literature Review (Due: End Week 2)
- [x] Paper 1: Differential Flatness Analysis
- [x] Paper 2: Control Landscape Survey
- [ ] Paper 3: System Dynamics Analysis
- [ ] Literature Summary Document
- [ ] Research Questions Refined

#### Phase 2: Method Selection (Due: End Week 3)
- [ ] RL Algorithm Selection Report
- [ ] Comparative Framework Analysis
- [ ] Reward Function Design Document
- [ ] State Representation Analysis

#### Phase 3: Architecture Design (Due: End Week 4)
- [ ] Control System Architecture Diagram
- [ ] MuJoCo Environment Specification
- [ ] Algorithm Pseudocode & Flowcharts
- [ ] Implementation Roadmap

#### Phase 4: Development (Due: End Week 6)
- [ ] MuJoCo simulation code
- [ ] Control architecture implementation
- [ ] RL training pipeline
- [ ] Baseline controller implementations

#### Phase 5: Validation (Due: End Week 7)
- [ ] Simulation results (all test cases)
- [ ] Ablation study results
- [ ] Baseline comparison
- [ ] Performance analysis report

#### Phase 6: Finalization (Due: End Week 8)
- [ ] Research paper (conference format)
- [ ] Code repository with documentation
- [ ] Supplementary videos/animations
- [ ] Presentation slides

---

## 8. CURRENT STATUS & NEXT STEPS

### Status: Week 1/8
- ✅ Project selected and motivated
- ✅ Literature curated (15 papers identified)
- ✅ Papers 1-2 read and analyzed
- ⏳ Paper 3 analysis in progress
- ⏳ Full plan created (this document)

### Immediate Next Steps (This Week)
1. **Complete Paper 3 Reading**
   - Read full paper and document key findings
   - Extract specific dynamic equations
   - Understand 15-DOF breakdown for our system

2. **Finalize Research Questions**
   - Use insights from Papers 1-3
   - Refine objectives based on literature
   - Define success criteria

3. **Start Papers 4-5** (Learning Methods)
   - Understand RL algorithms
   - Learn how they apply to suspended loads
   - Identify best approaches for our problem

### Decision Points Coming Up
- Week 2: RL algorithm selection (PPO vs. SAC vs. other?)
- Week 3: Controller architecture (flat hierarchy vs. cascaded?)
- Week 4: Estimation approach (EKF vs. particle filter vs. NN?)

---

## 9. SUCCESS CRITERIA

### Research Success Defined By:
- ✓ Novel approach combining flatness + RL (not done before)
- ✓ Theoretical justification from control theory
- ✓ Empirical validation in simulation
- ✓ Beats multiple baselines (PID, LQR, pure RL)
- ✓ Generalizes to unknown payload masses
- ✓ Publishable results

### Implementation Success Defined By:
- ✓ MuJoCo simulation working
- ✓ All test cases passing
- ✓ Max swing < 5° during maneuvers
- ✓ Tracking error < 0.1m
- ✓ Real-time feasibility proven
- ✓ Code is clean and reproducible

---

## APPENDIX: VISUALIZATION OF LEARNING PATHWAY

```
START: Problem Understanding
  │
  ├─→ PAPER 1 (Flatness Theory)
  │     └─ Learn: How to design trajectories mathematically
  │
  ├─→ PAPER 2 (Control Survey)
  │     └─ Learn: What methods exist and their trade-offs
  │
  ├─→ PAPER 3 (System Dynamics)
  │     └─ Learn: Exactly what we need to control (15 DOF)
  │
  ├─→ PAPERS 4-7 (Learning Methods)
  │     └─ Learn: How to adapt and optimize in real-time
  │
  ├─→ PAPERS 8-10 (Control Theory)
  │     └─ Learn: How to guarantee stability and robustness
  │
  ├─→ PAPERS 11-13 (Adaptation)
  │     └─ Learn: How to handle unknown parameters
  │
  ├─→ PAPERS 14-15 (Implementation)
  │     └─ Learn: How to build and validate real systems
  │
  └─→ SYNTHESIS: Design Novel Approach
       └─ Combine: Flatness + RL + Adaptation + Robustness
           Result: Intelligent payload tracking system
```

---

## Questions to Revisit After Each Phase

**After Phase 1 (Week 2):** Do the 15 papers I selected actually address my research questions?

**After Phase 2 (Week 3):** Is RL the right approach? Should I consider hybrid methods?

**After Phase 3 (Week 4):** Is my proposed architecture feasible to implement? What are the computational bottlenecks?

**After Phase 4 (Week 6):** Is the simulation behaving realistically? Do the results make sense physically?

**After Phase 5 (Week 7):** Does my approach actually outperform baselines? Where are the gaps?

**After Phase 6 (Week 8):** Is this contribution novel enough? What's the main insight I'm adding to the field?

---

**END OF RESEARCH & IMPLEMENTATION PLAN**

*Last Updated: 2026-10-29*  
*Next Review: After Paper 3 Completion*
