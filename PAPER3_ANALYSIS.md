# Paper 3 Analysis: Quadrotor with Suspended Loads (Omar et al., 2022)

**Citation:** Hanafy M. Omar, Rizwan Akram, Saad M.S. Mukras, Ahmed Alaa Mahvouz  
**Title:** Recent advances and challenges in controlling quadrotors with suspended loads  
**Journal:** Alexandria Engineering Journal (2023) 63: 253-270  
**DOI:** 10.1016/j.aej.2022.08.001

---

## Paper Summary

This is a **comprehensive survey** reviewing control methods, hardware implementations, and testing strategies for quadrotors carrying suspended payloads. Rather than presenting a single 15-DOF model, it synthesizes findings across multiple studies.

---

## Key Findings for Your Research

### 1. SYSTEM MODELING APPROACHES

#### Multi-body Dynamics Frameworks
- **Lagrangian Approach**: Most common for suspended load systems
- **Euler-Lagrange Equations**: Used to derive equations of motion considering interaction forces between quadrotor and load
- **Newton-Euler Formulation**: Alternative approach (mentioned for some implementations)

#### Typical DOF Representations Found in Literature
- **Quadrotor**: 6 DOF (x, y, z position + roll, pitch, yaw)
- **Suspended Load**: Additional DOF for swing angles
- **Total System**: 8-15 DOF depending on cable model assumptions

### 2. CABLE CONSTRAINT MODELING

**Key Simplifications in Literature:**
- **Inextensible cable assumption**: Cable length is constant
- **Massless cable**: Simplifies dynamics significantly
- **Rigid attachment**: Load is rigidly fixed at cable attachment point
- **Point mass load**: Load modeled as concentrated mass

**Elastic Cable Models** (mentioned but less common):
- Some papers include cable stiffness and damping
- Requires additional state variables for cable deflection

### 3. CONTROL ALGORITHMS REVIEWED

#### Main Approaches:
1. **Flight Trajectory Approach**
   - Generate swing-free trajectories without feedback
   - Minimize payload oscillation during transport
   - References: [30], [31] in paper

2. **Anti-swing Controller Approach**
   - Time-delayed feedback to suppress oscillations
   - Measure swing angles directly
   - Controller designed based on measured states

3. **PID-based Methods**
   - Simple gain tuning (LQR often better)
   - Work well for smaller systems

4. **LQR (Linear Quadratic Regulator)**
   - Better performance than PID
   - Requires linearization around equilibrium

5. **Advanced Methods Mentioned:**
   - Sliding Mode Control
   - Adaptive Control
   - Neural Networks / Fuzzy Logic
   - Reinforcement Learning (RL)

---

## Dynamic Model Components

### State Variables (Typical System)
```
Quadrotor states:
- Position: x_q, y_q, z_q
- Orientation: φ (roll), θ (pitch), ψ (yaw)
- Linear velocities: ẋ_q, ẏ_q, ż_q
- Angular velocities: φ̇, θ̇, ψ̇

Payload states:
- Position: x_p, y_p, z_p (or derived from cable angles)
- Swing angles: θ_p (pitch swing), φ_p (roll swing)
- Swing angle rates: θ̇_p, φ̇_p
```

### Actuator Constraints
- **4 Motor Thrust Commands** (rotors with variable speeds)
- **Thrust Constraints**: 0 ≤ T_i ≤ T_max for each rotor
- **Attitude Control**: Only 2 DOF for tilt (roll, pitch) are directly controllable
- **Underdactuation**: System has 6 DOF but only 4 control inputs

---

## Cable Swing Angle Measurement Methods

**Contact Methods:**
1. Potentiometer (simple but mechanical)
2. Encoder (better precision)
3. Joystick-based (Hall effect sensors)

**Non-contact Methods:**
1. Laser Displacement Sensor (LDS) - Most recommended
2. Vision-based (camera with marker tracking)
3. IMU + inclinometer at payload

**Key Insight:** Paper recommends **AHRS + IMU combination** for:
- Better accuracy
- Wireless capability
- Integration with onboard electronics

---

## Hardware Platforms Reviewed

The paper surveys 5 main platform categories:

1. **FPGA-Based** (e.g., Phenix Pro, OcPoC)
   - High performance, complex programming
   - Good for real-time control

2. **ARM-Based** (e.g., Pixhawk, Pixracer)
   - Most popular in research
   - Good balance of performance/simplicity

3. **ATMEL-Based** (e.g., APM 2.8)
   - Older but reliable
   - Fewer features

4. **Raspberry Pi Based**
   - Lower cost
   - Limited real-time capability

5. **Custom implementations** (Humminbird quadrotor, etc.)

**Recommended:** Pixhawk for suspended load applications

---

## Testing & Validation Approaches

### Simulation Platforms
- **Gazebo + Simulink** (SITL)
- **Hardware-in-the-loop** (HITL) with real flight code
- **Custom simulations** (MATLAB/Simulink)

### Testbed Platforms (DOF)
- **1 DOF**: Basic pitch/yaw control testing
- **3-4 DOF**: Realistic suspended load testing
- **6 DOF**: Full attitude and position control

**Gap Identified:** Few testbeds with suspended loads available

---

## Comparison to Sreenath Flatness Framework

### Sreenath et al. (2013) - Differential Flatness
**Your Paper 1** established:
- Under-actuated systems can be made flat with proper output selection
- Enables trajectory generation without solving full ODEs

### How This Paper Builds On It:
1. **Practical implementation challenges** of flatness-based approach
2. **Real-world constraints** (rotor speed limits, thrust saturation)
3. **Feedback requirements** for disturbance rejection
4. **Cable tension constraints** add complexity not fully addressed in flatness framework

---

## Key References for 15-DOF Model

The paper doesn't present a single 15-DOF model, but references several:

- **[50] Sadr et al. (2014)**: Dynamics modeling of quadrotor with slung load
- **[40] Sekiguchi et al.**: Feedback control for drone with suspended load
- **[36] Fiacchini et al.**: Multi-rotor with suspended load - System Dynamics
- **[51] Kosugi et al.**: Dynamics and Control of quadrotor with payload

---

## Degrees of Freedom Breakdown

For a **general suspended load system**:

```
15 DOF = 6 (Quadrotor) + 6 (Load) + 3 (Cable constraints effect)
```

More specifically:
- **Quadrotor**: 6 DOF (x, y, z, roll, pitch, yaw)
- **Payload**: 3 translational DOF (but coupled to quadrotor)
- **Payload Swing**: 2 DOF (pitch swing angle, roll swing angle)
- **Cable tension**: 1-4 DOF (constraint forces)

**Exact count depends on:**
- Whether cable is elastic (adds more DOF)
- Whether load has orientation (adds 3 more DOF for 15 total)
- Cable constraints (reduces effective DOF)

---

## Implementation Recommendations for MuJoCo

### Model Components to Include:
1. **Quadrotor body** (rigid body dynamics)
2. **Cable sling** (as single constraint or multiple elements)
3. **Payload mass** (point mass or rigid body)
4. **Motor dynamics** (rotor inertia, speed response)
5. **Sensor models** (for swing angle feedback)

### Actuator Representation:
- 4 motor thrust inputs (constrained 0 to T_max)
- Attitude control through differential thrusts

### Constraint Implementation:
- Cable length constraint (equality constraint)
- Cable tension non-negativity (can't push, only pull)

---

## Novel Contribution Strategy

Based on this survey, gaps for novel work:

1. **RL + Flatness**: Combine reinforcement learning with flatness-based trajectory generation
2. **Elastic Cable Models**: Better representation of real cables
3. **Adaptive Swing Angle Estimation**: Without additional sensors
4. **Multi-cable Systems**: Multiple attachment points
5. **Fast Trajectory Planning**: Under real-time constraints
6. **Sim-to-Real Transfer**: RL policies trained in simulation

---

## Questions for Paper 4-7 (Learning Methods)

- How do existing papers handle the 15-DOF complexity with RL?
- Do they use simplified models or full dynamics?
- What state representations work best?
- How is cable constraint enforced in learning?

---

## Next Steps

1. ✅ Understand overall control landscape
2. ⏳ Paper 4-7: Focus on learning-based methods
3. ⏳ Extract specific model equations from reference papers [50], [40], [36]
4. ⏳ Build composite 15-DOF model from insights across papers
5. ⏳ Implement in MuJoCo with validation
