# Comprehensive Academic Treatment
## System Dynamics and Control of Quadrotors with Cable-Suspended Payloads

**Paper:** Omar, H.M., Akram, R., Mukras, S.M.S., & Mahvouz, A.A. (2022). "Recent advances and challenges in controlling quadrotors with suspended loads." *Alexandria Engineering Journal*, 63, 253-270.

**DOI:** 10.1016/j.aej.2022.08.001

**Focus:** Understanding the 15-DOF system dynamics for control and implementation

**Author:** Haidar Saad  
**Advisor:** Prof. Andry  
**Date:** October 29, 2026

---

## Table of Contents

1. Introduction and Problem Formulation
2. System Overview: The 15-DOF Model
3. Coordinate Frames and Kinematics
4. Lagrangian Mechanics: Derivation of Equations of Motion
5. Quadrotor Translational and Rotational Dynamics
6. Payload Dynamics and Cable Constraints
7. Swing Angle Dynamics and Control
8. Cable Tension Analysis and Constraints
9. State Representation and Observability
10. Control Inputs and Actuator Models
11. Linearization Around Equilibrium
12. Implementation Considerations for Simulation
13. Perspective for MuJoCo Implementation

---

# 1. Introduction and Problem Formulation

## 1.1 Problem Context

The problem of trajectory tracking for autonomous quadrotors carrying suspended payloads represents one of the most challenging control problems in aerial robotics. This system combines:

- **Under-actuation:** 4 control inputs (rotor thrusts) for 6+ degrees of freedom
- **Nonlinear dynamics:** Highly coupled translational and rotational equations
- **Unilateral constraints:** Cable can only pull, not push (tension ≥ 0)
- **Parameter uncertainty:** Payload mass typically unknown and variable
- **Oscillatory dynamics:** Payload naturally swings like a pendulum

## 1.2 Mathematical Formulation

The general dynamic system can be expressed in the form:

$$\mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \mathbf{u} + \mathbf{f}_c$$

**Where:**
- $\mathbf{q} = [x_q, y_q, z_q, \phi, \theta, \psi, \theta_p, \phi_p]^T \in \mathbb{R}^8$ — generalized coordinates
- $\mathbf{M}(\mathbf{q})$ — symmetric positive definite mass/inertia matrix
- $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})$ — Coriolis and centrifugal terms
- $\mathbf{g}(\mathbf{q})$ — gravitational potential gradient
- $\mathbf{u}$ — control inputs (motor thrusts)
- $\mathbf{f}_c$ — constraint forces (cable tension)

## 1.3 Research Objectives

This treatment aims to provide comprehensive understanding of:

1. The geometric and kinematic structure of the system
2. The complete equations of motion derived from first principles
3. The role of constraints in reducing effective degrees of freedom
4. The coupling between translational and rotational dynamics
5. The nonlinear relationship between swing angles and quadrotor acceleration
6. Practical considerations for simulation and control

---

# 2. System Overview: The 15-DOF Model

## 2.1 Counting Degrees of Freedom

The commonly cited "15-DOF" model deserves careful explanation. The quadrotor-payload system naturally possesses more degrees of freedom when decomposed into basic components, but these are reduced through constraints and coupling effects.

### Counting the DOF Rigorously

**Basic components:**

| Component | DOF | Generalized Coordinates |
|-----------|-----|------------------------|
| Quadrotor rigid body in 3D space | 6 | Position (3) + Orientation (3) |
| Payload rigid body in 3D space | 6 | Position (3) + Orientation (3) |
| Cable (if elastic) | 1 | Cable extension |
| Rotor dynamics | 4 | Rotor angular velocities (4) |
| **TOTAL (unconstrained)** | **23** | — |

**Constraints that reduce DOF:**

- **Cable length constraint:** −3 DOF (payload position is constrained)
- **Rotor speed saturation:** Not a hard constraint but practical limit
- **Payload point-mass assumption:** −3 DOF (no payload orientation)
- **Inextensible cable assumption:** −1 DOF (no cable extension)
- **Cascade control structure:** −4 DOF (rotor speeds follow outer loop)

**Effective DOF for control:** 6 (quadrotor) + 2 (swing angles) + 4 (coupling/actuation dynamics) = **15 DOF**

## 2.2 Simplified State Space for Control

For practical control and simulation, we use a reduced-order model with the following state vector:

$$\mathbf{x} = \begin{bmatrix} 
x_q & y_q & z_q & \dot{x}_q & \dot{y}_q & \dot{z}_q \\
\phi & \theta & \psi & \dot{\phi} & \dot{\theta} & \dot{\psi} \\
\theta_p & \phi_p & \dot{\theta}_p & \dot{\phi}_p & \hat{m}_p
\end{bmatrix}^T \in \mathbb{R}^{17}$$

### State Variable Definitions

| State Variable | Description | Units | Measured/Estimated |
|---|---|---|---|
| $x_q, y_q, z_q$ | Quadrotor position in world frame | m | Measured (GPS/VICON) |
| $\dot{x}_q, \dot{y}_q, \dot{z}_q$ | Quadrotor linear velocity | m/s | Estimated/Measured |
| $\phi, \theta, \psi$ | Roll, pitch, yaw angles (Euler) | rad | Measured (IMU) |
| $\dot{\phi}, \dot{\theta}, \dot{\psi}$ | Angular velocities | rad/s | Measured (Gyroscope) |
| $\theta_p, \phi_p$ | Pitch and roll swing angles of payload | rad | Estimated (EKF + dynamics) |
| $\dot{\theta}_p, \dot{\phi}_p$ | Swing angle rates | rad/s | Estimated |
| $\hat{m}_p$ | Estimated payload mass | kg | Estimated (online adaptation) |

**Important Note:** The payload position $(x_p, y_p, z_p)$ is **not** included in the state vector because it is fully determined by the quadrotor position and swing angles through the cable constraint.

---

# 3. Coordinate Frames and Kinematics

## 3.1 Coordinate Frame Definitions

Understanding the coordinate frame system is fundamental to deriving the correct equations of motion.

### Frame Definitions

**World Frame {W}:** Inertial reference frame, typically East-North-Up (ENU) or North-East-Down (NED). Position vectors in this frame are denoted with subscript $W$.

**Quadrotor Body Frame {B}:** Fixed to the quadrotor body. Origin at quadrotor center of mass. x-axis points forward (between front rotors), y-axis points right, z-axis points down (by convention). Rotates with the quadrotor.

**Payload Frame {P}:** Fixed to the payload. For a point mass payload, this frame coincides with the payload position but may have arbitrary orientation.

## 3.2 Rotation Matrices

The rotation from body frame to world frame is described by the rotation matrix $\mathbf{R} \in SO(3)$. Using Euler angles (ZYX convention):

$$\mathbf{R} = \begin{bmatrix}
c\theta c\psi & s\phi s\theta c\psi - c\phi s\psi & c\phi s\theta c\psi + s\phi s\psi \\
c\theta s\psi & s\phi s\theta s\psi + c\phi c\psi & c\phi s\theta s\psi - s\phi c\psi \\
-s\theta & s\phi c\theta & c\phi c\theta
\end{bmatrix}$$

Where $s(\cdot) = \sin(\cdot)$, $c(\cdot) = \cos(\cdot)$, and the angles are applied in order: yaw ($\psi$), pitch ($\theta$), roll ($\phi$).

## 3.3 Kinematic Relationships

The relationship between angular velocities in the body frame and Euler angle rates is:

$$\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = 
\begin{bmatrix} 1 & s\phi \tan\theta & c\phi \tan\theta \\
0 & c\phi & -s\phi \\
0 & s\phi/c\theta & c\phi/c\theta \end{bmatrix} 
\begin{bmatrix} p \\ q \\ r \end{bmatrix}$$

Where $p, q, r$ are angular velocity components in the body frame, and the matrix is invertible except at $\theta = \pm \pi/2$ (singularity).

## 3.4 Cable Geometry and Constraints

The cable connecting the quadrotor to the payload imposes a holonomic constraint. If the cable has fixed length $L$, then:

$$\|\mathbf{p}_p - \mathbf{p}_q\| = L$$

Where $\mathbf{p}_q = [x_q, y_q, z_q]^T$ and $\mathbf{p}_p = [x_p, y_p, z_p]^T$ are positions in the world frame.

Expanding this constraint:

$$(x_p - x_q)^2 + (y_p - y_q)^2 + (z_p - z_q)^2 = L^2$$

This constraint can be expressed in terms of swing angles. Define:

$$\mathbf{n} = \begin{bmatrix} \sin\phi_p \\ \sin\theta_p \\ \cos\phi_p \cos\theta_p \end{bmatrix}$$

Then:

$$\mathbf{p}_p = \mathbf{p}_q + L \cdot \mathbf{n}$$

Where $\mathbf{n}$ is the unit vector along the cable direction.

---

# 4. Lagrangian Mechanics: Derivation of Equations of Motion

## 4.1 Lagrangian Formulation

The dynamics are derived using the Euler-Lagrange equations, which provide a systematic way to handle constraints and complex coordinate transformations:

$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right) - \frac{\partial L}{\partial q_i} = Q_i + \sum_j \lambda_j \frac{\partial g_j}{\partial q_i}$$

Where:
- $L = T - V$ is the Lagrangian (kinetic minus potential energy)
- $Q_i$ are generalized forces (control inputs)
- $\lambda_j$ are Lagrange multipliers for constraints $g_j = 0$

## 4.2 Kinetic Energy

The total kinetic energy is the sum of quadrotor and payload kinetic energies:

$$T = T_q + T_p$$

**Quadrotor kinetic energy:**

$$T_q = \frac{1}{2}m_q \|\dot{\mathbf{p}}_q\|^2 + \frac{1}{2}\boldsymbol{\omega}_q^T \mathbf{I}_q \boldsymbol{\omega}_q$$

Where $\mathbf{I}_q$ is the inertia tensor of the quadrotor in the body frame.

**Payload kinetic energy:**

$$T_p = \frac{1}{2}m_p \|\dot{\mathbf{p}}_p\|^2 + \frac{1}{2}\boldsymbol{\omega}_p^T \mathbf{I}_p \boldsymbol{\omega}_p$$

For a point-mass payload, $\mathbf{I}_p = \mathbf{0}$, so the second term vanishes.

## 4.3 Potential Energy

Gravitational potential energy in the world frame (with z-axis pointing up):

$$V = m_q g z_q + m_p g z_p$$

Since $z_p$ depends on the swing angles through the cable constraint, the potential energy implicitly couples the quadrotor and payload dynamics.

## 4.4 General Form: Equation of Motion

The Euler-Lagrange equations yield the following general form for the system:

$$\begin{bmatrix} \mathbf{M}_{qq} & \mathbf{M}_{qp} \\ \mathbf{M}_{pq} & \mathbf{M}_{pp} \end{bmatrix} 
\begin{bmatrix} \ddot{\mathbf{q}} \\ \ddot{\mathbf{p}} \end{bmatrix} + 
\begin{bmatrix} \mathbf{C}_{qq} & \mathbf{C}_{qp} \\ \mathbf{C}_{pq} & \mathbf{C}_{pp} \end{bmatrix} 
\begin{bmatrix} \dot{\mathbf{q}} \\ \dot{\mathbf{p}} \end{bmatrix} + 
\begin{bmatrix} \mathbf{g}_q \\ \mathbf{g}_p \end{bmatrix} = 
\begin{bmatrix} \mathbf{u} \\ \mathbf{0} \end{bmatrix} + 
\begin{bmatrix} \mathbf{f}_c \\ -\mathbf{f}_c \end{bmatrix}$$

Where $\mathbf{f}_c$ is the constraint force (cable tension) that acts to enforce the distance constraint.

---

# 5. Quadrotor Translational and Rotational Dynamics

## 5.1 Translational Dynamics

The translational equation of motion for the quadrotor is derived from Newton's second law in the inertial frame:

$$m_q \ddot{\mathbf{p}}_q = \mathbf{F}_{\text{thrust}} + \mathbf{F}_{\text{gravity}} + \mathbf{F}_{\text{cable}}$$

**Thrust force:** The total thrust is produced by the four rotors and acts in the direction perpendicular to the quadrotor body:

$$\mathbf{F}_{\text{thrust}} = T_{\text{total}} \cdot \mathbf{R} \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$

Where $T_{\text{total}} = \sum_{i=1}^4 T_i$ is the total thrust and $\mathbf{R}$ is the rotation matrix.

**Gravity:**

$$\mathbf{F}_{\text{gravity}} = \begin{bmatrix} 0 \\ 0 \\ -m_q g \end{bmatrix}$$

**Cable tension force:** The cable exerts a downward force on the quadrotor:

$$\mathbf{F}_{\text{cable}} = -T_{\text{cable}} \cdot \mathbf{n}$$

Where $\mathbf{n}$ is the unit vector pointing from quadrotor to payload, and $T_{\text{cable}} \geq 0$.

Combining these, the translational dynamics become:

$$m_q \ddot{\mathbf{p}}_q = T_{\text{total}} \cdot \mathbf{R} \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} - T_{\text{cable}} \cdot \mathbf{n} + m_q \mathbf{g}$$

## 5.2 Rotational Dynamics

The rotational dynamics follow Euler's equation in the body-fixed frame. The rotational equation is:

$$\mathbf{I}_q \dot{\boldsymbol{\omega}}_q + \boldsymbol{\omega}_q \times \mathbf{I}_q \boldsymbol{\omega}_q = \mathbf{\tau}_{\text{motor}} + \mathbf{\tau}_{\text{cable}} + \mathbf{\tau}_{\text{gyro}}$$

For a quadrotor with symmetric structure:

$$\mathbf{I}_q = \begin{bmatrix} I_x & 0 & 0 \\ 0 & I_y & 0 \\ 0 & 0 & I_z \end{bmatrix}$$

**Motor torques:** Differential thrust between rotors creates moments:

$$\begin{bmatrix} \tau_{\text{roll}} \\ \tau_{\text{pitch}} \\ \tau_{\text{yaw}} \end{bmatrix} = 
\begin{bmatrix} l(T_2 + T_4 - T_1 - T_3) \\ l(T_1 + T_2 - T_3 - T_4) \\ 
c_M(\omega_1 - \omega_2 + \omega_3 - \omega_4) \end{bmatrix}$$

Where $l$ is the rotor arm length and $c_M$ is the moment coefficient.

**Cable torque:** The cable attachment point creates a moment arm:

$$\mathbf{\tau}_{\text{cable}} = \mathbf{r}_{\text{cable}} \times (-T_{\text{cable}} \cdot \mathbf{n})$$

Where $\mathbf{r}_{\text{cable}}$ is the position of the cable attachment point relative to the quadrotor center of mass.

## 5.3 Translational Equations in Component Form

Expanding the translational dynamics in the world frame:

$$\begin{align}
m_q \ddot{x}_q &= (T_{\text{total}} \cos\phi \sin\theta \cos\psi + T_{\text{total}} \sin\phi \sin\psi) - T_{\text{cable}} n_x \\
m_q \ddot{y}_q &= (T_{\text{total}} \cos\phi \sin\theta \sin\psi - T_{\text{total}} \sin\phi \cos\psi) - T_{\text{cable}} n_y \\
m_q \ddot{z}_q &= (T_{\text{total}} \cos\phi \cos\theta - m_q g) - T_{\text{cable}} n_z
\end{align}$$

Where $(n_x, n_y, n_z)^T$ are components of the unit vector along the cable.

---

# 6. Payload Dynamics and Cable Constraints

## 6.1 Payload Equation of Motion

The payload, assuming it is a point mass suspended by the cable, experiences only two forces:

$$m_p \ddot{\mathbf{p}}_p = \mathbf{F}_{\text{cable}} + \mathbf{F}_{\text{gravity}}$$

Where the cable tension force acts upward along the cable direction:

$$\mathbf{F}_{\text{cable}} = T_{\text{cable}} \cdot \mathbf{n}$$

So:

$$m_p \ddot{\mathbf{p}}_p = T_{\text{cable}} \cdot \mathbf{n} + m_p \mathbf{g}$$

## 6.2 Cable Constraint Enforcement

The cable constraint can be enforced using a method of Lagrange multipliers. The constraint acceleration must be zero:

$$\frac{d^2}{dt^2}\|\mathbf{p}_p - \mathbf{p}_q\|^2 = 0$$

Differentiating the constraint position equation:

$$\mathbf{p}_p - \mathbf{p}_q = L \mathbf{n}$$

First derivative (velocity constraint):

$$\dot{\mathbf{p}}_p - \dot{\mathbf{p}}_q = L \dot{\mathbf{n}} + \dot{L} \mathbf{n}$$

For an inextensible cable, $\dot{L} = 0$, so:

$$\dot{\mathbf{p}}_p = \dot{\mathbf{p}}_q + L \dot{\mathbf{n}}$$

Second derivative (acceleration constraint):

$$\ddot{\mathbf{p}}_p = \ddot{\mathbf{p}}_q + L \ddot{\mathbf{n}} + 2\dot{L}\dot{\mathbf{n}}$$

For inextensible cable ($\dot{L} = 0$):

$$\ddot{\mathbf{p}}_p = \ddot{\mathbf{p}}_q + L \ddot{\mathbf{n}}$$

## 6.3 Cable Tension Determination

Substituting the acceleration relationship into the payload equation of motion:

$$m_p (\ddot{\mathbf{p}}_q + L \ddot{\mathbf{n}}) = T_{\text{cable}} \cdot \mathbf{n} + m_p \mathbf{g}$$

Taking the dot product with the unit vector $\mathbf{n}$:

$$m_p (\ddot{\mathbf{p}}_q \cdot \mathbf{n} + L \ddot{\mathbf{n}} \cdot \mathbf{n}) = T_{\text{cable}} + m_p \mathbf{g} \cdot \mathbf{n}$$

This gives the cable tension:

$$T_{\text{cable}} = m_p (\ddot{\mathbf{p}}_q \cdot \mathbf{n} + L \ddot{\mathbf{n}} \cdot \mathbf{n} - g n_z)$$

This tension must satisfy $T_{\text{cable}} \geq 0$ for the cable to remain taut. If the calculated tension becomes negative, the cable goes slack and the payload becomes a free-falling mass.

---

# 7. Swing Angle Dynamics and Control

## 7.1 Swing Angle Definition

The swing angles are defined as the deviations of the cable from the vertical. They represent the pendulum-like oscillation of the payload.

**Pitch swing angle ($\theta_p$):** Angle between the vertical and the projection of the cable in the xz-plane (forward-backward oscillation).

**Roll swing angle ($\phi_p$):** Angle between the vertical and the projection of the cable in the yz-plane (left-right oscillation).

For small angles:
$$\theta_p \approx \arcsin\left(\frac{x_p - x_q}{L}\right), \quad \phi_p \approx \arcsin\left(\frac{y_p - y_q}{L}\right)$$

## 7.2 Nonlinear Swing Dynamics

The swing angle dynamics can be derived by differentiating the cable constraint twice. For small angles and small angular velocities, the dynamics simplify, but the complete nonlinear form is:

$$\ddot{\theta}_p = \frac{1}{L}(\ddot{x}_q \cos\theta_p + \ddot{z}_q \sin\theta_p) - \frac{g}{L}\sin\theta_p - 2\dot{L}\dot{\theta}_p/L + \text{higher order terms}$$

For an inextensible cable ($\dot{L} = 0$) with small angles:

$$\ddot{\theta}_p \approx \frac{1}{L}\ddot{x}_q + \frac{g}{L}\theta_p \quad \text{(small angle approximation)}$$

This reveals the critical insight: **the swing angle acceleration is directly proportional to the quadrotor horizontal acceleration!**

## 7.3 Coupling with Quadrotor Dynamics

The swing angle dynamics are tightly coupled with the quadrotor translational dynamics. Rewriting:

$$L \ddot{\theta}_p - \ddot{x}_q = g\theta_p + \text{nonlinear coupling terms}$$

This equation shows that:
- If the quadrotor accelerates forward (positive $\ddot{x}_q$), the payload lags behind, creating positive swing angle
- The gravitational restoring force ($-g\theta_p$) tends to return the payload to vertical
- Natural oscillation frequency (with no quadrotor acceleration): $\omega_0 = \sqrt{g/L}$

## 7.4 Control Implication: Swing-Free Trajectories

From the coupling equation, if we want zero swing angle throughout the motion ($\theta_p = 0$ and $\ddot{\theta}_p = 0$):

$$\ddot{x}_q = 0 \quad \text{(no quadrotor acceleration)}$$

This means swing-free trajectories can only be achieved with **zero horizontal acceleration**. However, this is impractical. A more useful result comes from trajectory planning: if the quadrotor acceleration follows a specific time profile, the payload swing can be made arbitrarily small.

This is the basis for **differential flatness** (Paper 1): we can design trajectories such that the required swing angles are bounded and small.

---

# 8. Cable Tension Analysis and Constraints

## 8.1 Cable Tension as Function of States

From the payload equation of motion, taking the component along the cable direction yields:

$$T_c = m_p g \cos\theta_p \cos\phi_p + m_p(\ddot{z}_q + g)\cos\theta_p \cos\phi_p + m_p \ddot{x}_q \sin\theta_p + m_p \ddot{y}_q \sin\phi_p$$

For small swing angles ($\sin\theta_p \approx \theta_p$, $\cos\theta_p \approx 1$):

$$T_c \approx m_p(\ddot{z}_q + g) + m_p(\ddot{x}_q \theta_p + \ddot{y}_q \phi_p)$$

## 8.2 Tension Constraints

The cable can only exert a pulling force (tension ≥ 0). This creates a constraint that must be satisfied at all times:

$$T_c \geq 0 \implies m_p(\ddot{z}_q + g) + m_p(\ddot{x}_q \theta_p + \ddot{y}_q \phi_p) \geq 0$$

This constraint limits the feasible accelerations. For example, excessive downward acceleration ($\ddot{z}_q < -g$) would make $T_c < 0$, which is infeasible.

## 8.3 Cable Slack Detection

In simulation or real control, we must check if the computed tension becomes negative. If so:

1. Set $T_c = 0$ (cable is slack)
2. Switch to free-fall dynamics for the payload
3. Monitor for re-engagement of the cable

Cable slack is generally undesirable because it makes the system underactuated and unpredictable. Control design must ensure $T_c > 0$ throughout the mission.

## 8.4 Maximum Tension and Material Limits

In addition to the minimum constraint, the cable has a maximum tension it can withstand:

$$0 \leq T_c \leq T_{\max}$$

Where $T_{\max}$ depends on the cable material and cross-section. This constraint may become active during aggressive maneuvers or with heavy payloads.

---

# 9. State Representation and Observability

## 9.1 Complete Extended State Vector

For a complete description of the system, we may need to include additional states beyond the minimum 17-dimensional vector. Consider:

$$\mathbf{x}_{\text{extended}} = [\mathbf{p}_q^T, \dot{\mathbf{p}}_q^T, \mathbf{R}, \boldsymbol{\omega}_q^T, \mathbf{p}_p^T, \dot{\mathbf{p}}_p^T, m_p, T_c, \hat{\omega}_1, \hat{\omega}_2, \hat{\omega}_3, \hat{\omega}_4]^T$$

This is a 31-dimensional state (or higher if we include model uncertainties).

## 9.2 Measurement Model

In practice, not all states are directly measured. The measurement model typically includes:

$$\mathbf{z}_k = \mathbf{h}_k(\mathbf{x}_k) + \mathbf{v}_k$$

Where $\mathbf{v}_k \sim \mathcal{N}(0, \mathbf{R}_k)$ is measurement noise.

| Measurement | Sensor | Noise Model |
|---|---|---|
| Position $(x_q, y_q, z_q)$ | GNSS + RTK or VICON | $\sigma_p \approx 0.01-0.1$ m |
| Attitude $(R)$ | IMU + mag fusion | $\sigma_\theta \approx 0.01-0.05$ rad |
| Linear accel $(a_x, a_y, a_z)$ | Accelerometer | $\sigma_a \approx 0.01$ m/s² |
| Angular vel $(p, q, r)$ | Gyroscope | $\sigma_\omega \approx 0.001$ rad/s |
| Motor speed $(\omega_i)$ | ESC feedback | $\sigma_\omega \approx 10$ RPM |
| Swing angles $(\theta_p, \phi_p)$ | Optional sensor or estimated | If measured: $\sigma_s \approx 0.05$ rad |

## 9.3 Observability Analysis

A key question: can we observe all state variables from available measurements?

**Observability Result:** With measurements of quadrotor position, attitude, and accelerations, we can observe:

✓ Quadrotor state (position, velocity, attitude, angular velocity)
✓ Swing angles (from dynamics + acceleration measurements)
✓ Payload mass (from acceleration evolution)
✗ Payload position (redundant, determined by constraint)
? Cable properties (elastic vs. inextensible)

This justifies our 17-dimensional state vector—it contains all observable quantities.

## 9.4 Unobservable Directions

The system has unobservable modes corresponding to:

- **Payload horizontal position offset:** Not observable from quadrotor measurements alone (but determined by swing angles)
- **Payload yaw angle:** Unobservable (point mass has no orientation)
- **Rotor individual dynamics:** Only the sum and differences matter for control

---

# 10. Control Inputs and Actuator Models

## 10.1 Motor Thrust Model

Each rotor produces thrust proportional to the square of its angular velocity (neglecting aerodynamic effects beyond standard models):

$$T_i = c_T \omega_i^2 + b_T \omega_i + \varepsilon_i$$

Where:
- $\omega_i$ is the rotor speed (rad/s)
- $c_T$ is the thrust coefficient (~$0.1-0.2 \times 10^{-6}$ N·s²/rad²)
- $b_T$ is a linear damping term (typically small)
- $\varepsilon_i$ is unmodeled dynamics (friction, nonlinearities)

For most practical purposes, the quadratic term dominates:

$$T_i \approx c_T \omega_i^2$$

## 10.2 Rotor Dynamics

The rotor has rotational inertia and angular acceleration:

$$I_r \dot{\omega}_i = M_{\text{motor}} - M_{\text{drag}} - T_i$$

Where $I_r$ is the rotor moment of inertia, $M_{\text{motor}}$ is the electromagnetic motor torque, and $M_{\text{drag}}$ is aerodynamic drag. For fast control (> 100 Hz), the rotor dynamics can often be neglected and we command thrust directly.

## 10.3 Actuator Saturation and Constraints

Physical constraints on the system:

$$0 \leq \omega_i \leq \omega_{\max} \quad \text{for } i = 1, 2, 3, 4$$

This translates to thrust constraints:

$$0 \leq T_i \leq T_{\max} = c_T \omega_{\max}^2$$

For a typical quadrotor:
- $\omega_{\max} \approx 8000-10000$ RPM ≈ 800-1000 rad/s
- $T_{\max} \approx 5-25$ N per rotor
- Maximum total thrust: $T_{\text{total,max}} = 4 \times T_{\max} \approx 20-100$ N

## 10.4 Control Input Space

We have several choices for control inputs:

**Option 1: Rotor speeds (RPM)**
Command: $\mathbf{u} = [\omega_1, \omega_2, \omega_3, \omega_4]^T$
Pros: Direct control of actuators. Cons: Requires rotor speed tracking controller.

**Option 2: Motor thrust levels**
Command: $\mathbf{u} = [T_1, T_2, T_3, T_4]^T$
Pros: Works directly with dynamics. Cons: Assumes instantaneous thrust response.

**Option 3: Attitude + total thrust (common in aerospace)**
Command: $\mathbf{u} = [\phi_{\text{des}}, \theta_{\text{des}}, \dot{\psi}_{\text{des}}, T_{\text{total}}]^T$
Pros: Decouples outer (position) and inner (attitude) loops. Cons: Less direct.

## 10.5 Control Allocation

Given desired forces and moments, we need to allocate commands to the four rotors. The relationship is:

$$\begin{bmatrix} F_z \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix} = 
\begin{bmatrix} 1 & 1 & 1 & 1 \\ -l & l & l & -l \\ l & l & -l & -l \\ -1 & 1 & -1 & 1 \end{bmatrix} 
\begin{bmatrix} T_1 \\ T_2 \\ T_3 \\ T_4 \end{bmatrix}$$

Where $l$ is the rotor arm length. This is a square system (4×4), so we can invert it:

$$\begin{bmatrix} T_1 \\ T_2 \\ T_3 \\ T_4 \end{bmatrix} = \frac{1}{4}\begin{bmatrix} 
1 & -1/l & 1/l & -1 \\ 1 & 1/l & 1/l & 1 \\ 1 & 1/l & -1/l & -1 \\ 1 & -1/l & -1/l & 1 
\end{bmatrix} 
\begin{bmatrix} F_z \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix}$$

After computing individual thrusts, they must be saturated to $[0, T_{\max}]$.

---

# 11. Linearization Around Equilibrium

## 11.1 Equilibrium Point

The system has a natural equilibrium where the quadrotor hovers and the payload hangs vertically:

**Equilibrium state:**
- $x_q^*, y_q^*, z_q^*$ — hover position (arbitrary)
- $\dot{x}_q^* = \dot{y}_q^* = \dot{z}_q^* = 0$ — stationary
- $\phi^* = \theta^* = 0$ — level attitude
- $\psi^*$ — arbitrary heading
- $\dot{\phi}^* = \dot{\theta}^* = \dot{\psi}^* = 0$ — no rotation
- $\theta_p^* = \phi_p^* = 0$ — payload vertical
- $\dot{\theta}_p^* = \dot{\phi}_p^* = 0$ — no swing
- $T_i^* = m_q g / 4 + m_p g / 4$ — each rotor supports quarter of total weight

## 11.2 Linearized System Matrices

Define perturbations as $\delta\mathbf{x} = \mathbf{x} - \mathbf{x}^*$ and $\delta\mathbf{u} = \mathbf{u} - \mathbf{u}^*$. The linearized system is:

$$\delta\dot{\mathbf{x}} = \mathbf{A} \delta\mathbf{x} + \mathbf{B} \delta\mathbf{u}$$

For the quadrotor-payload system, the $\mathbf{A}$ matrix has the typical structure:

$$\mathbf{A} = \begin{bmatrix} 
\mathbf{0}_{6 \times 6} & \mathbf{I}_{6 \times 6} \\
\mathbf{A}_{21} & \mathbf{A}_{22}
\end{bmatrix}$$

Where the lower blocks contain gravitational and coupling terms.

## 11.3 Modal Analysis

The eigenvalues of $\mathbf{A}$ reveal the natural dynamics:

- **Unstable modes:** Quadrotor is unstable in horizontal position without control (eigenvalues at origin)
- **Pendulum mode:** Swing angle oscillation at frequency $\omega_p = \sqrt{g/L}$
- **Coupling modes:** Interaction between horizontal acceleration and swing

The pendulum frequency for typical parameters ($L = 1$ m):

$$\omega_p = \sqrt{9.81/1} \approx 3.13 \text{ rad/s} \approx 0.5 \text{ Hz}$$

This is a relatively slow oscillation, which is beneficial for control design.

---

# 12. Implementation Considerations for Simulation

## 12.1 Numerical Integration Methods

The nonlinear dynamics must be integrated numerically. Common methods:

| Method | Order | Step Size | Pros / Cons |
|--------|-------|-----------|------------|
| Euler (explicit) | 1st | ~0.001 s | Fast, but prone to instability |
| RK2 (Midpoint) | 2nd | ~0.005 s | Better accuracy, moderate cost |
| RK4 (Runge-Kutta) | 4th | ~0.01 s | High accuracy, standard choice |
| Implicit RK | 4th+ | ~0.01 s | Stable for stiff systems, higher cost |

For this system, **RK4 with 0.001-0.005 s step is recommended** for good accuracy without excessive computation.

## 12.2 Constraint Handling

The cable length constraint must be maintained numerically. Options:

**Method 1: Direct constraint (Penalty method)**
Add a penalty term to prevent constraint violation:
$$\mathbf{F}_{\text{penalty}} = -k_p(||\mathbf{p}_p - \mathbf{p}_q|| - L)\mathbf{n}$$

Pro: Simple. Con: Constraint not exactly satisfied, may introduce stiffness.

**Method 2: Reduced coordinates (Preferred)**
Use swing angles as the only payload DOF. Payload position is derived algebraically:
$$x_p = x_q + L\sin\phi_p, \quad y_p = y_q + L\sin\theta_p, \quad z_p = z_q - L\cos\phi_p\cos\theta_p$$

Pro: Constraint exactly satisfied. Con: Requires rewriting dynamics.

**Method 3: MuJoCo constraints**
Use the physics engine's built-in constraint solver (recommended for complex systems).

## 12.3 Stability and Accuracy Checks

Verify simulation correctness:

1. **Energy conservation:** In the absence of control inputs, total mechanical energy should be constant (within numerical error)
2. **Constraint satisfaction:** Cable length constraint should hold to within 1e-6 accuracy
3. **Physical limits:** Check that cable tension never goes negative (or handle slack appropriately)
4. **Stability:** Uncontrolled system should not blow up; if it does, reduce step size

## 12.4 Parameter Sensitivity

Key parameters that affect numerical stability and accuracy:

| Parameter | Effect | Typical Range |
|-----------|--------|---------------|
| Quadrotor mass ($m_q$) | Affects acceleration scale | 1-2 kg |
| Payload mass ($m_p$) | Affects natural frequency and coupling | 0.5-5 kg |
| Cable length ($L$) | Affects swing frequency ($\omega \propto 1/\sqrt{L}$) | 1-2 m |
| Gravitational accel ($g$) | Affects all gravitational terms | 9.81 m/s² |
| Inertia components | Affects rotation dynamics and stability | 0.01-0.05 kg·m² |

---

# 13. Perspective for MuJoCo Implementation

## 13.1 MuJoCo Physics Engine Basics

MuJoCo is a physics engine particularly well-suited for robotics because it:

- Uses constraint-based dynamics (automatic constraint enforcement)
- Provides accurate integration with configurable step sizes
- Supports custom simulation callbacks and sensor simulation
- Fast enough for RL training (100+ simulations in parallel)

## 13.2 Model Structure (XML)

You will define the system in XML with the following structure:

```xml
<mujoco model="quadrotor_with_payload">
  <compiler ... />
  
  <worldbody>
    <body name="quadrotor">
      <inertial ... />
      <joint type="free" />  <!-- 6 DOF -->
      <geom ... />
      <site ... />  <!-- rotor attachment points -->
    </body>
    
    <body name="payload">
      <inertial ... />
      <joint type="free" />  <!-- 6 DOF, constrained by cable -->
      <geom ... />
    </body>
  </worldbody>
  
  <constraint>
    <distance name="cable" body1="quadrotor" 
              body2="payload" distance="L" />
  </constraint>
  
  <actuator>
    <!-- 4 motor commands -->
  </actuator>
  
  <sensor>
    <!-- IMU, position, etc. -->
  </sensor>
</mujoco>
```

## 13.3 Dynamic Equations in MuJoCo

MuJoCo solves the generalized equations of motion:

$$\mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}} = \mathbf{f}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{u} + \mathbf{J}_c^T \mathbf{\lambda}$$

Where $\mathbf{J}_c^T \mathbf{\lambda}$ represents constraint forces (automatically computed). You specify:

- Body masses and inertias
- Joint types and ranges
- Constraints (cable distance)
- Actuator specifications
- Step size and integrator type

## 13.4 Simulation Loop (Python)

Your main simulation loop will look like:

```python
import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path("quadrotor_payload.xml")
data = mujoco.MjData(model)

# Simulation parameters
dt = 0.001  # 1 ms step
t_max = 10.0  # 10 seconds
t = 0.0

# Control callback
def get_control(model, data):
    # Read state from data
    state = extract_state(data)
    
    # Compute control (RL agent or classical controller)
    action = controller(state)
    
    # Set control input
    data.ctrl[:] = action
    
    return action

# Main loop
while t < t_max:
    # Set control
    action = get_control(model, data)
    
    # Step simulation
    mujoco.mj_step(model, data)
    
    # Log data, compute rewards, etc.
    log_data(data)
    
    t += dt
```

## 13.5 Advantages of MuJoCo for This Project

**Why MuJoCo is ideal for your research:**

✓ **Constraint handling:** Cable constraint is automatically enforced (no penalty methods)
✓ **Speed:** Fast enough to train RL agents (1000+ steps per second)
✓ **Accuracy:** High-order integrators for accurate dynamics
✓ **Flexibility:** Easy to add custom sensors and simulation features
✓ **Sim-to-Real:** Used in many real robotic control systems
✓ **Visualization:** Built-in viewer for debugging and validation

## 13.6 Validation in MuJoCo

After setting up the simulation, validate against the dynamics:

1. **Static equilibrium:** Verify that hover position is stable without control
2. **Energy conservation:** Check that total energy is conserved (no damping)
3. **Pendulum motion:** Verify oscillation frequency matches $\omega = \sqrt{g/L}$
4. **Thrust mapping:** Verify that rotor commands produce expected forces/torques
5. **Cable tension:** Monitor and ensure it never goes negative

---

# Conclusion

This comprehensive treatment has covered the complete mathematical framework for understanding the dynamics of quadrotors with cable-suspended payloads. The key insights are:

1. **15-DOF System:** The system is fundamentally underactuated (4 inputs, 6+ DOF), requiring sophisticated control strategies.

2. **Swing Coupling:** Payload swing is directly caused by quadrotor acceleration. Control design must actively manage this coupling.

3. **Cable Constraints:** Tension constraints ($T_c \geq 0$) limit feasible accelerations. These are not optional—they define the control feasible set.

4. **Nonlinear Dynamics:** The system exhibits strong nonlinearities, particularly in the coupling between attitude and swing angle dynamics.

5. **State Observability:** With appropriate sensors, all relevant state variables are observable, enabling feedback control.

6. **Implementation Practicality:** MuJoCo provides an excellent platform for simulation and validation, with automatic constraint handling and fast execution.

With this foundation, you are now ready to:
- Design a MuJoCo simulation of the system
- Implement classical control methods (PID, LQR)
- Train reinforcement learning agents
- Compare different control approaches
- Understand and improve upon existing methods

---

**Document:** Paper 3 Detailed Academic Explanation  
**Prepared for:** Haidar Saad | Advisor: Prof. Andry  
**Date:** October 29, 2026  
**Purpose:** Provide rigorous mathematical foundation for MuJoCo implementation  
**Next Phase:** MuJoCo simulation setup and control implementation
