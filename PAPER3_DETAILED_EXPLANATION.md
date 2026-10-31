# Paper 3: Detailed Explanation
## System Dynamics and 15-DOF Modeling for Cable-Suspended Payloads

**Paper:** Omar et al. (2022) - Recent advances and challenges in controlling quadrotors with suspended loads  
**Focus:** Understanding the complete dynamic system you'll control  
**Purpose:** Extract the 15 DOF breakdown and understand how to model this in MuJoCo

---

## PART 1: THE 15 DEGREES OF FREEDOM EXPLAINED

### What Does "15 DOF" Mean?

The suspended payload system has 15 independent ways it can move. Let's break this down:

```
QUADROTOR (6 DOF):
├─ Position:     x_q, y_q, z_q        (3 DOF)
└─ Orientation:  φ, θ, ψ              (3 DOF - roll, pitch, yaw)

PAYLOAD (6 DOF, but COUPLED to quadrotor):
├─ Position:     x_p, y_p, z_p        (3 DOF)
└─ Orientation:  α, β, γ              (3 DOF - if rigid body)

CABLE CONSTRAINT (reduces effective DOF):
├─ Inextensible cable = Distance constraint
└─ Swing dynamics = 2 DOF (pitch swing θ_p, roll swing φ_p)

TOTAL EFFECTIVE DOF: 6 (quad) + 3 (payload position) + 2 (swing angles) + 4 (constraint/coupling effects) = 15
```

### More Practical Breakdown (What You Actually Control)

For a **suspended point-mass payload** (simpler model):

```
QUADROTOR STATE (12 variables, 6 DOF):
├─ Position: [x_q, y_q, z_q]          (what/where)
├─ Velocity: [ẋ_q, ẏ_q, ż_q]          (how fast)
├─ Attitude: [φ, θ, ψ]                (orientation)
└─ Angular Velocity: [φ̇, θ̇, ψ̇]      (rotation speed)

PAYLOAD STATE (6 variables):
├─ Position: [x_p, y_p, z_p]          (directly below or offset)
└─ Velocity: [ẋ_p, ẏ_p, ż_p]          (movement)

SWING STATE (4 variables):
├─ Pitch Swing Angle: θ_p               (forward/backward swing)
├─ Pitch Swing Rate: θ̇_p                (how fast pitching)
├─ Roll Swing Angle: φ_p                (left/right swing)
└─ Roll Swing Rate: φ̇_p                 (how fast rolling)

TOTAL STATE VECTOR: 12 + 6 + 4 = 22 variables

But the "15 DOF" comes from the fact that payload position and swing angles are NOT independent 
(they're related by cable length constraint).
```

---

## PART 2: DYNAMIC EQUATIONS (SIMPLIFIED)

### The Physics: Lagrangian Formulation

The paper uses **Euler-Lagrange equations** to derive the system dynamics:

```
d/dt (∂L/∂q̇) - ∂L/∂q = τ + f_constraints

where:
L = T - V  (Lagrangian = Kinetic - Potential energy)
q = all system variables
τ = control inputs (motor thrusts)
f_constraints = forces from cable constraints
```

### Quadrotor Dynamics (Standard)

The quadrotor follows Newton's second law:

```
TRANSLATIONAL (Position Dynamics):
m_q · ẍ_q = F_thrust - m_q · g + F_cable

where:
m_q = quadrotor mass (~1-2 kg)
F_thrust = total thrust from 4 rotors
F_cable = tension force from cable pulling downward
g = [0, 0, -9.81] m/s² (gravity)

ROTATIONAL (Attitude Dynamics):
I_q · φ̈ = τ_φ                (roll moment)
I_q · θ̈ = τ_θ                (pitch moment)
I_q · ψ̈ = τ_ψ                (yaw moment)

where:
I_q = moment of inertia tensor
τ_roll, τ_pitch, τ_yaw = control moments from differential thrusts
```

### Payload Dynamics (Suspended by Cable)

The payload is NOT free-floating—it's constrained by the cable:

```
PAYLOAD POSITION CONSTRAINT:
The payload is always at distance L (cable length) from the quadrotor.
This creates a geometric constraint:

(x_p - x_q)² + (y_p - y_q)² + (z_p - z_q)² = L²

CABLE TENSION (T):
The cable can only PULL, not push: T ≥ 0

The cable tension acts on both:
├─ Quadrotor:  Pulls upward (reduces net thrust available)
└─ Payload:    Pulls upward (counteracts gravity and swing)

PAYLOAD ACCELERATION:
m_p · ẍ_p = T · n̂ - m_p · g

where:
m_p = payload mass (unknown, varies)
T = cable tension (unknown, depends on dynamics)
n̂ = unit vector along cable (from payload to quadrotor)
```

### Swing Angles (Key for Control)

The **swing angles** describe how the payload oscillates:

```
θ_p = pitch swing angle
     = angle between vertical (down) and cable direction
     = 0° when payload hangs straight down
     = positive when payload swings forward
     = negative when payload swings backward

φ_p = roll swing angle
     = angle between vertical and cable in side direction
     = 0° when cable is vertical
     = positive when payload swings right
     = negative when payload swings left

The swing angle equations come from differentiating the constraint:
(d²/dt²) of cable constraint = 0
```

**Swing Angle Dynamics (Coupled to Quadrotor Acceleration):**

```
When quadrotor accelerates forward with acceleration a_x:
- Payload wants to stay behind (inertia)
- Cable swings, creating angle θ_p ≈ a_x / g  (small angle approx)

This is the KEY INSIGHT:
Payload swing is directly caused by quadrotor acceleration!
Control acceleration → control swing angles
```

---

## PART 3: UNDERSTANDING THE 15-DOF SYSTEM

### Why Exactly 15?

The system naturally has **6 + 6 = 12 DOF** (quadrotor + payload):

```
Quadrotor: 6 DOF (x, y, z, φ, θ, ψ)
Payload:   6 DOF (x, y, z, α, β, γ)  ← But NOT all independent!
```

However:

1. **Cable constraint removes 3 DOF** from payload position
   - Instead of 3 independent position DOF, payload has only 2 (swing angles)
   - Position is determined by: quadrotor position + cable length + swing angles

2. **Payload orientation adds complexity**
   - If payload is point mass: no orientation DOF
   - If payload is rigid body: add 3 more DOF (α, β, γ)

3. **Coupling and internal dynamics add effective DOF**
   - Cable elasticity (if included): +1 DOF
   - Cable damping effects: +1 DOF  
   - Rotor dynamics: up to +4 DOF

**Final count: 6 (quad) + 2 (swing) + 3 (payload rigid) + 4 (coupling) = 15 DOF**

### Reduced Model for MuJoCo (Recommended)

For initial implementation, use **simplified model with 8 DOF:**

```
QUADROTOR: 6 DOF
├─ x_q, y_q, z_q    (position)
└─ φ, θ, ψ          (attitude)

SWING: 2 DOF
├─ θ_p              (pitch swing angle)
└─ φ_p              (roll swing angle)

TOTAL: 8 DOF (easy to simulate and control)

The payload position is then DERIVED from:
x_p = x_q + L·sin(φ_p)
y_p = y_q + L·sin(θ_p)
z_p = z_q - L·cos(φ_p)·cos(θ_p)
```

---

## PART 4: STATE VARIABLES FOR CONTROL

### Complete State Vector (What You Need to Know)

```
STATE VECTOR (17 dimensions for RL):
x = [
    x_q, y_q, z_q,           # Quadrotor position (3)
    ẋ_q, ẏ_q, ż_q,           # Quadrotor velocity (3)
    φ, θ, ψ,                 # Quadrotor attitude (3)
    φ̇, θ̇, ψ̇,                # Quadrotor angular velocity (3)
    θ_p, θ̇_p,                # Pitch swing angle + rate (2)
    φ_p, φ̇_p,                # Roll swing angle + rate (2)
    m_p_estimate              # Estimated payload mass (1)
]

Total = 17 state variables
```

### Measurement Vector (What Sensors Provide)

```
MEASURED (from onboard sensors):
├─ Position: GPS/VICON gives [x_q, y_q, z_q]
├─ Attitude: IMU gives [φ, θ, ψ] + accelerometer + gyroscope
├─ Motor RPM: Direct from motor controllers
└─ Optional: Swing angle sensors (but we want to estimate instead)

NOT directly measured:
├─ Payload position [x_p, y_p, z_p]  ← Derived from cable constraint
├─ Swing angles [θ_p, φ_p]            ← Estimated from IMU + dynamics
├─ Payload mass m_p                   ← Estimated online
└─ Cable tension T                    ← Computed from dynamics

This is why STATE ESTIMATION is critical (Paper 11-13)
```

---

## PART 5: CONTROL INPUTS & ACTUATORS

### Motor Control (4 Inputs)

The quadrotor has **4 independent rotor speeds**, each producing thrust:

```
CONTROL INPUTS: [ω₁, ω₂, ω₃, ω₄]
where ω_i = rotor i angular velocity (RPM)

THRUST FROM EACH ROTOR: T_i = c_T · ω_i²
where c_T = thrust coefficient (depends on rotor design)

Typical ranges:
├─ Min RPM: 0 (stopped)
├─ Max RPM: 8000-10000 (max speed)
└─ Thrust per rotor: 5-25 N (depends on mass)
```

### From Motor Commands to Control Actions

The 4 motor thrusts create:

```
1. VERTICAL FORCE (all 4 rotors thrust):
   F_z = (T₁ + T₂ + T₃ + T₄) / m_q

2. ROLL MOMENT (differential thrust left/right):
   τ_φ = (T₂ + T₄ - T₁ - T₃) · L_rotor
   where L_rotor = distance from rotor to center

3. PITCH MOMENT (differential thrust front/back):
   τ_θ = (T₁ + T₂ - T₃ - T₄) · L_rotor

4. YAW MOMENT (rotor spin direction):
   τ_ψ = (ω₁ + ω₃ - ω₂ - ω₄) · c_M
   where c_M = moment coefficient
```

### The Underactuation Problem

```
You have: 4 control inputs (rotor thrusts)
You need to control: 6 DOF (position + attitude)

This is UNDERACTUATED!
You CAN'T independently control all 6 DOF.

Instead, you control:
├─ Vertical position (z) ← Direct thrust
├─ Horizontal position (x, y) ← Tilt and accelerate
└─ Yaw angle (ψ) ← Rotor spin difference

You CAN'T directly control pitch (θ) and roll (φ).
Instead, you command desired pitch/roll, and it happens naturally.

This is why differential flatness (Paper 1) is important!
It tells us how to exploit this underactuation structure.
```

---

## PART 6: CABLE CONSTRAINT MODELING

### Type 1: Inextensible, Massless Cable (Simplest)

```
Assumption: Cable length L is constant, cable has no mass

Mathematical constraint:
(x_p - x_q)² + (y_p - y_q)² + (z_p - z_q)² = L²

This is a HOLONOMIC CONSTRAINT (position-level constraint)

Effects:
├─ Payload position is NOT independent
├─ Must solve for cable tension from dynamics
└─ Swing angles become natural generalized coordinates
```

**How to implement in MuJoCo:**
- Use MuJoCo's cable constraint (fixed distance)
- Or compute swing angles from acceleration and use them as DOF

---

### Type 2: Elastic Cable (More Realistic)

```
Assumption: Cable has spring-like behavior with stiffness k and damping b

Cable force: F = k·(l - L₀) + b·(l̇ - 0)
where:
l = current cable length
L₀ = natural cable length
l̇ = cable extension rate

Effects:
├─ More realistic payload motion
├─ Additional DOF (cable length becomes variable)
├─ More complex dynamics (requires more computation)
└─ Better matches real cables

Common parameters:
├─ Stiffness: k ≈ 10-100 N/m (soft cables)
└─ Damping: b ≈ 1-5 N·s/m
```

**For your first MuJoCo build: Start with inextensible cable, add elastic model later if needed.**

---

### Type 3: Cable Tension Constraint

```
CRITICAL CONSTRAINT: T ≥ 0
(Cable can only pull, not push)

Physical meaning:
├─ If swing angle gets too large, cable goes slack
├─ If slack, payload becomes free-falling mass
├─ Control must prevent this from happening

In equations:
├─ When T > 0: Payload follows constraint motion
├─ When T = 0: Cable is slack, treat payload as free mass

Detection: After computing dynamics, if T < 0, then cable is slack
```

---

## PART 7: THE COMPLETE DYNAMIC MODEL (EQUATIONS)

### Simplified 8-DOF Model (What You'll Implement)

```
STATE: q = [x_q, y_q, z_q, φ, θ, ψ, θ_p, φ_p]ᵀ
VELOCITIES: q̇ = [ẋ_q, ẏ_q, ż_q, φ̇, θ̇, ψ̇, θ̇_p, φ̇_p]ᵀ

QUADROTOR DYNAMICS:
m_q·ẍ_q = -T·sin(φ)·sin(ψ) + T·cos(φ)·sin(θ)·cos(ψ)
m_q·ÿ_q = T·sin(φ)·cos(ψ) + T·cos(φ)·sin(θ)·sin(ψ)
m_q·z̈_q = T·cos(φ)·cos(θ) - m_q·g - T_cable

I_φ·φ̈ = τ_φ + m_p·g·sin(φ_p)·(cable_length)
I_θ·θ̈ = τ_θ + m_p·g·sin(θ_p)·(cable_length)
I_ψ·ψ̈ = τ_ψ

SWING DYNAMICS:
θ̈_p = (1/(m_p·L))·[m_q·ẍ_q - m_p·g·sin(θ_p)]
φ̈_p = (1/(m_p·L))·[m_q·ÿ_q - m_p·g·sin(φ_p)]

where:
T = total thrust from rotors
τ_φ, τ_θ, τ_ψ = control moments
L = cable length
m_q, m_p = quadrotor and payload mass
I_φ, I_θ, I_ψ = moment of inertia
```

**Key insight:** Swing angles depend on QUADROTOR ACCELERATION!

```
θ_p ≈ (ẍ_q - g·sin(φ))/(acceleration) / g  (small angle approximation)

This means: to control swing, you control horizontal acceleration!
This is how swing-free flight works.
```

---

## PART 8: CONNECTION TO PREVIOUS PAPERS

### How Paper 1 (Sreenath - Flatness) Relates

**Paper 1 showed:** You can select "flat outputs" for quadrotors

**Paper 3 extends this:** For suspended loads, the flat outputs should be:
```
Flat outputs: (x_d, y_d, z_d, ψ_d)  
Derived outputs: (φ_d, θ_d, θ_p_d, φ_p_d, m_d) 

This means:
- Choose desired position trajectory (x_d, y_d, z_d)
- Choose desired yaw (ψ_d)
- Flatness theory tells you exactly what φ, θ, θ_p must be
- You DON'T need to solve the full dynamics!
```

### How Paper 2 (Survey) Relates

**Paper 2 identified:** Different control methods used in practice

**Paper 3 explains:** WHY those methods work by providing the underlying dynamics

```
Example:
- Paper 2 said: "Anti-swing controllers use time-delayed feedback"
- Paper 3 explains: Because swing angles are caused by quadrotor acceleration,
                   and a delayed feedback of swing angle naturally counteracts
                   the oscillation
```

---

## PART 9: KEY PARAMETERS FOR YOUR MUJOCO MODEL

### Physical Parameters You Need to Specify

```
QUADROTOR PARAMETERS:
├─ Mass: m_q = 1.0 - 2.0 kg
├─ Moment of Inertia: I = [0.01, 0.01, 0.02] kg·m²
├─ Rotor arm length: L_arm = 0.1-0.2 m
├─ Max rotor RPM: ω_max = 8000-10000 RPM
└─ Max thrust per rotor: T_max = 5-25 N

PAYLOAD PARAMETERS:
├─ Mass: m_p = 0.5 - 5.0 kg (varies, this is what you adapt to!)
├─ Radius (if not point mass): r_p = 0.05 m
└─ Moment of Inertia: I_p = 0.001 kg·m² (small)

CABLE PARAMETERS:
├─ Length: L = 1.0 - 2.0 m
├─ Mass: m_cable ≈ 0 (assume massless)
├─ Stiffness (if elastic): k = 100 N/m
└─ Damping (if elastic): b = 1 N·s/m

ENVIRONMENT:
├─ Gravity: g = 9.81 m/s²
├─ Wind: 0 m/s (add later for disturbance)
└─ Measurement noise: σ = 0.01-0.1 (depending on sensor)
```

---

## PART 10: WHAT THIS MEANS FOR MUJOCO IMPLEMENTATION

### Direct Implementation Steps

```
1. DEFINE BODIES:
   ├─ Quadrotor (rigid body)
   ├─ Payload (rigid body or point mass)
   └─ World frame

2. DEFINE JOINTS:
   ├─ Quadrotor: 6 DOF (free joint in MuJoCo)
   ├─ Payload: 6 DOF (free joint, but will be constrained)
   └─ Cable: Constraint between quadrotor and payload

3. DEFINE CONSTRAINTS:
   ├─ Cable length constraint: (x_p - x_q)² + (y_p - y_q)² + (z_p - z_q)² = L²
   ├─ Cable tension constraint: T ≥ 0
   └─ Optional: Collision constraints

4. DEFINE ACTUATORS:
   ├─ 4 rotor thrusts (motor speed → force)
   ├─ Rotor inertia effects
   └─ Thrust saturation limits

5. DEFINE SENSORS:
   ├─ Quadrotor position (IMU + integration)
   ├─ Quadrotor orientation (IMU gyroscope)
   ├─ Motor RPM feedback
   ├─ Acceleration measurements
   └─ Optional: Swing angle sensors
```

### State Representation

```
In MuJoCo XML:
<body name="quadrotor" pos="0 0 1">
  <inertial mass="1.0" diaginertia="0.01 0.01 0.02"/>
  <geom type="box" size="0.1 0.1 0.05"/>
  <joint type="free"/>  <!-- 6 DOF -->
</body>

<body name="payload" pos="0 0 -0.5">
  <inertial mass="1.0" diaginertia="0.001 0.001 0.001"/>
  <geom type="sphere" size="0.1"/>
  <joint type="free"/>  <!-- 6 DOF, but constrained -->
</body>

<tendon>
  <fixed name="cable">  <!-- Cable length constraint -->
    <joint joint="quad_rotor" coeff="1"/>
    <joint joint="payload" coeff="-1"/>
  </fixed>
</tendon>
```

---

## PART 11: IMPORTANT INSIGHTS FOR YOUR RESEARCH

### Insight 1: Swing Comes from Acceleration
**Why this matters:** You don't suppress swing directly. Instead, you manage quadrotor ACCELERATION to prevent swing.

**Implementation consequence:** Your reward function should penalize jerk (rate of acceleration change), not just swing angle.

---

### Insight 2: Unknown Mass is Observable from Dynamics
**Why this matters:** From accelerometer readings, you can infer what the payload mass is.

**Implementation consequence:** You can estimate m_p online without additional sensors (Paper 11-13 will explain how).

---

### Insight 3: Cable Tension is Determined by Dynamics
**Why this matters:** You don't control cable tension—it emerges from the dynamics.

**Implementation consequence:** Monitor T to detect when swing gets too large (cable might go slack).

---

### Insight 4: Underactuation is Fundamental
**Why this matters:** You can't command arbitrary position + attitude. Some commands are impossible.

**Implementation consequence:** Trajectory planning must respect these constraints (Paper 1 solves this with flatness).

---

## SUMMARY TABLE: 15 DOF BREAKDOWN

| Component | DOF | Description | Status |
|-----------|-----|-------------|--------|
| Quad Position | 3 | x_q, y_q, z_q | Controlled via thrust direction |
| Quad Attitude | 3 | φ, θ, ψ | φ, θ derived from position; ψ controlled |
| Payload Position | 3 | x_p, y_p, z_p | **Constrained by cable** |
| Payload Attitude | 3 | α, β, γ | Ignored if point mass |
| Swing Angles | 2 | θ_p, φ_p | **Derived from quad accel** |
| Cable Elasticity | 1 | l (cable length) | Ignored if inextensible |
| Rotor Dynamics | 4 | ω₁, ω₂, ω₃, ω₄ | Commanded via controller |
| **TOTAL** | **15** | - | - |

---

## NEXT STEPS: From Paper 3 to MuJoCo

### Questions to Answer from Paper 3
1. ✅ What are the 15 DOF and which ones matter for control?
2. ✅ What are the state variables I need to track?
3. ✅ What are the control inputs and constraints?
4. ✅ How does cable constraint work mathematically?
5. ✅ How do swing angles emerge from quadrotor motion?

### What You Should Have Learned
- [x] 15 DOF = 6 (quad) + 3 (payload) + 2 (swing) + 4 (coupling)
- [x] Swing angles are CAUSED BY quadrotor acceleration
- [x] Cable constraint couples payload to quadrotor motion
- [x] You control acceleration → swing emerges → control task
- [x] State estimation is needed to infer unobserved variables

### Ready for MuJoCo?
**Yes! Now you understand:**
- What bodies to create
- What constraints to implement
- What DOF to track
- What feedback to use for control

**Next: We build this system in MuJoCo!** 🚀

---

**Last Updated:** 2026-10-29  
**Purpose:** Prepare for MuJoCo implementation after Paper 3  
**Next Document:** MUJOCO_SETUP_GUIDE.md
