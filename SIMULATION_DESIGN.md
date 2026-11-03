# Simulation Design — Suspended-Payload Quadrotor (MuJoCo)

Decision log for the research simulation. Each decision states **what**, **why**, and **source**.
Status: design phase complete (D1–D12 agreed, reviewed), no code yet. Started 2026-10-31.

---

## D1 — Simulator: MuJoCo ✅ agreed
- **Why:** accurate multibody dynamics; standard in RL; GPU version (MJX) available for RL later (Papers 4–7);
  used by Paper 14 (MuJoCo-Drones-Gym), Crazyflow, Udaan.
- Installed version: MuJoCo 3.17.

## D2 — Drone: Skydio X2 (MuJoCo Menagerie) ✅ agreed
- **Source:** official `google-deepmind/mujoco_menagerie/skydio_x2` (Apache-2.0, geometry provided by Skydio).
- **Model as shipped:** mass 1.325 kg (4 × 0.25 kg rotors + 0.325 kg body), X configuration,
  4 rotor thrust actuators 0–13 N each, yaw torque ratio 0.0201 m, IMU sensors (gyro, accelerometer, quaternion),
  default timestep 0.01 s, air density 1.225 / viscosity 1.8e-5 (fluid drag on).
- **Measured in MuJoCo 3.17 (review, 2026-10-31):**
  | Property | Skydio X2 model | Paper 3 drone |
  |---|---|---|
  | Mass | 1.325 kg | 1.365 kg |
  | Inertia Ixx / Iyy / Izz (body axes) | 0.0367 / 0.0254 / 0.0605 kg m², Ixz = −0.0021 | 0.0216 / 0.0212 / 0.0217 kg m² |
  | CoM | 5.4 cm above model body origin | 8.0 cm below its body origin |
  | Rotor layout | non-square: x = ±0.14 m, y = ±0.18 m; front rotors 3 cm higher (z 0.08 vs 0.05) | "+", arm 21.07 cm |
  | Hover thrust | 3.25 N per rotor (range 0–13 N) | — |
- **Why chosen:**
  1. Official, maintained, reproducible model of a real drone.
  2. Closest available mass to Paper 3's drone (1.325 vs 1.365 kg), so Paper 3's payload/cable values are
     physically meaningful on it. **Inertia is 1.2–2.8× larger** → slower attitude dynamics than Paper 3's drone
     (a known difference, reported and used in the analysis).
  3. Per-rotor thrust actuation (same input structure as Paper 3).
  4. Thrust/weight ≈ 4 (headroom for agile manoeuvres and RL).
- **Known limitations (to report in the thesis):** inertia comes from simplified geometry (not system-identified);
  non-square rotor layout → roll and pitch control authority differ (mixer must use the real rotor positions);
  no motor lag, drag model only via MuJoCo fluid model, no downwash; timestep must be reduced (see D5).
- **Alternatives considered:**
  | Model | Source | Why not chosen |
  |---|---|---|
  | Crazyflie 2.x | Menagerie / Crazyflow / MuJoCo-Drones-Gym | 27–45 g, payload only ~10–15 g |
  | Holybro X500 V2 | Crazyflow (2026) | no thrust/motor parameters; 2.28 kg; no real drone available to us |
  | Udaan quadrotor | vkotaru/udaan | generic, not a real product (useful as cable reference) |
  | Paper 3 custom quad | Tolba & Shirinzadeh 2025, Table 1 | not available in MuJoCo; would be hand-built |
- No real hardware available → simulation-only study.

## D3 — Cable models: six variants, cable model is an experimental variable ✅ agreed
Literature taxonomy: single rigid pendulum, single elastic pendulum, chain of rigid links, chain of elastic links,
distributed models. Catenary/PDE models are excluded (meant for long, sagging outdoor cables).

| ID | Model | Slack? | Literature | MuJoCo implementation (planned) |
|---|---|---|---|---|
| **C1** | Rigid massless link (always taut) | no | Sreenath, Lee, Kumar 2013 (Paper 1) | ball joint at drone attachment |
| **C2** | Inextensible, slack-capable | yes | Sreenath, Michael, Kumar ICRA 2013 (hybrid); FLARE/ASTER theory; Impactor (T-RO) | spatial tendon, `limited`, `range="0 L"` (as in Udaan) |
| **C3** | Single elastic spring-damper, tension-only | yes | **Tolba & Shirinzadeh 2025 (Paper 3)** | spatial tendon spring with deadband `springlength="0 L"`; damping applied **only when taut** (custom) |
| **C4** | Chain of rigid links with mass | yes | Goodarzi & Lee 2013/2015; Kotaru, Wu, Sreenath 2017; **FLARE, ASTER (Papers 4–5)** | N capsule bodies + ball joints |
| **C5** | Chain of point masses with tension-only springs ("beads") | yes | arXiv 2305.09175; arXiv 2607.00024 | sphere bodies + deadband spring tendons |
| **C6** | Elastic rod (bending/twisting stiffness) | yes | Cosserat / spectral-method works (e.g. arXiv 2403.17565) | MuJoCo `mujoco.elasticity.cable` plugin |

**Preliminary check (MuJoCo 3.17, tendon with `springlength="0 0.25" stiffness=500 damping=3.5`):**
- slack, at rest → 0 N ✓; taut, stretched 5 cm → 25.0 N ✓; taut, stretching at 1 m/s → 21.5 N ✓
- slack but moving at 1 m/s → **3.5 N ✗** (native tendon damping also acts while slack, which violates Paper 3 Eq. 25)
  → for C3 the damping term must be applied only when taut.

**Cable parameters (review fixes A5, B1, B2):**
- **Axial stiffness scales with length** (real cable: k = EA/L). Keep Paper 3's **EA = k·L = 500 × 0.25 = 125 N**
  constant → k = 500 / 250 / 125 N/m for L = 0.25 / 0.5 / 1.0 m; damping scaled the same way
  (c·L = 3.5 × 0.25 = 0.875 N s constant → c = 3.5 / 1.75 / 0.875 N s/m).
  Bead/segment springs (C5) with 0.1 m segments: k_seg = EA / 0.1 = 1250 N/m.
- **Cable mass (C4, C5, C6):** nominal linear density **10 g/m** (light rope); ablation: near-massless (1 g/m) vs 50 g/m.
  C1–C3 are massless (as Papers 1 and 3).
- **C6 elastic rod:** axial stiffness from EA above; bending (and twisting) stiffness not available in the literature
  → chosen by a sweep in experiment X2 (natural dynamics) and documented.

**Implementation notes (review risks C1, C3):**
- C3 "damping only when taut" needs custom force code: Python callback (`mjcb_passive`) for CPU experiments;
  implemented inside the JAX step function for MJX/GPU (RL phase).
- C2 tendon length limit is a soft constraint (slight stretch): tune its `solref`/`solimp`; verify stretch < 1 mm in X1.

**Implementation order:** C1 → C2 → C3 (Papers 1–3), then C4 (Papers 4–5), then C5, C6.

## D4 — Cable-model experiments ✅ agreed
Same drone and payload in all experiments; only the cable model changes.

| ID | Question | Metrics |
|---|---|---|
| X1 | Static validation: correct equilibrium? | hanging position, tension = m·g |
| X2 | Natural dynamics vs pendulum theory | swing period, damping |
| X3 | Slack/taut event (payload thrown upward) | slack duration, peak (snap) tension |
| X4 | Controller sensitivity: same controller on C1–C6 | tracking RMSE, max/residual swing, failures |
| X5 | Computational cost / GPU (MJX) compatibility | steps per second |

Research value: quantifies how much controller results depend on the cable assumption (model-mismatch robustness),
and informs which cable model RL should be trained on.

## D5 — Timestep and control rates ✅ agreed
Literature:
| Source | Physics | Inner (rate) loop | Outer controller / policy |
|---|---|---|---|
| Paper 3 (Tolba) | 1000 Hz | — | 50 Hz (LQR + Kalman) |
| FLARE / ASTER (P4–5) | 100 Hz (Genesis) | Betaflight (hardware) | 100 Hz onboard |
| Leveling the Field (P15) | 100 Hz | — | 50 Hz |
| Adaptive outer-loop RL (P6, Isaac Lab) | 500 Hz | — | 50 Hz |
| MuJoCo-Drones-Gym (P14) | 240 Hz | — | 48 Hz |
| Agile fall recovery (real drone) | — | 200 Hz | 50 Hz |
| PX4 autopilot (real) | — | typically 800 Hz (gyro-synced; 250–2000 Hz observed) | position ~50 Hz |

Pattern: policy/outer loop 50–100 Hz; rate loop 200–800 Hz; physics ≥ 500 Hz when fidelity matters.
No paper isolates control-frequency effects on sim-to-real; the interface (CTBR) matters more (Kaufmann 2022).

Requirements from our design: motor lag τ_m = 35 ms (1 ms → 35 steps per time constant); elastic cable C3
(k = 500 N/m at 0.25 m with 0.2 kg → ω ≈ 50 rad/s); bead cable C5 is the stiffest case (k_seg = 1250 N/m with
0.1 m segments; at 10 g/m a bead is 1 g → single-bead ω = √(k/m) ≈ 1100 rad/s, highest chain mode ≈ 2√(k/m)
≈ 2200 rad/s → ω·dt ≈ 2.2 at 1 ms, close to RK4's stability limit (≈ 2.8) → C5 will likely need dt = 0.5 ms or
`implicitfast`; to be confirmed by the convergence test); elastic rod C6 may need smaller dt or the implicit integrator
(MuJoCo discussion #3329). No contacts → RK4 preferred for accuracy and energy conservation.

**Decision:**
| Layer | Rate | Reason |
|---|---|---|
| **Physics** | **1000 Hz** (dt = 1 ms), RK4 | Paper 3 plant rate; resolves motor lag and cable springs |
| Motor lag | integrated with physics (1000 Hz) | — |
| **Rate controller** | **500 Hz** | within PX4/Betaflight practice |
| **Outer controller / RL policy — main benchmark** | **100 Hz**, same for all controllers | FLARE/ASTER onboard rate; fairness |
| Paper 3 reproduction | 50 Hz (LQR + Kalman) | Paper 3 value |
| Paper 1 reproduction | 500 Hz CTBM | approximates continuous-time controller |
| Ablation | 50 Hz vs 100 Hz policy rate | sensitivity check |

**Safeguards:**
1. Timestep convergence test for every cable model (1 ms vs 0.5 ms must give the same results); if C5/C6 need it,
   use smaller dt or `implicitfast` and document it.
2. RL training on GPU (MJX) may use a coarser physics step — decided in the RL phase, only if the convergence
   test shows it is accurate enough.

**D5 exception (step 2.6a, agreed):** cable C2 runs use physics dt = 0.5 ms (snap impulse within 3 %; finding F6);
control rates unchanged.

## D6 — Payload models ✅ agreed
Literature taxonomy:

| ID | Payload model | Captures | Literature |
|---|---|---|---|
| P1 | Point mass | swing only | Sreenath 2013 (Paper 1); FLARE, ASTER (Papers 4–5); Kotaru 2017 |
| P2 | Rigid body, cable through CoM | payload rotation, decoupled from swing | Lee 2014; Wu & Sreenath CDC 2014 |
| P3 | Rigid body, cable attached off CoM | double-pendulum (payload rocks about attach point) | **Tolba 2025 (Paper 3)**; Goodarzi & Lee |
| P4 | Double pendulum (2 point masses) | simplified equivalent of P3 | Paper 3 (LQR design model); Estevez et al. 2021 |
| P5 | Uncertain / time-varying mass | adaptation problem | Papers 11–13; RoVerFly; arXiv 2607.00024 |
| P6 | Sloshing liquid container | moving CoM and inertia | L1Quad; Sayyadi & Soltani 2018; Paper 3 future work |

Related effects: payload aerodynamic drag (Paper 3 linear drag; arXiv 2601.03386 drag force + torque);
rotor downwash on payload (not modelled in Paper 3); cable attached off the drone CoM couples drone attitude
to swing and breaks differential flatness (Zeng & Sreenath ACC 2019).

**Decision:**
| Role | Payload | Reason |
|---|---|---|
| Main model A | **P1 point mass** | Papers 1, 4, 5; theoretical baseline |
| Main model B | **P3 rigid body, offset attachment** = Paper 3 cylinder geometry: I_D ≈ diag(1.0708e4, 1.0708e4, 2.1084e3) g cm² at 170 g, r_L/D = [0, 0, −5.33] cm, r_P/D = [0, 0, 0.74] cm (attach-to-CoM 6.07 cm) | Paper 3 re-implementation; realistic double-pendulum |
| Controller-design model only | P4 double pendulum | used inside LQR design (as Paper 3), not as simulated plant |
| Robustness study | P5 mass variation | handled in D11 (randomisation) and Papers 11–13 |
| Optional, later | payload drag; P6 sloshing | extensions, not first phase |

**Masses (Skydio X2 = 1.325 kg):** nominal **0.2 kg** (P1); sweep **0.1 / 0.2 / 0.35 / 0.5 kg**
(payload/drone ratio 0.08–0.38; literature: Goodarzi 0.05, Paper 3 0.12, FLARE 0.23, Paper 3 double pendulum 0.37).
At 0.5 kg the thrust/weight is still ≈ 2.9 (same as Paper 3's drone).
**Review fix A4:** in the benchmark, P3 is **scaled to the same nominal 0.2 kg** as P1 (inertia scaled proportionally),
so P1-vs-P3 comparisons isolate the payload model from the mass; the exact **170 g** is used only for the Paper 3
re-implementation. For the mass sweep, P3 is scaled the same way.

**Attachment point on drone (parameter), defined relative to the drone CoM (review fix A3):**
- **0 cm** — at the CoM: theory case (flatness holds) and Paper 3's Table 2 case (r_B/A = −7.7 cm with CG at
  −8.0 cm from Paper 3's body origin → attachment ≈ at the CG).
- **−7.7 cm** — 7.7 cm below the CoM: Paper 3's realistic case (r_B/A = −15.72 cm → 7.7 cm below its CG).
Paper 3's offsets are measured from its body origin A, not its CG; converting to CoM-relative offsets keeps the
physics (lever arm between attachment and CoM) correct on the X2.

**Core experiment grid:** 2 payloads (P1, P3) × cable models (C1–C6). P3 requires a single attachment point,
so it pairs with C2–C6 (C1 = rigid rod to a point mass only).

## D7 — Cable length(s) ✅ agreed
Literature:
| Paper | Length | Note |
|---|---|---|
| Tolba 2025 (Paper 3) | 0.25 m (elastic) | double-pendulum payload |
| Learning to Throw (arXiv 2606.27603) | 0.33 m | agile throwing, tennis ball |
| Goodarzi, Lee, Lee 2013 | 0.5 m (5 × 0.1 m links; hardware 0.5 m) | flexible cable |
| Cooperative manipulation (arXiv 2501.18802) | 1.0 m | 3 drones, 1.4 kg payload |
| FLARE (P4) | nominal not stated; tested ±50 % | success drops for longer cables (payload swings above drone) |
| ASTER (P5) | tested ±40 % | +20 % length → 82.5 % success vs 98.5 % nominal |
| Stevens RA-L 2024 | variable in flight | length control reduces swing and energy |

Physics (pendulum, ω = √(g/L)): 0.25 m → 6.3 rad/s (T = 1.0 s); 0.5 m → 4.4 rad/s (T = 1.4 s);
1.0 m → 3.1 rad/s (T = 2.0 s). Short cable = fast swing, strong coupling with drone attitude dynamics;
long cable = slow, large swing, needs space; drone disturbance grows with length and payload mass.

**Decision:**
| Role | Length | Reason |
|---|---|---|
| **Nominal** | **0.5 m** | middle of literature range; swing period 1.4 s |
| Paper 3 reproduction | **0.25 m** | Paper 3 value |
| Long-cable case | **1.0 m** | Paper 1 lineage / cooperative works; hardest case for agile RL |
| Robustness | ±40–50 % of nominal | as FLARE/ASTER; see D11 |

Experiment set: **0.25 / 0.5 / 1.0 m**.
For link/bead cables (C4, C5): fixed segment length ≈ **0.1 m**, so N scales with length
(0.25 m → 3 segments (rounded), 0.5 m → 5, 1.0 m → 10), keeping cable resolution constant.
**Review risk C2:** MJX (GPU) cannot batch models with different numbers of links → for RL training with randomised
length, use **fixed N = 5 with variable segment length**; evaluation keeps the 0.1 m rule above.
Stiffness and damping scale with length as defined in D3 (EA = 125 N constant).

## D8 — Actuation interface and motor model ✅ agreed
Literature:
| Interface | Commands | Used by | Evidence |
|---|---|---|---|
| SRT — single rotor thrusts | 4 rotor thrusts | Paper 3 (via allocation), MuJoCo-Drones-Gym (RPM) | Kaufmann et al. ICRA 2022: works in sim, poor sim-to-real |
| CTBM — collective thrust + body moments | thrust + 3 torques | Sreenath (Paper 1), Tolba (Paper 3), Leveling the Field (P15) | natural output of geometric control / LQR |
| CTBR — collective thrust + body rates | thrust + 3 body rates (inner rate loop) | FLARE, ASTER (P4–5), arXiv 2412.11764, PX4-based RL (RESC) | Kaufmann 2022: best balance of robustness and agility |
| LV — linear velocity | velocity setpoints | navigation works | needs full autopilot; too slow for agile swing control |

Motor modelling: thrust/torque ∝ rotor speed² (MuJoCo-Drones-Gym, Flightmare); first-order motor lag is the most
cited sim-to-real factor — E2E-Fly identified τ_m = 35 ms and reports instability without it (also Flightmare,
FlightGoggles, P15 "realistic" mode; asymmetric rise/fall in arXiv 2606.16513); rotor saturation; optional one-step
communication delay. Fairness (P15): classical and learned controllers get the same interface and information.

**Decision — one layered actuator stack used by every controller:**
```
Controller / RL policy
   │  I3: CTBR (collective thrust + body rates)   ← main benchmark interface
   ▼
Rate controller (PX4/Betaflight-style PID on body rates)
   │  I2: CTBM (collective thrust + 3 torques)    ← Papers 1 & 3 reproduction
   ▼
Allocation (mixer) + saturation 0–13 N per rotor
   │  I1: SRT (4 rotor thrusts)                   ← ablation only
   ▼
Motor model: first-order lag τ_m on each rotor thrust
   ▼
Skydio X2 (MuJoCo)
```
| Item | Decision |
|---|---|
| Papers 1 & 3 reproduction | CTBM interface, **ideal motors (τ_m = 0)** as the papers assume |
| Main benchmark (classical vs RL) | **CTBR for every controller**, motor lag **τ_m = 35 ms** (E2E-Fly value) |
| Motor-lag range | randomised around nominal in D11 (e.g. ±50 %); ablation τ_m = 0 vs 35 ms |
| Rotor limits | X2 model: 0–13 N per rotor, no reverse thrust |
| One-step action delay | optional realism setting, decided in D10 |
| SRT | ablation only (to show why CTBR is preferred) |

Rationale for CTBR: used by the RL papers and transfers best to real drones; same interface for classical and RL
(fair comparison, P15); matches real autopilot rate mode. Classical controllers (geometric, LQR) produce a desired
attitude, which converts directly to body-rate commands.
Caveat: X2's motor time constant is not identified; 35 ms is a literature value from another platform → randomised.

## D9 — Benchmark protocol: trajectories, metrics, statistics ✅ agreed
Literature: hover/initial-swing stabilisation (Paper 3, arXiv 1612.04324); rest-to-rest flatness trajectories
(Sreenath ICRA 2013); circle (Paper 3); figure-8 / Lissajous at several speeds (P6: period 15 → 5 s, RMSE
0.055 → 0.118 m; RAPTOR; P15); square paths (arXiv 2605.03666, 2508.11547); random held-out trajectories
(RoVerFly, NanoBench, TACO, P15); agile waypoints / gates (FLARE, ASTER).
Metrics in use: position RMSE (per axis + norm), max error, max swing, settling time normalised by pendulum period
(RoVerFly), success rate and unsafe state "payload above drone" (FLARE/ASTER), compute time (P15).
Statistics: 1000 trials (P15); 2000 Monte Carlo runs with median + 20–80 percentiles (arXiv 2207.04782);
multiple seeds (RoVerFly); matched conditions (NanoBench).
Gap: no shared protocol for payload controllers → defining one is itself a contribution.

**Reference:** payload position is the main tracked quantity; all references are smooth up to snap (needed by
flatness, Paper 1). For methods that track the drone (Paper 3 LQR), the drone reference is derived from the payload
reference, and drone tracking is also reported.

**Trajectory set:**
| ID | Trajectory | Purpose | Aggressiveness levels |
|---|---|---|---|
| T0 | Hover + disturbance: initial swing 15° / 30°; lateral gust impulse on payload | stabilisation (Paper 3 test) | 2 amplitudes |
| T1 | Rest-to-rest 1 m move (snap-smooth polynomial) | swing suppression, residual swing, settling | duration 4 / 2 / 1 s |
| T2 | Circle r = 1 m, counter-clockwise | steady tracking with constant tilt | peak payload accel 0.25 g / 0.5 g / 1 g |
| T3 | Figure-8 (Lissajous 1:2) | changing direction | period 15 / 10 / 5 s (as P6) |
| T4 | Random held-out set: 50 random Lissajous / sum-of-sinusoids, fixed seeds | generalisation; RL train/test split (test never seen in training) | randomised |
| T5 | Slack-inducing manoeuvre (fast descent > g, or payload thrown upward) | slack/taut handling | cable models C2–C6 only |

Aggressiveness is defined physically (peak acceleration as a fraction of g; period relative to pendulum period),
so results are comparable across cable lengths (RoVerFly idea).

**Trajectory details (review fix B5)** — final values fixed in the implementation spec:
- T0 gust: lateral force on the payload for 0.5 s. **Revised in step 2.5 (agreed):** main level = 0.25 g × payload
  mass (0.49 N for 0.2 kg), so it scales with mass; severe level = 2 N (original value; 1 g on 0.2 kg, beyond the
  0.5 m success limit even with ideal actuators).
- T3 figure-8: amplitude 1 m (x) × 0.5 m (y).
- T4 random set: amplitudes ≤ 1.5 m, frequencies capped so that peak payload acceleration ≤ 1 g; fixed seeds;
  separate tuning/training split and held-out test split.
- T5: (a) payload given 3 m/s upward velocity at hover; (b) commanded descent with acceleration > g.

**World and failure definitions (review fix B6):**
- No ground contact (contacts disabled); start altitude 2 m (payload well above z = 0 for all cable lengths).
- Crash = drone altitude < 0.3 m, or drone tilt > 90°, or numerical divergence.
- Swing angle = angle between the attachment-to-attachment line (drone attach → payload attach) and the vertical,
  computed the same way for all cable models; counted only while the cable is taut.

**Metrics:**
| Group | Metrics |
|---|---|
| Tracking | payload position RMSE (norm + per axis), max error, final error; drone RMSE |
| Swing | max swing, RMS swing, residual swing (after manoeuvre ends); **dynamic swing** = deviation from flatness-predicted cable direction (separates necessary tilt from unwanted oscillation) |
| Transient | settling time (5 cm band held 1 s), also in pendulum periods |
| Cable | min / peak tension, slack fraction, peak snap tension |
| Effort & limits | control effort, rotor-saturation fraction, max drone tilt |
| Success | no crash; payload error always < 0.5 m; swing < 60°; payload never above drone |
| Compute | controller time per step |

**Statistics:** each configuration over 20 seeds (initial conditions, noise); T4 over 50 trajectories;
identical seeds and conditions for every controller; report median + 20–80 percentiles, mean ± std, and success rate.

**Experiment matrix (review risk C4):** the full factorial (payloads × cables × lengths × masses × trajectories ×
levels × controllers × seeds) is too large. Staged plan: a **core grid** at nominal values first, then
**one-factor-at-a-time sweeps** (cable model, length, mass, realism level), then joint random robustness (D11).
Exact stages defined in the implementation plan.


## D10 — Aerodynamics, sensing noise, latency, state estimation ✅ agreed
Literature:
- Drag: linear drag on drone and payload in Paper 3 (coefficients not published); rotor drag linear in velocity,
  keeps flatness (Faessler et al. RA-L 2018, platform-specific coefficients); near-linear at low speed
  (Hattenberger et al. IMAV 2022); drag force + torque on payload (arXiv 2601.03386); rotor downwash on payload
  excluded by Paper 3 (stated limitation).
- **The Menagerie X2 file already sets `density=1.225`, `viscosity=1.8e-5`** → MuJoCo's built-in fluid model gives
  shape-based drag on drone and payload without invented coefficients.
- Noise / latency (RL sim-to-real): MuJoCo-Drones-Gym recommends pos 1 cm, vel 5 cm/s, attitude 1°, gyro 0.05 rad/s,
  sensor latency 0–20 ms, action latency 0–50 ms, and names motor lag + action latency the biggest unmodelled
  effects; tilt-rotor RL (arXiv 2602.21583) ±1 cm, ±5 mm/s, 3–5 steps @ 10 ms; Crazyflie studies ~0.001–0.002;
  Paper 3: process noise cov 5e-4, measurement noise cov 1.5e-3. No consensus on magnitudes.
- Payload state: measured by motion capture (FLARE/ASTER); payload attitude measured but cable length estimated
  (Paper 3); sensorless EKF from drone IMU (arXiv 2605.03666; Taki & Umemoto arXiv 2608.18625, no payload parameters;
  Bologna KF flight-validated; MDPI Drones EKF on Pixhawk). Links to Research Question 2 (minimum state for swing
  estimation without extra sensors).

**Decision — three realism levels:**
| Level | State the controller sees | Aerodynamics | Use |
|---|---|---|---|
| **L0 Ideal** | perfect full state (drone + payload) | none | reproduction of Papers 1 & 3, debugging, theory checks |
| **L1 Realistic** (main benchmark) | noisy drone **and payload** measurements (mocap-like): pos 1 cm, vel 5 cm/s, attitude 1°, gyro 0.05 rad/s; one-step action delay (10 ms) | MuJoCo fluid model on (drone + payload) | main comparison of all methods |
| **L2 Sensorless payload** (extension) | drone sensors only (IMU + drone pose); payload state estimated (EKF as arXiv 2605.03666 / 2608.18625) | as L1 | Research Question 2; possible contribution |

**Rules:**
- Fairness: at each level every controller receives the same raw measurements; a method may include its own filter
  (e.g. Paper 3's Kalman filter), documented as part of the method.
- Paper 3 reproduction additionally uses its own noise covariances and 50 Hz Kalman filter.
- Rotor downwash: excluded, documented limitation (as Paper 3); possible future work.
- Rotor drag (Faessler-type): not modelled separately; covered by drag randomisation in D11.
- Wind: only the T0 gust; constant wind optional later.

## D11 — Randomisation ranges (robustness tests and RL training) ✅ agreed
Literature:
- Which parameters matter (arXiv 2412.11764, ablation of 4 parameters): **thrust coefficient matters most**
  (randomising it significantly improves robustness); mass and inertia are measurable → identify, randomising them
  makes training harder; motor time constant: weak sensitivity, no significant benefit from randomising.
- Ranges used: MuJoCo-Drones-Gym mass/inertia ×[0.8, 1.2], thrust coeff ×[0.85, 1.15], motor lag 5–50 ms,
  latency sensor 0–20 ms / action 0–50 ms; P15 ±20/40 %; AirDreamer mass ×[0.9, 1.1], inertia and T/W ×[0.85, 1.15];
  SkyJEPA mass ±50 %, inertia ±30 %, motor lag 10–100 ms; RoVerFly payload 0–0.2 kg, cable 0–1.0 m (plus drone mass,
  inertia, CoM, motor gain); ASTER eval payload mass and cable length ±40 %; FLARE ±50 %, initial swing ±10°.
- Too much randomisation hurts: racing improved from 30 % → 10 % and failed at 0 % (arXiv 2504.21586);
  ±10–20 % suggested as practical (arXiv 2603.05944); robustness costs speed/accuracy; curriculum / adaptive schedules
  (DORAEMON) widen ranges gradually.

**Principle:** drone parameters narrow (±10–15 %, identifiable on a real drone); payload and cable wide (task variable,
adaptation is a core research goal); evaluation range wider than training range → out-of-distribution (OOD) testing.

| Parameter | Nominal | RL training range | Evaluation (in-dist.) | OOD evaluation | Source |
|---|---|---|---|---|---|
| Payload mass | 0.2 kg | U[0.1, 0.5] kg | 0.1 / 0.2 / 0.35 / 0.5 (D6) | 0.05, 0.65 kg | RoVerFly, FLARE, ASTER |
| Cable length | 0.5 m | U[0.25, 1.0] m | 0.25 / 0.5 / 1.0 (D7) | 1.25 m | RoVerFly, FLARE, ASTER |
| Thrust coefficient / max thrust | X2 model | ×[0.85, 1.15] | ±15 % | ±25 % | arXiv 2412.11764, MuJoCo-Drones-Gym |
| Motor lag τ_m | 35 ms | [17.5, 52.5] ms (±50 %, D8) | ±50 % | 70 ms | MuJoCo-Drones-Gym, SkyJEPA |
| Drone mass, inertia | X2 model | ×[0.9, 1.1] | ±10 % | ±20 % | AirDreamer, arXiv 2603.05944 |
| Drone CoM offset | 0 | ±1 cm | ±1 cm | ±2 cm | RoVerFly; Paper 3 offsets |
| Action latency | 10 ms (1 step) | 0–20 ms | 0–20 ms | 30 ms | MuJoCo-Drones-Gym |
| Cable axial stiffness / damping (C3, C5, C6) | EA = 125 N, c·L = 0.875 N s (D3) | ±30 % | ±30 % | ±50 % | our choice (not in literature) |
| Cable linear density (C4–C6) | 10 g/m | — (fixed) | 1 / 10 / 50 g/m ablation | — | our choice (B1) |
| Air drag (fluid density scale) | 1.0 | ×[0.7, 1.3] | ±30 % | — | our choice; covers rotor drag (D10) |
| Initial swing | 0 | 0–15° | per T0 (15° / 30°) | — | FLARE (±10°), Paper 3 |
| Sensor bias (L1) | 0 | per-episode bias | same | — | MuJoCo-Drones-Gym |

**Evaluation procedure:**
- One-at-a-time sweeps (as ASTER Table III) → which parameter each controller is sensitive to.
- Joint random sampling (as P15, 1000 samples) → overall robustness.
- Classical controllers designed on nominal values, evaluated on the same samples as RL (fairness).
- RL training: fixed ranges vs curriculum (DORAEMON-style) — decided in the RL phase.

## D12 — Controller tuning protocol and reproduction scope ✅ agreed (review fixes B3, B4)
- **Tuning (fairness, P15):** all classical controller gains are tuned with **Optuna, the same trial budget for every
  method, on the same objective**, using only the tuning/training split of T4 (never the test trajectories).
  RL uses the same training split. Rate-controller gains are tuned once on the bare X2 and then frozen for all methods.
- **Reproduction scope:** Paper 3's LQR weights are unpublished and its drone differs from the X2, so its exact
  numbers (11.4°, 14.65°, t_s = 3.52 s) cannot be matched. "Paper 3 reproduction" therefore means a
  **re-implementation of the method on the X2** (qualitative comparison). Optional sanity check: a Paper-3-drone
  parameter set (Table 1 values) in the same framework. The same applies to Paper 1.
- Papers 1 and 3 in the CTBR benchmark: geometric control produces a desired attitude → converted to body-rate
  commands; LQR is (re)designed on the plant augmented with the rate loop and motor lag (numerical linearisation).

---

## Reference values from Paper 3 (Tolba & Shirinzadeh, RAS 193, 2025, 105060)
**Drone (Table 1, "+" quadcopter):** m = 1.365 kg; arm 21.07 cm; b = 1.1165e-5 N s²; d = 1.6778e-7 N m s²;
c = d/b = 0.01503 m; f_max = 9.842 N per motor; payload capacity 1.722 kg;
CG offset [0.57, −0.35, −80.15] mm; Ixx, Iyy, Izz = 0.0216, 0.0212, 0.0217 kg m²;
Ixy, Ixz, Iyz = 74.9, −512.55, 276.2 g cm².

**Cable / payload (Table 2, chosen arbitrarily by authors):** cable natural length 25 cm, k = 500 N/m, c = 3.5 N s/m,
attachment on drone [0, 0, −7.7] cm (realistic case: [0, 0, −15.72] cm); 1st bob 300 g; 2nd bob 200 g, arm 20 cm.
Realistic payload: cylinder ≈ 170 g, I_D ≈ diag(1.0708e4, 1.0708e4, 2.1084e3) g cm²,
r_L/D = [0, 0, −5.33] cm, r_P/D = [0, 0, 0.74] cm; equivalent pendulum l = 6.07 cm, m_P = 0.001 kg.

**Model / control:** Newton–Euler, 15 DOF (30 states); simplified double pendulum 11 DOF (22 states);
LQR with output weighting (LQRY) linearised at hover with taut cable; inputs [f_T, τ_p, τ_q, τ_r];
plant 1000 Hz, controller + Kalman filter 50 Hz; process noise cov 5e-4, measurement noise cov 1.5e-3;
cable radial states not measured; start [0, 0, 0.55] m, counter-clockwise trajectory.
**Not published:** Q/R/N weights, drag coefficients, exact trajectory. Downwash not modelled.
**Reported results:** stabilisation t_s = 3.52 s, peak angles 11.4° / 14.65°; tracking peaks 3.17° / 3.54°.

## Simulation setups of other papers (for comparison)
| Paper | Simulator | Drone | Payload / cable | Timestep / control | Action |
|---|---|---|---|---|---|
| Goodarzi, Lee, Lee 2013 (arXiv 1309.6717)* | custom | 0.5 kg, J = diag(5.57, 5.57, 10.5)e-3; hardware 0.755 kg | 5 links × 0.1 kg × 0.1 m; hardware 36 g, 0.5 m | — | thrust + moment |
| FLARE (P4) | Genesis | 305 g, T/W 3.5 | 70 g; chain of rigid links | 0.01 s; policy 100 Hz onboard | collective thrust + body rates |
| ASTER (P5) | Genesis | same as FLARE | chain of rigid links; tested mass/length ±40 % | 0.01 s; 100 Hz | collective thrust + body rates |
| MuJoCo-Drones-Gym (P14) | MuJoCo/MJX | Crazyflie 27 g; RACE 250 g | none | 240 Hz sim, 48 Hz ctrl | RPM / thrust / velocity / PID |
| Leveling the Field (P15) | Isaac Lab | aerial manipulator | none | 100 Hz sim, 50 Hz ctrl | thrust + moments |

\* The guide's Paper 1 link (arXiv 1309.6717) is Goodarzi et al.; the user's Paper 1 is Sreenath et al. CDC 2013 (resolved).

---

## Open decisions (next)
- [x] D5 — Timestep and control rates (agreed, see above)
- [x] D6 — Payload models (agreed, see above)
- [x] D7 — Cable lengths (agreed, see above)
- [x] D8 — Actuation interface and motor model (agreed, see above)
- [x] D9 — Benchmark protocol (agreed, see above)
- [x] D10 — Aerodynamics, noise, latency, estimation (agreed, see above)
- [x] D11 — Randomisation ranges (agreed, see above)

- [x] D12 — Controller tuning protocol and reproduction scope (added in review)

**All design decisions D1–D12 agreed; full review done and fixes applied (2026-10-31). Next: implementation plan.**
Resolved: Paper 1 = Sreenath, Lee, Kumar, CDC 2013 (user's notes describe it; only the link in the guide/notes,
arXiv 1309.6717, points to Goodarzi et al.). Paper PDF: https://hybrid-robotics.berkeley.edu/publications/CDC2013.pdf

## Review log (2026-10-31)
| # | Type | Fix |
|---|---|---|
| A1 | error | X2 inertia is 1.2–2.8× Paper 3's → "transfer directly" reworded; measured values added (D2) |
| A2 | error | non-square X2 rotor layout documented; mixer uses real rotor positions (D2) |
| A3 | error | attachment offsets redefined relative to drone CoM (Paper 3 offsets were from body origin, CG at −8 cm) (D6) |
| A4 | error | P3 scaled to 0.2 kg nominal in benchmark; 170 g only for Paper 3 re-implementation (D6) |
| A5 | error | cable stiffness/damping scale with length (EA = 125 N, c·L = 0.875 N s); bead stiffness note corrected (D3, D5) |
| B1 | gap | cable linear density 10 g/m nominal; 1 / 50 g/m ablation (D3, D11) |
| B2 | gap | C6 bending stiffness chosen by sweep in X2 (D3) |
| B3 | gap | equal-budget Optuna tuning on training split only (D12) |
| B4 | gap | "reproduction" = re-implementation on X2; optional Paper-3-drone check (D12) |
| B5 | gap | trajectory details: gust, figure-8 amplitude, T4 sampling, T5 cases (D9) |
| B6 | gap | no ground contact, start 2 m, crash and swing definitions (D9) |
| C1 | risk | C3 taut-only damping: Python callback (CPU) / JAX step (MJX) (D3) |
| C2 | risk | RL with link cable: fixed N = 5, variable segment length (D7) |
| C3 | risk | C2 soft tendon limit: tune solref/solimp, stretch < 1 mm (D3) |
| C4 | risk | staged experiment matrix instead of full factorial (D9) |
| — | cosmetic | D5 moved to numeric order |
| F1 | finding (Step 1.1) | MuJoCo fluid drag on X2 is quadratic: horizontal ≈ 0.10 N at 2 m/s, 0.63 N at 5 m/s; vertical 4.1 N at 5 m/s. Real rotor drag is ≈ linear at low speed (Faessler 2018) and likely several times larger at 1–3 m/s → MuJoCo probably underestimates low-speed horizontal drag beyond D11's ±30 %. Only affects L1/L2; decide in the L1 phase whether to add a linear rotor-drag term. |
| F2 | finding (Step 1.2) | Motor lag (D8) implemented as MuJoCo actuator dynamics. Under RK4, `dyntype=filter` is accurate (6e-9 N error, 1 ms vs 0.5 ms 7e-9 m), whereas `filterexact` is exact only under Euler/implicit integrators and has 1.6e-2 N error under RK4 → use `filter`. Also: `data.actuator_force` after `mj_step` refers to the start of the step; read the motor state from `data.act`. |
| F3 | finding (Step 2.2) | Paper 1 geometric controller on the X2 (C1+P1, L0): (i) Paper 1's time-scale separation needs **≈ 5× between loops** — with 3× (2/6/20 rad/s) the coupled loop is lightly damped (1 m step settles in 3.65 s vs 2.13 s predicted), with 2× it oscillates indefinitely, and a cable loop only 3× below attitude (10/30) is unstable; with ≥ 5× the closed loop matches the linear design (1.5/7.5/30: 2.87 s vs 2.83 s predicted). (ii) The X2's 13 N rotors **cap the attitude bandwidth at ≈ 30 rad/s**: at 40–60 rad/s even a 30° swing saturates the rotors (≈ 100 % of ticks) and the drone diverges, while the same gains converge with unlimited thrust. Paper 1 assumes unlimited inputs. → Paper 1 gains set to **1.5 / 7.5 / 30 rad/s** (approved); theory gates that exceed the X2's limits (K2, K5) use an `ideal_actuator` option; saturation effects studied in Step 2.4. |
| F4 | finding (Step 2.3) | Paper 1 ellipse (E1.1) on the X2: (i) flatness feasibility boundary (rotors within 0–13 N) **T₀\* = 2.12 s** for 0.5×1.0 m (load 3.0 m/s, 8.8 m/s²) and **2.61 s** for 1.0×2.0 m (4.8 m/s, 11.6 m/s²); T₀\* grows monotonically with size (1.35 s at A_y = 0.25 m → 2.92 s at 3 m). (ii) Closed loop above T₀\*: X2 identical to the ideal actuator, no saturation (≤ 1.6 mm). Slightly below T₀\* the mixer keeps tracking (42–51 % ticks saturated, ≤ 1.7 mm); further below the X2 diverges (main at 1.5 s, second at 2.0 s). (iii) At T₀ = 1.5 s **even the ideal actuator fails** (19 cm / 2.2 m): the reference needs ≈ 62 rad/s body rates and > 90° tilt below ≈ 1.25 s, beyond the 30 rad/s attitude loop → the limit is controller bandwidth (time-scale separation) as well as rotor thrust. (iv) The drone path shrinks inside the load ellipse as T₀ decreases (load swings outward). Ideal-actuator tracking gate applied for T₀ ≥ 2 s only. |
| F5 | finding (Step 2.4) | Paper 1 recovery (E1.2; drone + cable flipped θ about y, load 2 m off, ellipse 0.5×1.0 m T₀ = 5 s, 10 s): (i) ideal actuator, θ = 178°: reproduced — Ψ_R, Ψ_q, e_x → 0, inside 5 cm at 5.1 s, final 0.07 mm, 0.72 m height loss; but the rigid link **pushes** the load (tension down to −2.8 N, 9 % of the run) → depends on the always-taut assumption (input to 2.6, C2). (ii) X2 limits: 0° (2 m error only) and 10° recover in ≈ 3 s with no height loss; for θ ≥ 20° the controller demands 15–46 N m vs ≈ 2 N m available, rotors pinned, outcome **chaotic** (30° fails, 31° recovers; 20/29/30/80/90/140/178° not recovered in 10 s, falls up to 50 m; recovering cases lose 1–9 m). → gap: saturation-aware recovery control. |
| F6 | finding (Step 2.6a) | C2 (tendon, 0 ≤ length ≤ L, solref 4 ms) validated: hover stretch 0.01 mm, swing period vs theory 0.008 %, C1-equivalence 0.10 mm, exact zero tension and ballistic flight while slack, tension never negative, 1.6× C1 cost. **Snaps:** an inextensible cable's snap is an impulse — the peak snap tension (186–509 N) is set by constraint softness and timestep and is **not physical**; the snap impulse (≈ 0.64 N s = payload momentum change) converges (0.5 ms vs 0.25 ms: 3 %) but is under-resolved by ≈ 20 % at 1 ms; snap stretch ≈ 4 mm (taut stretch ≤ 0.05 mm). Stiffer limit (2 ms) halves stretch but worsens both. → **C2 runs use dt = 0.5 ms (D5 exception, agreed)**; C2 reports slack time and snap impulse, never peak tension; physical snap forces need elasticity (C3, Paper 3). |

## Sources
- MuJoCo Menagerie: https://github.com/google-deepmind/mujoco_menagerie
- Crazyflow: https://github.com/learnsyslab/crazyflow
- Udaan: https://github.com/vkotaru/udaan — https://udaan.readthedocs.io/
- Paper 3: https://www.sciencedirect.com/science/article/pii/S0921889025001460 (local copy: `3.pdf`)
- Sreenath, Michael, Kumar ICRA 2013: https://hybrid-robotics.berkeley.edu/publications/ICRA2013.pdf
- Kotaru, Wu, Sreenath 2017: https://arxiv.org/abs/1711.04895
- Goodarzi & Lee 2015: https://arxiv.org/pdf/1501.04910
- Elastic/flexible cables: https://arxiv.org/pdf/2305.09175 ; bead model: https://arxiv.org/pdf/2607.00024
- Spectral flexible cable: https://arxiv.org/pdf/2403.17565
- Payload models: Wu & Sreenath CDC 2014 https://hybrid-robotics.berkeley.edu/publications/CDC2014.pdf ;
  Zeng & Sreenath ACC 2019 (offset) https://hybrid-robotics.berkeley.edu/publications/ACC2019_QuadLoad_Offset.pdf ;
  Goodarzi & Lee rigid body https://arxiv.org/abs/1511.02180 ; off-center slung load https://arxiv.org/pdf/2601.03386 ;
  Estevez et al. 2021 double pendulum https://www.mdpi.com/2076-3417/11/12/5487 ; L1Quad https://arxiv.org/pdf/2302.07208 ;
  RoVerFly https://arxiv.org/pdf/2509.11149
- Cable length: Learning to Throw https://arxiv.org/pdf/2606.27603 ; cooperative manipulation https://arxiv.org/pdf/2501.18802 ;
  in-flight length control https://researchwith.stevens.edu/en/publications/in-flight-cable-length-control-for-improved-quadrotor-based-suspe/ ;
  optimal length (JIRS 2020) https://link.springer.com/doi/10.1007/s10846-020-01166-4
- Actuation: Kaufmann et al. ICRA 2022 https://arxiv.org/pdf/2202.10796 ; zero-shot sim-to-real study https://arxiv.org/pdf/2412.11764 ;
  E2E-Fly https://arxiv.org/pdf/2604.12916 ; Flightmare https://arxiv.org/pdf/2009.00563 ; FlightGoggles https://arxiv.org/pdf/1905.11377 ;
  agile fall recovery https://arxiv.org/pdf/2606.16513 ; motor-delay system ID https://arxiv.org/abs/2404.07837 ;
  RESC (PX4 RL) https://arxiv.org/pdf/2408.00275
- Rates: PX4 rate loop https://github.com/PX4/PX4-Autopilot/pull/14759 ; PX4 forum https://discuss.px4.io/t/problem-of-mc-rate-controller-sampling-rate/23884 ;
  MuJoCo cable timestep discussion https://github.com/google-deepmind/mujoco/discussions/3329 ;
  adaptive outer-loop RL https://arxiv.org/pdf/2605.16015 ; Crazyflie RL https://arxiv.org/pdf/2306.03951
- Benchmark: RoVerFly https://arxiv.org/pdf/2509.11149 ; uncertain load https://arxiv.org/pdf/1612.04324 ;
  sensorless NMPC https://arxiv.org/pdf/2605.03666 ; onboard estimation https://arxiv.org/pdf/2508.11547 ;
  NanoBench https://arxiv.org/pdf/2603.09908 ; RAPTOR https://arxiv.org/pdf/2509.11481 ;
  Monte Carlo statistics https://arxiv.org/pdf/2207.04782 ; TACO https://arxiv.org/pdf/2511.02060 ;
  Sreenath & Kumar RSS 2013 https://www.roboticsproceedings.org/rss09/p11.html
- Estimation / noise / drag: swing estimation without payload parameters https://arxiv.org/pdf/2608.18625 ;
  Bologna swing estimation https://dspace-erf.nlr.nl/items/d1413050-f289-4672-8db2-ed7dfcc653f0 ;
  EKF on Pixhawk https://www.mdpi.com/2504-446x/7/11/654 ; Faessler rotor drag https://arxiv.org/abs/1712.02402 ;
  Hattenberger drag https://www.imavs.org/papers/2022/4.pdf ; tilt-rotor RL noise https://arxiv.org/pdf/2602.21583 ;
  practical DRL controller https://arxiv.org/pdf/2406.08815
- Randomisation: What Matters https://arxiv.org/pdf/2412.11764 ; SkyJEPA https://arxiv.org/pdf/2606.23444 ;
  AirDreamer https://arxiv.org/pdf/2606.03252 ; Crazyflie Brushless https://arxiv.org/pdf/2603.05944 ;
  One Net to Rule Them All https://arxiv.org/pdf/2504.21586 ; curriculum racing https://arxiv.org/pdf/2602.24030 ;
  RoVerFly v2 https://arxiv.org/html/2509.11149v2
- FLARE: https://arxiv.org/abs/2508.09797 ; ASTER: https://arxiv.org/abs/2603.10715
- MuJoCo-Drones-Gym: https://arxiv.org/abs/2606.08039 ; Leveling the Field: https://arxiv.org/html/2506.17832
