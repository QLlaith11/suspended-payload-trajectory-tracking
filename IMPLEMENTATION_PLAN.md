# Implementation Plan — Suspended-Payload Quadrotor Simulation

Companion to `SIMULATION_DESIGN.md` (the *why*). This file tracks the *how* and progress.
Created 2026-10-31. Status: **Phase 1 complete; Step 2.2 done.**

## Working rules (agreed)
- **Own implementation.** Udaan (BSD-3, Kotaru) is used only as an independent cross-check, never copied.
- **Approval after every step.** Each step ends with its validation gate; results are shown and discussed before the
  next step starts.
- **Version control:** git in the project folder; each run records the exact code version.
- **Python:** current system Python 3.10 with MuJoCo 3.17 (no separate environment).
- **Paper 1 scope:** all of steps 2.1–2.6.
- Every value in a config must trace back to a decision (D1–D12) in `SIMULATION_DESIGN.md`.

## Project layout
```
sim/            simulation code: model, cables, actuators, controllers, flatness, trajectories, metrics
configs/        one YAML per experiment (values traced to D1–D12)
experiments/    one script per experiment
tests/          unit tests (pytest)
results/<phase>/<experiment>/<timestamp>/
                config copy, git commit, raw data, metrics.json, figures, README.md
```
Tools: numpy, scipy, matplotlib, pyyaml, pytest (Optuna from Phase 3).

---

## Phase 0 — Setup
- [x] 0.1 Folder structure, git repository, `.gitignore` (results data, caches), base config, run/save utilities.
  **Gate:** a dummy run creates a complete results folder (config copy, commit id, metrics, figure, README).

## Phase 1 — Simulation core (only what Paper 1 needs)
| Step | Build | Validation gate |
|---|---|---|
| [x] 1.1 | Skydio X2 from Menagerie; dt = 1 ms, RK4; fluid drag switchable | hover at exact hover thrust → zero drift; mass, inertia, CoM equal the measured values (D2) |
| [x] 1.2 | Actuator stack: CTBM → mixer (real rotor positions) → 0–13 N limits; motor lag switchable (0 / 35 ms) | commanded thrust/torques reproduced exactly; motor step response matches τ_m |
| [x] 1.3 | Cable C1 (rigid massless link, ball joint) + payload P1 (point mass); attachment at drone CoM | X1: tension = m·g at rest; X2: swing period vs two-body pendulum formula; energy conserved with motors off; 1 ms vs 0.5 ms identical |
| [x] 1.4 | Snap-smooth trajectories, logging, metrics (D9) | unit tests: analytic derivatives vs numerical; metrics on synthetic signals |

## Phase 2 — Paper 1 re-implementation (Sreenath, Lee, Kumar, CDC 2013)
Setting: C1 + P1, ideal motors, L0 (perfect state), CTBM at 500 Hz, nominal 0.5 m / 0.2 kg unless stated.

| Step | Build / experiment | Validation |
|---|---|---|
| [x] 2.1 | Flatness map: load trajectory (derivatives up to 6th order) → cable direction q, tension T, quad position, attitude R, rates Ω | matches Udaan's flatness module; open-loop feed-forward on the simulator reproduces the load path |
| [x] 2.2 | Geometric controller, three modes: quad attitude → load attitude → load position | each mode stabilises from moderate errors; behaviour consistent with the paper's stability results |
| [x] 2.3 | E1.1 Flatness study: ellipse x_L = (A_x sin 2πt/T₀, A_y cos 2πt/T₀, A_z), ψ = 0, T₀ = 10 → 1 s | reproduce the paper's Fig. 3 trend; find T₀ at which the X2's 13 N rotor limit is reached (new result) |
| [x] 2.4 | E1.2 Recovery: quad flipped 178°, load 178°, large position error | recovery and tracking; note whether rotor limits allow it on the X2 |
| [x] 2.5 | E1.3 Benchmark subset: T0–T3 (D9) at nominal settings | first entries of the benchmark results table |
| [x] 2.6 | E1.4 Assumption-breaking: same controller on (a) slack cable C2, (b) L1 realism + CTBR + motor lag, (c) 0.25 m and 1.0 m cables | where Paper 1's assumptions fail → motivates later papers and the contribution |

**Deliverable:** Paper 1 results report (figures, tables, what matched, what did not, why).

## Later phases (detailed when reached)
- **Phase 3 — Paper 2 (survey → benchmark):** survey baselines (load-agnostic, anti-swing PD), Optuna tuning protocol (D12),
  full T0–T4 protocol.
- **Phase 4 — Paper 3:** cable C3 (taut-only damping) + payload P3, double-pendulum design model, LQR with output weighting
  + Kalman filter at 50 Hz.
- **Phase 5 — Cable study:** C4–C6 and experiments X1–X5 (including the C5 timestep check).
- **Phase 6+ — RL (Papers 4–7):** MJX, randomisation (D11).

## Progress log
| Date | Step | Result | Approved |
|---|---|---|---|
| 2026-10-31 | Plan | written and agreed | ✅ |
| 2026-10-31 | 0.1 Setup | 3/3 tests pass; dummy run folder complete (config, meta, metrics, data, figure, README) | ✅ |
| 2026-10-31 | 1.1 X2 model | G1–G5 pass (hover drift 0, free-fall err 4e-14 m, dt err 3e-10 m); 6/6 tests; drag finding noted | ✅ |
| 2026-10-31 | 1.2 Actuators | A1–A6 pass (wrench err 3e-14, lag err 6e-9 N, dt err 7e-9 m); 14/14 tests; FILTER instead of FILTEREXACT under RK4 (F2) | ✅ |
| 2026-10-31 | 1.3 C1 + P1 | P1–P7 pass (tension err 0, swing period vs theory ≤ 0.012 % for 0.25/0.5/1.0 m, energy drift 1.3e-7 J, length err 3e-15 m, dt err 5e-8 m, P6 converges at 4×); 23/23 tests | ✅ |
| 2026-10-31 | 1.4 Trajectories, metrics, logger | M1–M5 pass (derivatives vs symbolic oracle ≤ 1.4e-11 rel, ramp joins continuous (10× shrink), aggressiveness exact, metrics on known signals, logger bit-identical); 30/30 tests; T4/T5 deferred to Phase 3 | ✅ |
| 2026-11-01 | 2.1 Flatness map | F1–F6 pass (hover exact; EOM residual 1e-14; rates converge 4×; open-loop replay in MuJoCo within 1 cm for 2.9 s / >6 s / >6 s; Udaan agreement 1.1e-13; free-fall guard); 37/37 tests | ✅ |
| 2026-11-01 | 2.2 Geometric controller | K1–K7 pass (attitude decay 28.4 vs 27/s; cable 7.6 vs 6.75/s; 1 m step settles 2.87 vs 2.83 s; tracking ≤ 0.17 mm; ablation flat 0.05 mm / numdiff 12.5 mm / zero 0.67 m); gains revised to 1.5/7.5/30 (F3); 46/46 tests | ✅ |
| 2026-11-02 | 2.3 Ellipse study (E1.1) | A: Fig. 3 trend reproduced (tilt/torque grow as T₀ ↓, drone path shrinks inside load ellipse); B: X2 T₀\* = 2.12 s (0.5×1.0) / 2.61 s (1.0×2.0), size sweep monotone; C: X2 = ideal above T₀\*, diverges below; ideal fails at 1.5 s (bandwidth, F4); 49/49 tests | ✅ |
| 2026-11-03 | 2.4 Recovery (E1.2) | ideal actuator 178°: all errors → 0, 5 cm at 5.1 s, rod pushes 9 % of run; X2: ≤ 10° recovers cleanly, ≥ 20° chaotic, falls up to 50 m (F5); 51/51 tests (first reported as 53: one file was collected twice) | ✅ |
| 2026-11-03 | 2.5 Benchmark T0–T3 (E1.3) | 13 cases × (X2, ideal): all flat-feasible cases succeed, X2 = ideal (≤ 0.45 mm on T1–T3); T0 swing settles 1.6/2.0 s, gust (0.25 g·m_L) 1.25 s; T1 1 s infeasible (±570 N), fails on both; severe gust 2 N fails success; dynamic swing ≤ 0.01° on tracking; 1.4 ms/step; gust redefined (D9); 52/52 tests | ✅ |
| 2026-11-03 | 2.6a C2 slack cable | S1–S6 pass (hover stretch 0.01 mm, period 0.008 %, C1-equivalence 0.10 mm, exact slack + ballistic flight, tension ≥ 0, snap impulse converges at 0.5 ms); peak snap tension not physical, C2 runs at dt 0.5 ms (F6); 58/58 tests | ✅ |
| 2026-11-03 | 2.6a Paper 1 on C2 | benchmark T0–T3 = C1 (≤ 0.03 mm); T5(a) throw recovers (0.88 s slack, settles 2.44 s, payload above drone); 178° ideal-actuator recovery diverges (cable cannot push); X2 sweep 7/19 recover (C1 14/19) (F7); tension metrics skip first tick | ✅ |
| 2026-11-03 | 2.6b Cable length | 0.25 m works on all feasible cases; 1.0 m crashes on T0 swing 15°/30° (k_q ∝ l, F8); T₀\* 1.94 / 2.12 / 2.26 s; flat feasibility conservative | ✅ |
| 2026-11-03 | 2.6c-1 L1 + CTBR build | R1–R8 pass (noise ≤ 1 %, delay exact, CTBR = Eq. 21 to 1e-14, runner = step 2.5 exactly, rate τ 17.5 vs 18.5 ms, drag quadratic + payload C_D 0.47, seeds reproducible); CTBR costs ff torque (F9); F1 resolved; 66/66 tests | ✅ |
| 2026-11-04 | 2.6c-2 L1 one-factor study | 990 runs: nominal gains fail under sensor noise (0 %, velocity noise is the cause), lag breaks swing recovery only; lag-compatible gains never crash but cannot damp swing (F10); checkpoint/resume added | ✅ |
