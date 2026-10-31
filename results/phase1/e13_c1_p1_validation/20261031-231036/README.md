# phase1 / e13_c1_p1_validation

Step 1.3: C1 rigid massless cable (ball joint at drone CoM) + P1 point-mass payload, nominal L = 0.5 m, m_L = 0.2 kg; gates P1-P7.

- Started: 2026-10-31 23:10:36  (wall time 9.404 s)
- Code: commit `6abc0941ca4e36d96e1e5edd46946d151728aa38`  **(uncommitted changes present)**
- Command: `experiments/e13_c1_p1_validation.py`
- MuJoCo 3.17.0, Python 3.10.12

## Metrics

| metric | value |
|---|---|
| P1_static_hover.max_drift_m | 0.0 |
| P1_static_hover.max_swing_deg | 0.0 |
| P1_static_hover.tension_err_N | 0.0 |
| P1_static_hover.passed | True |
| P2_swing_period.L0.25_period_s | 0.9350642515742651 |
| P2_swing_period.L0.25_theory_s | 0.9349500007566899 |
| P2_swing_period.L0.25_rel_err | 0.00012219992243729045 |
| P2_swing_period.L0.5_period_s | 1.3223348901047758 |
| P2_swing_period.L0.5_theory_s | 1.3222189712108463 |
| P2_swing_period.L0.5_rel_err | 8.766996726973579e-05 |
| P2_swing_period.L1.0_period_s | 1.8700477932315487 |
| P2_swing_period.L1.0_theory_s | 1.8699000015133798 |
| P2_swing_period.L1.0_rel_err | 7.903723089433024e-05 |
| P2_swing_period.passed | True |
| P3_energy.energy_drift_J | 1.2677146798978356e-07 |
| P3_energy.max_length_err_m | 1.6653345369377348e-15 |
| P3_energy.passed | True |
| P4_length.max_length_err_m | 2.7755575615628914e-15 |
| P4_length.passed | True |
| P5_timestep.drone_pos_diff_m | 5.28771948828056e-08 |
| P5_timestep.payload_pos_diff_m | 4.131960788294347e-08 |
| P5_timestep.passed | True |
| P6_state_consistency.v_Q_err_dt1ms | 1.6043667145915208e-06 |
| P6_state_consistency.v_Q_err_ratio | 3.993635865593589 |
| P6_state_consistency.q_dot_err_dt1ms | 1.0002318559187984e-05 |
| P6_state_consistency.q_dot_err_ratio | 3.9963016186015126 |
| P6_state_consistency.v_L_err_dt1ms | 5.562708181194864e-06 |
| P6_state_consistency.v_L_err_ratio | 3.9958108897128466 |
| P6_state_consistency.tension_err_N_dt1ms | 7.386404073272956e-06 |
| P6_state_consistency.tension_err_N_ratio | 3.9958432322157305 |
| P6_state_consistency.passed | True |
| P7_offset_attachment.at_com_hover_drift_m | 0.0 |
| P7_offset_attachment.at_com_drone_ang_acc | 1.1102230246251565e-16 |
| P7_offset_attachment.offset_hover_drift_m | 0.0 |
| P7_offset_attachment.offset_drone_ang_acc | 1.0153989754695423 |
| P7_offset_attachment.passed | True |
| all_passed | True |

## Figures

- swing_period.png

## Notes

Gate passes only if all_passed is true. P6 is a convergence test (errors vs finite differences must shrink ~4x when dt halves).
