# phase2 / e26a_c2_validation

Step 2.6a: C2 tendon cable (0 <= length <= L, solref 4 ms) + P1, drone at hover thrust. Gates S1-S6.

- Started: 2026-11-03 12:49:44  (wall time 14.903 s)
- Code: commit `de134271ff30318bf2eb96c92d78fe10f8d8032c`  **(uncommitted changes present)**
- Command: `experiments/e26a_c2_validation.py`
- MuJoCo 3.17.0, Python 3.10.12

## Metrics

| metric | value |
|---|---|
| S1_static_hover.max_stretch_m | 1.0201906109230663e-05 |
| S1_static_hover.tension_err_N | 1.6525047996651665e-10 |
| S1_static_hover.passed | True |
| S2_swing_period.period_s | 1.3223224727200982 |
| S2_swing_period.theory_s | 1.3222189712108463 |
| S2_swing_period.rel_err | 7.827864484283215e-05 |
| S2_swing_period.min_tension_N | 1.9487936698971464 |
| S2_swing_period.passed | True |
| S3_c1_equivalence.max_pos_diff_m | 0.00010436602427714451 |
| S3_c1_equivalence.max_stretch_m | 5.1061527033491316e-05 |
| S3_c1_equivalence.min_tension_N | 1.6910724401116277 |
| S3_c1_equivalence.passed | True |
| S4_slack_and_retaut.slack_episodes | 2 |
| S4_slack_and_retaut.slack_time_s | 0.5750000000000001 |
| S4_slack_and_retaut.t_retaut_s | 0.5320000000000004 |
| S4_slack_and_retaut.min_tension_N | 0.0 |
| S4_slack_and_retaut.peak_snap_tension_N | 186.11275630315654 |
| S4_slack_and_retaut.tension_while_slack_max_N | 0.0 |
| S4_slack_and_retaut.ballistic_acc_err | 2.1154100693365763e-10 |
| S4_slack_and_retaut.snap_impulse_Ns | 0.5153948821303846 |
| S4_slack_and_retaut.max_stretch_m | 0.0038896562879322527 |
| S4_slack_and_retaut.min_length_m | 0.10144432037735607 |
| S4_slack_and_retaut.passed | True |
| S5_timestep.impulse_1ms_Ns | 0.5153948821303846 |
| S5_timestep.impulse_0.5ms_Ns | 0.6512099632336579 |
| S5_timestep.impulse_0.25ms_Ns | 0.6315725300501773 |
| S5_timestep.impulse_rel_err_0.5ms | 0.031092918468003012 |
| S5_timestep.impulse_rel_err_1ms | 0.18394981160843485 |
| S5_timestep.peak_tension_N | {0.001: 186.11275630315654, 0.0005: 252.74176770305337, 0.00025: 258.19234965627305} |
| S5_timestep.retaut_time_diff_s | 0.0005000000000311422 |
| S5_timestep.passed | True |
| S6_cost.C1_us_per_step | 14.504883799236268 |
| S6_cost.C2_us_per_step | 23.40394320199266 |
| S6_cost.ratio | 1.6135215921705597 |
| S6_cost.passed | True |
| all_passed | True |

## Figures

- c2_slack_retaut.png

## Notes

Gate passes only if all_passed is true.
