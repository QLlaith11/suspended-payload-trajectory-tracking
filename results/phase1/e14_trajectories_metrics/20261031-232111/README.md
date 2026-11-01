# phase1 / e14_trajectories_metrics

Step 1.4: payload reference trajectories T0-T3 + Paper 1 ellipse with exact derivatives up to 6th order; D9 metrics; logger. Gates M1-M5.

- Started: 2026-10-31 23:21:11  (wall time 19.117 s)
- Code: commit `399249526cffc680e74c999819f47bc2903fc881`
- Command: `experiments/e14_trajectories_metrics.py`
- MuJoCo 3.17.0, Python 3.10.12

## Metrics

| metric | value |
|---|---|
| M1_derivatives.circle_0.5g_max_rel_err | 8.933327671519156e-14 |
| M1_derivatives.figure8_5s_max_rel_err | 5.184567888893032e-12 |
| M1_derivatives.p2p_1s_max_rel_err | 1.4402748919853771e-11 |
| M1_derivatives.ellipse_T2_max_rel_err | 2.011834563008429e-16 |
| M1_derivatives.passed | True |
| M2_boundaries.p2p_rest_to_rest | True |
| M2_boundaries.circle_0.5g_jump_eps1e-6 | 0.0005142196091583173 |
| M2_boundaries.circle_0.5g_min_shrink_ratio | 10.000002318689116 |
| M2_boundaries.figure8_5s_jump_eps1e-6 | 0.0005124306547656943 |
| M2_boundaries.figure8_5s_min_shrink_ratio | 9.999998243689342 |
| M2_boundaries.passed | True |
| M3_aggressiveness.circle_0.25g_rel_err | 0.0 |
| M3_aggressiveness.circle_0.25g_max_over_peak | 1.0000000000017941 |
| M3_aggressiveness.circle_0.5g_rel_err | 0.0 |
| M3_aggressiveness.circle_0.5g_max_over_peak | 1.0000000000017943 |
| M3_aggressiveness.circle_1.0g_rel_err | 0.0 |
| M3_aggressiveness.circle_1.0g_max_over_peak | 1.0000000000017941 |
| M3_aggressiveness.figure8_15.0s_periodicity_err | 1.1102230246251565e-15 |
| M3_aggressiveness.figure8_10.0s_periodicity_err | 2.7755575615628914e-16 |
| M3_aggressiveness.figure8_5.0s_periodicity_err | 2.449293598294707e-16 |
| M3_aggressiveness.p2p_4.0s_peak_acc_g | 0.08394945058223971 |
| M3_aggressiveness.p2p_2.0s_peak_acc_g | 0.3357978023290451 |
| M3_aggressiveness.p2p_1.0s_peak_acc_g | 1.3431912093161804 |
| M3_aggressiveness.passed | True |
| M4_metrics.rmse | True |
| M4_metrics.swing_max | True |
| M4_metrics.swing_rms | True |
| M4_metrics.effort | True |
| M4_metrics.saturation | True |
| M4_metrics.success | True |
| M4_metrics.settling | True |
| M4_metrics.payload_above_fails | True |
| M4_metrics.settling_time_s | 1.3900000000000001 |
| M4_metrics.passed | True |
| M5_logger.n_fields | 16 |
| M5_logger.n_samples | 100 |
| M5_logger.identical | True |
| M5_logger.passed | True |
| all_passed | True |

## Figures

- trajectories.png

## Notes

Gate passes only if all_passed is true. M1 compares against an independent symbolic (sympy) differentiation; M2 is a convergence test for smoothness at the ramp join.
