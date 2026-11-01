# phase2 / e22_geometric_control

Step 2.2: Paper 1 geometric controller (quadrotor attitude, load attitude, load position), C1+P1, L0, CTBM at 500 Hz, gains 1.5/7.5/30 rad/s. Gates K1-K7.

- Started: 2026-11-01 11:39:05  (wall time 113.123 s)
- Code: commit `cfda782ba4bbbad21f1337dfff1993068f628011`  **(uncommitted changes present)**
- Command: `experiments/e22_geometric_control.py`
- MuJoCo 3.17.0, Python 3.10.12

## Metrics

| metric | value |
|---|---|
| K1_attitude_mode.eR_final | 1.6941339281442816e-17 |
| K1_attitude_mode.decay_rate | 28.353134345936255 |
| K1_attitude_mode.expected_rate | 27.0 |
| K1_attitude_mode.passed | True |
| K2_load_attitude_mode.Psi_q_final | 0.0 |
| K2_load_attitude_mode.error_decay_rate | 7.631463707367602 |
| K2_load_attitude_mode.expected_rate | 6.75 |
| K2_load_attitude_mode.passed | True |
| K3_hover_recovery.settling_time_s | 2.8659999999997954 |
| K3_hover_recovery.expected_s | 2.8341465755294935 |
| K3_hover_recovery.final_err_m | 0.0009391982513286163 |
| K3_hover_recovery.rel_dev | 0.011239159168876389 |
| K3_hover_recovery.passed | True |
| K4_tracking.T1_2s_max_err_m | 0.00015488356531193211 |
| K4_tracking.T1_2s_rmse_m | 5.76352362856681e-05 |
| K4_tracking.T2_0.5g_max_err_m | 0.00016895071257584147 |
| K4_tracking.T2_0.5g_rmse_m | 5.380522553754004e-05 |
| K4_tracking.T3_10s_max_err_m | 8.186592891470122e-06 |
| K4_tracking.T3_10s_rmse_m | 4.076029108500946e-06 |
| K4_tracking.passed | True |
| K5_time_scale_separation.wR15_rmse_m | 5.8976542534094634e-05 |
| K5_time_scale_separation.wR30_rmse_m | 5.380522553754004e-05 |
| K5_time_scale_separation.wR60_rmse_m | 5.2944694418564744e-05 |
| K5_time_scale_separation.ordering_ok | True |
| K5_time_scale_separation.monotone | True |
| K5_time_scale_separation.passed | True |
| K6_rate_error_guard.tangent_to_S2 | True |
| K6_rate_error_guard.residual_swing_correct_deg | 0.09156894542221204 |
| K6_rate_error_guard.residual_swing_wrong_form_deg | 179.4641730630131 |
| K6_rate_error_guard.passed | True |
| K7_derivative_ablation.flat_rmse_m | 5.380522553754004e-05 |
| K7_derivative_ablation.flat_max_err_m | 0.00016895071257584147 |
| K7_derivative_ablation.numdiff_rmse_m | 0.012530914905365547 |
| K7_derivative_ablation.numdiff_max_err_m | 0.014365784917238689 |
| K7_derivative_ablation.zero_rmse_m | 0.672347785166603 |
| K7_derivative_ablation.zero_max_err_m | 0.7700204836581703 |
| K7_derivative_ablation.passed | True |
| all_passed | True |

## Figures

- geometric_control.png

## Notes

Gate passes only if all_passed is true. K1, K2, K5 run with the ideal actuator (Paper 1's unlimited-input assumption); all others with the X2's 0-13 N rotor limits (finding F3).
