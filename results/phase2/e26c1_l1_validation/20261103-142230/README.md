# phase2 / e26c1_l1_validation

Step 2.6c-1: L1 realism + CTBR validation, gates R1-R8.

- Started: 2026-11-03 14:22:31  (wall time 47.967 s)
- Code: commit `523d97dc8191c7de094166f4fba5421cd4f91045`  **(uncommitted changes present)**
- Command: `experiments/e26c1_l1_validation.py`
- MuJoCo 3.17.0, Python 3.10.12

## Metrics

| metric | value |
|---|---|
| R1_noise_statistics.x_Q_std_rel_err | 0.008497125617691115 |
| R1_noise_statistics.v_Q_std_rel_err | 0.0036294405791232887 |
| R1_noise_statistics.att_std_rel_err | 0.007552251339167371 |
| R1_noise_statistics.Omega_std_rel_err | 0.005511763431395256 |
| R1_noise_statistics.x_L_std_rel_err | 0.007991394236423255 |
| R1_noise_statistics.v_L_std_rel_err | 0.010271514092968559 |
| R1_noise_statistics.q_angle_noise_rms_deg | 2.2887729679672826 |
| R1_noise_statistics.passed | True |
| R2_delay.n_ticks_100Hz | 1 |
| R2_delay.n_ticks_500Hz | 5 |
| R2_delay.non_multiple_rejected | True |
| R2_delay.passed | True |
| R3_ctbr_equals_paper1.err_Omega_c_zero_Nm | 7.105427357601002e-15 |
| R3_ctbr_equals_paper1.err_with_ff_term_Nm | 1.4210854715202004e-14 |
| R3_ctbr_equals_paper1.passed | True |
| R4_runner_reproduces_2.5.max_pos_diff_m | 0.0 |
| R4_runner_reproduces_2.5.passed | True |
| R5_ctbr_tracking.CTBM_500_max_err_m | 0.00016895071257584147 |
| R5_ctbr_tracking.CTBM_500_rmse_m | 6.322790296139917e-05 |
| R5_ctbr_tracking.CTBR_500_max_err_m | 0.00299322291338041 |
| R5_ctbr_tracking.CTBR_500_rmse_m | 0.0023005111147294754 |
| R5_ctbr_tracking.CTBR_100_max_err_m | 0.0036422811400247204 |
| R5_ctbr_tracking.CTBR_100_rmse_m | 0.002845413962028681 |
| R5_ctbr_tracking.passed | True |
| R6_rate_step.time_constant_s | 0.017520710386252258 |
| R6_rate_step.expected_s | 0.018518518518518517 |
| R6_rate_step.rel_err | 0.05388163914237803 |
| R6_rate_step.passed | True |
| R7_drag.drone_drag_x_at_2ms_N | -0.10102194805031905 |
| R7_drag.drone_drag_x_at_4ms_N | -0.4038351300394989 |
| R7_drag.drone_quadratic_ratio | 3.997498937937215 |
| R7_drag.payload_drag_x_at_2ms_N | -0.004455241767706733 |
| R7_drag.payload_drag_x_at_4ms_N | -0.017773466189904653 |
| R7_drag.payload_quadratic_ratio | 3.989338203536656 |
| R7_drag.payload_sphere_CD047_at_4ms_N | 0.01772596530898237 |
| R7_drag.payload_vs_sphere | 1.0026797345077851 |
| R7_drag.passed | True |
| R8_seeds.same_seed_diff_m | 0.0 |
| R8_seeds.other_seed_diff_m | 0.045473767998278625 |
| R8_seeds.passed | True |
| all_passed | True |

## Figures


## Notes

Gate passes only if all_passed is true.
