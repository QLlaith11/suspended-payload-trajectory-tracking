# phase2 / e21_flatness

Step 2.1: Paper 1 flatness map x_L, psi -> q, T, x_Q, R, Omega, Omega_dot, f, M with exact derivatives (jet arithmetic). Gates F1-F6.

- Started: 2026-11-01 11:12:17  (wall time 25.171 s)
- Code: commit `2672d9b2eab252d7864d17190ff33bf4efb0f321`  **(uncommitted changes present)**
- Command: `experiments/e21_flatness.py --udaan-src /tmp/claude-1000/-home-haidar-Trajectory-Tracking-of-a-Suspended-Payload/2d93e21c-1469-42a9-beb1-612150680c24/scratchpad/udaan_src`
- MuJoCo 3.17.0, Python 3.10.12

## Metrics

| metric | value |
|---|---|
| F1_hover.q_err | 0.0 |
| F1_hover.f_err | 0.0 |
| F1_hover.R_err | 0.0 |
| F1_hover.Omega_err | 0.0 |
| F1_hover.M_err | 0.0 |
| F1_hover.T_err | 0.0 |
| F1_hover.x_Q_err | 0.0 |
| F1_hover.passed | True |
| F2_equations_of_motion.max_residual | 1.0658141036401503e-14 |
| F2_equations_of_motion.passed | True |
| F3_consistency.q_dot_err | 5.053802274446184e-07 |
| F3_consistency.q_dot_ratio | 4.008561349171135 |
| F3_consistency.v_Q_err | 3.413672950980384e-07 |
| F3_consistency.v_Q_ratio | 4.00511174192129 |
| F3_consistency.Omega_err | 3.7732340354423854e-06 |
| F3_consistency.Omega_ratio | 3.9262353885204027 |
| F3_consistency.Omega_dot_err | 4.1087512599347065e-05 |
| F3_consistency.Omega_dot_ratio | 3.999997854236244 |
| F3_consistency.passed | True |
| F4_open_loop_replay.circle_0.25g_init_err | 3.3306690738754696e-16 |
| F4_open_loop_replay.circle_0.25g_err_1s_m | 9.384465319834314e-05 |
| F4_open_loop_replay.circle_0.25g_within_1cm_s | 2.941000000000982 |
| F4_open_loop_replay.ellipse_T10_init_err | 1.1102230246251565e-16 |
| F4_open_loop_replay.ellipse_T10_err_1s_m | 1.2014346532014362e-06 |
| F4_open_loop_replay.ellipse_T10_within_1cm_s | 6.0 |
| F4_open_loop_replay.p2p_2s_init_err | 0.0 |
| F4_open_loop_replay.p2p_2s_err_1s_m | 1.5757901668155972e-15 |
| F4_open_loop_replay.p2p_2s_within_1cm_s | 6.0 |
| F4_open_loop_replay.passed | True |
| F6_singularity.raises_on_free_fall | True |
| F6_singularity.passed | True |
| F5_udaan.skipped | False |
| F5_udaan.max_abs_diff | 1.1368683772161603e-13 |
| F5_udaan.passed | True |
| F5_udaan.udaan_commit | 8a3914d62359f3a9750e6081251be64c48e51660 |
| all_passed | True |

## Figures

- flatness.png

## Notes

Gate passes only if all_passed is true. F5 compares with Udaan (BSD-3, commit 8a3914d) loaded from source in a temporary folder, not installed and not copied into the project.
