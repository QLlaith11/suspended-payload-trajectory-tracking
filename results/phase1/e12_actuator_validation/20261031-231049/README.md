# phase1 / e12_actuator_validation

Step 1.2: CTBM -> mixer (PX4-style priorities) -> 0-13 N -> motor lag (MuJoCo FILTER actuator); gates A1-A6.

- Started: 2026-10-31 23:10:49  (wall time 3.392 s)
- Code: commit `6abc0941ca4e36d96e1e5edd46946d151728aa38`  **(uncommitted changes present)**
- Command: `experiments/e12_actuator_validation.py`
- MuJoCo 3.17.0, Python 3.10.12

## Metrics

| metric | value |
|---|---|
| A1_round_trip.max_err | 7.105427357601002e-15 |
| A1_round_trip.passed | True |
| A2_physics_wrench.max_err | 2.842170943040401e-14 |
| A2_physics_wrench.max_lateral_force | 7.105427357601002e-15 |
| A2_physics_wrench.passed | True |
| A3_rotational_response.max_err_rad_s2 | 1.4210854715202004e-14 |
| A3_rotational_response.passed | True |
| A4_saturation.a_thrust_too_high | True |
| A4_saturation.b_yaw_reduced_first | True |
| A4_saturation.c_roll_scaled | True |
| A4_saturation.d_feasible_unchanged | True |
| A4_saturation.bounds_respected | True |
| A4_saturation.passed | True |
| A5_motor_lag.max_err_N | 6.276495767565393e-09 |
| A5_motor_lag.fraction_at_tau | 0.6321205567363926 |
| A5_motor_lag.ideal_instant | True |
| A5_motor_lag.passed | True |
| A6_timestep.pos_diff_m | 6.72035618381981e-09 |
| A6_timestep.passed | True |
| all_passed | True |

## Figures

- actuators.png

## Notes

Gate passes only if all_passed is true. Motor lag uses MuJoCo dyntype FILTER (FILTEREXACT is inaccurate under RK4).
