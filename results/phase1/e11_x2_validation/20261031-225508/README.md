# phase1 / e11_x2_validation

Step 1.1: Skydio X2 (MuJoCo Menagerie) loaded with our settings; gates G1-G5.

- Started: 2026-10-31 22:55:08  (wall time 1.018 s)
- Code: commit `124e5e8c5b095d27e5083b88690518f1ae27bc55`  **(uncommitted changes present)**
- Command: `experiments/e11_x2_validation.py`
- MuJoCo 3.17.0, Python 3.10.12

## Metrics

| metric | value |
|---|---|
| G1_properties.mass_kg | 1.325 |
| G1_properties.mass_err | 0.0 |
| G1_properties.inertia_max_err | 2e-06 |
| G1_properties.com_max_err_m | 3.77358e-05 |
| G1_properties.passed | True |
| G2_hover.hover_thrust_per_rotor_N | 3.24956 |
| G2_hover.max_drift_m | 0.0 |
| G2_hover.max_rotation_deg | 0.0 |
| G2_hover.passed | True |
| G3_free_fall.max_abs_err_m | 3.68594e-14 |
| G3_free_fall.passed | True |
| G4_drag.vz_end_no_drag | -19.62 |
| G4_drag.vz_end_with_drag | -8.74557 |
| G4_drag.drag_force_at_5mps_N | 4.09317 |
| G4_drag.passed | True |
| G5_timestep.pos_diff_m | 3.40693e-10 |
| G5_timestep.passed | True |
| all_passed | True |

## Figures

- gates.png

## Notes

Gate passes only if all_passed is true.
