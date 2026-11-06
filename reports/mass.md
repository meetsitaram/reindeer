# Mass model

Bottom-up: every part from its material and shape (`tools/mass_model.py`, materials in `spec/robot.py`), a 3 % wiring allowance per link on its frames.

**Total 0.63 kg** (microduck: 0.8 kg).

| material | kg | share |
|---|---|---|
| pla | 0.35 | 56 % |
| actuator | 0.20 | 33 % |
| pla_skin | 0.03 | 5 % |
| cells | 0.02 | 3 % |
| electronics | 0.02 | 3 % |

| link | mass kg | budget kg (hand estimate) | CoM (m, world, zero pose) | principal inertia (kg m2) |
|---|---|---|---|---|
| body | 0.13 | 0.25 | -0.013 +0.000 +0.084 | 0.0001 0.0002 0.0002 |
| neck | 0.03 | 0.04 | +0.006 -0.000 +0.114 | 0.0000 0.0000 0.0000 |
| yawlink | 0.02 | 0.03 | +0.012 -0.007 +0.148 | 0.0000 0.0000 0.0000 |
| head | 0.17 | 0.14 | +0.034 +0.000 +0.163 | 0.0002 0.0004 0.0004 |
| L_front_thigh | 0.03 | 0.03 | +0.040 +0.018 +0.059 | 0.0000 0.0000 0.0000 |
| L_front_shin | 0.04 | 0.03 | +0.040 +0.017 +0.018 | 0.0000 0.0000 0.0000 |
| L_rear_thigh | 0.03 | 0.03 | -0.040 +0.018 +0.059 | 0.0000 0.0000 0.0000 |
| L_rear_shin | 0.04 | 0.03 | -0.040 +0.017 +0.018 | 0.0000 0.0000 0.0000 |
| R_front_thigh | 0.03 | 0.03 | +0.040 -0.018 +0.059 | 0.0000 0.0000 0.0000 |
| R_front_shin | 0.04 | 0.03 | +0.040 -0.017 +0.018 | 0.0000 0.0000 0.0000 |
| R_rear_thigh | 0.03 | 0.03 | -0.040 -0.018 +0.059 | 0.0000 0.0000 0.0000 |
| R_rear_shin | 0.04 | 0.03 | -0.040 -0.017 +0.018 | 0.0000 0.0000 0.0000 |
