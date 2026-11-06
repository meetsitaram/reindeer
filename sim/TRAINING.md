# Training the reindeer's walk (on another machine)

**Not run.** This is prepared to run elsewhere: the user's rule is that this laptop doesn't train. What runs here is a
parametric trot tuned by a 20-minute CPU search (`tools/gait.py`, `sim/gait_params.json`). It's a stand-in for the
videos, not a controller for the real robot.

## Framework

Use **mjlab** (MuJoCo-Warp, rsl_rl PPO), the same stack as duck-droids' trainer (`../duck-droids/trainer/`: Pollen's
`microduck_rl` with `mjlab_microduck`, launched by `train_dynamixel.sh`). For a quadruped, start from mjlab's own
Unitree Go1 flat velocity task (`Mjlab-Velocity-Flat-Unitree-Go1`) rather than the duck's biped task. Its rewards,
terminations and curriculum fit four legs. Swap the robot for the reindeer's MJCF.

**MuJoCo Playground** (`mujoco_playground`, JAX/MJX, Go1 joystick) is an alternative. Its domain-randomisation recipe
is the reference either way.

Hardware: one GPU with 16 GB or more, 4,096 parallel environments, about 1-2 hours to a usable walk at the Go1
settings.

## The robot file

Write it from the spec, as duck-droids' `tools/make_rl_model.py` does. The reindeer has no `make_rl_model.py` yet:
it's the first thing to write, about 150 lines, from `tools/render_motion.py`'s `build()`. It writes `reindeer.xml`,
one mesh per link, and `home.json`:

- **Collision:** the hooves only, as spheres or boxes at the shins' feet. Also the body's hull, for the termination check.
- **Actuators:** XL330-M288 position servos, kp 0.55 N m/rad (microduck's identified gain, not the 6 in `sim_stand.py`), force limit 0.52 N m (stall at 5 V). Use duck-droids' DC-motor model: a torque-speed line through 0.52 N m at 0 and 0 at 10.8 rad/s, and a 10 ms delay.
- **Joint dynamics** from `spec/actuators.json`: armature 0.0018 reflected (check the units against microduck's file), damping 0.053, friction 0.0048.
- **Home:** the zero pose (standing, straight legs), or a slight crouch, which the tuned trot prefers (`sim/gait_params.json`, `crouch`).
- **Locked:** the jaw and the antlers. The head joints are either locked, or policy actions with small action rates.

## Spaces

- **Observation**, 45 values and the history of the last 3 steps:
  - base angular velocity (gyro), 3
  - projected gravity, 3
  - command (vx, vy, yaw rate), 3
  - the 8 leg joints' positions relative to home, 8
  - their velocities, 8
  - the last action, 8
  - gait phase, sin and cos, 2
  - head joints, if they're actions
- **Action:** 8 leg joint position offsets about home, scaled by 0.25 rad. Clip them to the spec's limits: hips ±0.5, knees ±1.3.

## Rewards

The Go1 task's, with these weights to start:

| term | weight | note |
|---|---|---|
| track linear velocity (xy), exp(-err²/0.25) | 1.5 | commands vx 0-0.25 m/s, vy ±0.1, yaw ±0.6 rad/s; the 24 cm robot's scale |
| track yaw rate | 0.8 | |
| feet air time (> 0.15 s) | 0.3 | for a trot, not a shuffle |
| base height near 0.11 m | -10 × err² | |
| orientation (projected gravity xy) | -2 | |
| joint torques | -2e-4 | |
| action rate | -0.02 | |
| joint limits | -5 | |
| body or thigh touching the floor | -1, and end the episode | |
| base z velocity, xy angular velocity | -2, -0.05 | |

## Domain randomisation

- **Friction:** 0.4-1.0.
- **Mass:** ±10 % per link, plus up to 50 g added to the body. The bus board and battery are estimates.
- **Servos:**
  - stall torque: 0.8-1.1 × (the XL330's 0.52 N m is at 5 V, and the 1S pack sags)
  - kp: 0.8-1.2 ×
  - delay: 5-20 ms
- **Sensors:** IMU noise and a ±3° mounting tilt.
- **Pushes:** 0.3 m/s every 5-8 s.
- **Terrain:** flat. Add 5 mm roughness after the first stable run.

## Run

```bash
python scripts/train.py Mjlab-Velocity-Flat-Reindeer --env.scene.num-envs 4096 --agent.max-iterations 3000
```

Register the task first: a `reindeer` package beside `mjlab_microduck`, copying the Go1 velocity config with the
robot swapped. Then export to ONNX (`scripts/export.py`) and evaluate in this repo's MuJoCo model before touching
hardware: the same `build()`, the policy in place of `gait.targets()`.

## What to check before calling it a walk

- **Speed:** the commanded speed at 0.25 m/s, with yaw drift under 10°/m. The parametric trot's number is in `sim/gait_params.json`.
- **Torque:** no servo above 80 % of stall for more than 0.2 s. A real XL330 heats fast at stall.
- **Contact:** the hooves only. No knee plate or body touching the floor.
- **The servos:** an identified XL330 model at kp 0.55, not this repo's stiff kp 6, before trusting any result.
