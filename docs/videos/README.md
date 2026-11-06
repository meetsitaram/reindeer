# Motion clips

Rendered by `tools/render_motion.py` from `spec/robot.json`, at 720 x 720 and 30 fps. Each clip is a **dynamic
MuJoCo simulation**, not keyframed puppetry:

- The body is free on the floor.
- Every servo is a position servo limited to the XL330's stall torque (0.52 N m), as in `tools/sim_stand.py`.
- Joints follow eased targets, clipped to the spec's limits.
- The camera follows the body from the front right.

To render again:

```bash
python tools/render_motion.py --frames
```

| clip | what it shows | source | numbers from its run |
|---|---|---|---|
| `stand_up.mp4` (6.4 s) | Lying with all four legs folded under it, it stands up: rear legs first, then front, then a look up | keyframes in `STAND_UP`, simulated | tilt at most 2.7°, no drift |
| `sit.mp4` (8.8 s, from the side) | It sits back: the rear legs fold into a kneel, front legs straight. It looks round with a head tilt, then stands again | `SIT_DOWN`, simulated | tilt at most 9.2° (the sit tips the body back), drift 6 mm |
| `walk.mp4` (9.8 s) | A trot, diagonal pairs together | `tools/gait.py`, `sim/gait_params.json` `video` | 0.20 m/s, 1.77 m in 9.8 s, 7 cm side drift, 0.07 rad yaw, tilt at most 8.3° |
| `motions.mp4` (14 s) | Head tilt each way, a nod, a paw lifted; then a close-up as the jaw opens twice and the antlers tilt fore and aft | `MOVES`, simulated | tilt at most 0.7° |

## What to know before trusting them

- **The walk is not a trained policy.** It's a parametric trot, sines on each hip and knee, tuned by a 20-minute
  CPU search (CMA-ES, 4,560 runs of 5 s at a 2 ms step).
  - The search's best ran at 0.39 m/s, but it held a servo at stall 91 % of the time and fell after 9.7 s at the render's 1 ms step.
  - The clip uses 0.6 × its amplitudes, chosen from 16 checks. It still holds some servo at stall 36 % of the time, at 3 Hz: a real XL330 would heat up doing that.
  - A real walk needs the policy in `sim/TRAINING.md`, trained off this laptop.
- **The servos are stiffer than the real ones.** kp is 6 N m/rad, an estimate. microduck's identified XL330 is 0.55, and a softer servo will walk worse until it's trained for.
- **The sit is a kneel, not a sit.** The hips' ±0.5 rad don't let the rump reach the floor.
- **Contact:** only the body, thighs and shins touch the floor. Decals (eyes, hooves' colour) are drawn but don't collide.
