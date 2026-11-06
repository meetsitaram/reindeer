# reindeer

A small walking reindeer droid, inspired by a plush reindeer in the user's two videos. It's designed from one Python
spec with the `droid-from-spec` skill (github.com/tinkerdroids/design-and-build-skills), and it's that skill's first
test on something that isn't a duck: see [SKILL_TEST.md](SKILL_TEST.md).

- 24 cm to the antler tips, about 630 g bottom-up.
- 11 ROBOTIS XL330-M288 servos on one bus: hip and knee on each leg, neck pitch, head yaw, head roll.
- A Raspberry Pi Zero 2 W, a 1S pack and a bus board in the body.
- Moves: walking (to train), crouching, lifting a paw, nodding, looking round, and a head tilt.

**Phase 2 of 6 (sim prototype).** Nothing has been printed or bought. The walk is untested: it needs a gait or a
policy trained off this PC.

## Run

```bash
python -I spec/robot.py
```

```bash
python -I tools/checks.py
```

```bash
python tools/sim_stand.py
```

```bash
python tools/render_pose.py
```

`tools/compare.py` needs the reference frames, which stay local (`python -I tools/reference_frames.py`, written to
`%TEMP%/figrefs/reindeer`).

Fusion: `cad/fusion/build.py` through Fusion's MCP server (`new_document`, `link` for each link, `parametric`,
`joints`). The model is saved in the hub at `my-projects / reindeer`.

## Credits

- The tools are forked from github.com/meetsitaram/duck-droids (Apache-2.0).
- XL330 numbers come from ROBOTIS's e-manual, as collected there.
- The look is inspired by a plush toy: no brand, tag or geometry of it is used.
