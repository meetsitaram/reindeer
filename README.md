# reindeer

A small walking reindeer droid, inspired by a plush reindeer in the user's two videos. It's designed from one Python
spec with the `droid-from-spec` skill (github.com/tinkerdroids/design-and-build-skills), and it's that skill's first
test on something that isn't a duck: see [SKILL_TEST.md](SKILL_TEST.md).

- 24 cm to the antler tips, about 690 g bottom-up.
- 13 ROBOTIS XL330-M288 servos on one bus: hip and knee on each leg, neck pitch, head yaw, head roll, a jaw, and both antlers together.
- A Raspberry Pi Zero 2 W, a 1S pack and a bus board in the body.
- Moves: walking (a tuned gait in simulation; RL prepared to train elsewhere), crouching, sitting, lifting a paw, nodding, looking round, a head tilt, talking, and an antler nod.

**Phases 2-4 in part.** Nothing has been printed or bought. See `SKILL_TEST.md` for what is done and what is open,
`BOM.md`, `docs/WIRING.md`, `docs/DETAIL_PASS.md`, `prints/` and `docs/videos/`.

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
