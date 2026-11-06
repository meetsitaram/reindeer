# Detail pass (phase 3), joint by joint

This file asks, for each joint, what a person holding the parts would ask. Round 8, 2026-11-06, from `spec/robot.py`.
Statics are the most torque in any of the task poses in `tools/checks.py`. The rated torque, 0.17 N m, is a third of
the XL330's stall (an estimate).

| joint | servo case in | horn side | rear side | plug and lead | statics |
|---|---|---|---|---|---|
| neck_pitch | body: a cradle (floor + back wall) on the keel, 4 frame screws M2 x 8 | neck's horn plate, 4 M2 x 6 | neck's rear plate on the FPX330 idler | lead up to the yaw servo through the neck's U, inside the pitch's swing | 0.078 N m nodding: 46 % of rated |
| head_yaw | neck: on its bridge and seat, 4 frame screws into them | yaw plate, 4 M2 x 6 | **none**: the case's rear face sits on the bridge, screwed through its frame | lead from neck_pitch round the bridge; it twists ±1.0 rad: leave a service loop | 0.001 N m |
| head_roll | yawlink: on the yaw plate, its rear wall on the idler | head bracket on the horn, 4 M2 x 6 | the rear wall, on the idler | lead from yaw to roll inside the head's opening | 0.006 N m |
| jaw | head: a cradle on top, posted to the face post | jaw plate (white) | the right jaw plate on the idler | from the roll servo's 2nd socket, then on to the antlers | 0.001 N m |
| antlers | head: a cradle under it, posted to the head bracket | antler horn plate, then an arm and a bar | **none** (the roll case sweeps where its plate would be): the antlers are about 20 g. **Open item:** test flex on the printed head | the chain's end: one lead | 0.018 N m |
| hips (4) | each thigh holds its own case | the keel, 4 M2 x 6 into the horn | a boss in the body's side wall on the idler: the shell carries it (`load_bearing`) | from the bus board's socket 2, leg to leg along the keel | 0.037 N m trotting: 22 % |
| knees (4) | each shin holds its own case | the thigh's outer plate and its knee cap | the thigh's inner plate on the idler | thigh to shin outside the knee, inside the cap: route not modelled | 0.013 N m |

The MuJoCo stand test (`tools/sim_stand.py`) peaks at 79 % of stall. That is position tracking at kp 6 (0.068 rad of
error), not gravity: the statics above say gravity uses at most 46 % of rated.

## Not done yet

- **Screws** are not in the spec. Each case's 4 frame screws and each horn's 4 screws come in its box. No screw that
  holds printed parts together is designed yet (the head's halves, the body's halves, the bars).
- **Leads:** lengths and routes, and room for the plugs at each socket.
- **The battery:** a 2S pack of at most 44 x 30 x 8 mm, not chosen. The regulator and IMU have no place yet.
- **Strength:** no beam estimate for the body's side wall carrying the hips' bosses; a sharp corner where the boss meets
  the wall. Print one leg and its piece of body side before buying the rest (phase 5's first step).
- **A belly cover** under the Pi, which shows from below through the leg slots.
