# Testing droid-from-spec on something that isn't a duck

The reindeer was designed by following the `droid-from-spec` skill (github.com/tinkerdroids/design-and-build-skills)
phase by phase, starting 2026-11-05 22:38, with the user away. Each entry is one instruction or step the run used:

- **W**: it worked as written.
- **A**: it needed a workaround.
- **M**: it was missing or wrong.

Each entry says what failed, the fix, and the change made to the skill. Commits in the skills repo are named in the
last column.

The reference changed mid-run. The user's first video (`reindeer.mp4`) is a turntable of a sitting plush, so rounds
1-4 designed a sitting desk droid. At 23:01 the user sent `reindeer_walking.mp4`, the same plush standing and walking
on four legs, and rounds 5-7 redesigned it as a walking quadruped. The sitting design is described here; the spec
holds the quadruped.

## Score

| | entries | share |
|---|---|---|
| worked as written (W) | 21 | **51 %** |
| needed a workaround (A) | 12 | 29 % |
| missing or wrong (M) | 8 | 20 % |
| total used | 41 | |

Not used this run: the green-background gap render, the phase-3 detail pass (screws, wiring, connectors), STEP export,
phases 5 and 6, agentic-team. Those weren't reached in the time, so they aren't scored.

The changes to the skill are listed at the end, with their size.

## Entries

### SKILL.md

| # | instruction | | what happened | fix / change to the skill |
|---|---|---|---|---|
| 1 | Find the phase: LESSONS.md, last reports, `gate.py --status` | W | "no gate run yet here": phase 1 | |
| 2 | Start a new droid by forking the nearest one (duck-droids) | A | duck-droids' `spec/robot.py` is 1,600 lines of a biped with five servo recipes, and its Fusion `build.py` (826 lines) is tied to `parts.json`. Only `geom.py`, `checks.py`, `mass_model.py`, `render_pose.py` and `figlib.py` could be forked; the spec, compare, sim and Fusion builder were written new | each-round.md "Make it yours" now lists which modules are generic and which are a droid's own |
| 3 | Read the lessons of the repo it came from | W | duck-droids' LESSONS.md and the skill's matching-a-reference file | |
| 4 | "Python 3.12 with numpy, scipy, ..." | A | `python` isn't on this PC's PATH (the Store alias answers). The interpreter is `~/.venvs/duck`, found in duck-droids' `tools/pipeline.sh` | SKILL.md: find the project's interpreter first (its pipeline script or venv) |
| 5 | Work on your own; ask only for the look, money, public steps | M | the user said mid-run "put a timeout on your questions, go with your recommendations when I am not around". The skill has gates that need the user (the look, the target) and nothing for when they're away | SKILL.md: when the user is away, take the recommended option, log it, carry on; gates the user owns are marked "provisional" |
| 6 | Rule 4: judge the look in pictures, side by side | W | the first comparison showed what no check did: a chest stopping short of the chin with a gap of dark neck, chest pockets cut open, dark brackets outside the chest, spiky antlers. The quadruped's first picture showed blocky legs and too-tall antlers | |
| 7 | Rules 6-8: copy the reference's joints, measure its meshes, validate poses | M | the reference is a plush toy in two phone videos: no model, no joints, no poses. The rules assume a robot | phases/1-concept.md: a reference without a model (a toy, an animal, a drawing): choose joints for the moves wanted, hide them in the silhouette, measure fractions from frames |
| 8 | Rule 3: prove every check on a broken design | W | 9 cases. It found three weak cases: `tip` (120 g moved 5 cm) stayed inside the hooves; `poke` went blind once the Pi was marked exposed; `cover` aimed at a body that leg pockets had opened. All three were rewritten and now fail | checks.md: marking a part exposed removes it from the skin check, so a `--bad` case must use a part that isn't |

### Phase 1: concept

| # | instruction | | what happened | fix / change to the skill |
|---|---|---|---|---|
| 9 | Write the goal, a weight and part-count goal | W | 450 g sitting, then 650 g and 11 servos standing; bottom-up 630 g | |
| 10 | Pin the target: frames from the maker's page in a browser | A | the reference was a local mp4 with no page. No ffmpeg on PATH; the one bundled with `imageio-ffmpeg` worked (`tools/reference_frames.py`: frames at exact times and a contact sheet) | reference-images.md: a local video section |
| 11 | `spec/targets.json`: each landmark with its frame, time and tolerance | W | 10 fractions of the height, each with its frame | |
| 12 | Measure heights in level views only | W | the walking video's front frame has a low, close camera: eyes 0.63 H there, 0.51 H from the side. The side's was used | |
| 13 | Get the maker's CAD for every bought part | A | XL330 numbers came from duck-droids' `actuators.json`. ROBOTIS's STEP is local but wasn't put into the Fusion model in time: the servos are boxes there | open item |
| 14 | Ask what the user already owns | M | the user was away, and nothing says what to do then | covered by entry 5 |
| 15 | Borrow a proven layout from an open-source robot (Asimov 1, Unitree G1/H2, AgiBot X2) | M | all humanoids. For a 24 cm quadruped the nearest proven layout is Petoi's Bittle and Nybble (2 DOF a leg, a servo at hip and knee), which the reindeer follows | phases/1-concept.md: examples for quadrupeds and small animals |
| 16 | Hold every joint on both sides | W | it shaped the legs: each servo rides in its segment, its horn on the link above, its rear horn on a plate or boss opposite | |
| 17 | Gate 1: the user agrees the target | A | user away: recorded as provisional, with the video frames as the argument | entry 5's rule |

### Phase 2: sim prototype

| # | instruction | | what happened | fix / change to the skill |
|---|---|---|---|---|
| 18 | "Can it stand and walk?" | A | a sitting toy doesn't walk. The phase was run as "does it hold still through its moves" (`sim_sit`, round 1), then as written once it was a quadruped | phases/2: ask the question the droid's moves pose: stand and walk, or sit and gesture |
| 19 | Anchors at the top of `spec/robot.py` | W | `H, ZH, ZK, XF, XR, NX, ZNP...`: moving the hips moved everything | |
| 20 | Write the left side once, mirror the right | A | two things the mirror didn't cover: per-side colours (the plush alternates red and green hooves and antlers: `color_R`), and joint interface names left as `L_` on the right, which failed two static checks | spec-patterns.md: per-side colours; mirror every name a joint carries |
| 21 | A comment with the reason by each number | W | rounds 1-7 are in the spec's comments | |
| 22 | Run the checks until 0 failing | W | sitting: 41, 18, 6, 3, 0. Quadruped: 28, 8, 2, 0. Each failure named its part | |
| 23 | checks.md "Make them yours": `measure()`, `POSES`, mass target | A | the statics are a biped's (`L_foot`, `R_foot`, ankle balancing). They were rewritten generically: lower the body until its lowest point meets the floor; whatever is within 2 mm carries the weight, shared between the hooves down | checks.md: a support that doesn't assume feet |
| 24 | `compare.py` beside the reference | A | duck-droids' compare renders microduck's model. A new one puts a video frame beside the render at one height, with the target lines across both | |
| 25 | `render_pose.py --views` | W | forked as is, with a front-right 3/4 view added | |
| 26 | `sim_stand.py`: stand, squat to 90° knees | A | written new: MuJoCo from the spec's meshes, masses from materials, servos at stall. The quadruped stands, crouches, lifts a paw and tilts its head (0.6° tilt, 0.3 mm drift, 50 % of stall), and a 150 g lump in front tips it (89°). Measuring "slide" as body travel first failed a crouch, which moves the body over planted hooves: it's now drift | |
| 27 | Sim results sanity-checked: the floor carries the weight | W | this rule found a real bug: the support polygon took points under 1.5 mm, the body's bottom sat at exactly 1.5 mm, and only the hooves carried the sitting droid. The CoM looked 10 mm from tipping when it had 17 mm | checks.md: count rigid parts in the support, with a threshold above the shell's own bottom |
| 28 | Motion: retarget a human capture | M | nothing for an animal. Walking needs a gait or a trained policy; per the user's rule, training is prepared elsewhere (not started) | phases/2: for animals, a gait generator or RL, trained off this PC |
| 29 | `gate.py 2` | W | PASS (`robot.py`, `checks.py`, `sim_stand.py`): the default list fitted once the sim was named `sim_stand.py` | |

### Phases 3-4 (as far as reached)

| # | instruction | | what happened | fix / change to the skill |
|---|---|---|---|---|
| 30 | Each servo held at both ends | W | eight leg joints, each with a horn and a rear horn | |
| 31 | fusion-mcp.md: connect, call the build module in short calls | W | the MCP was up; the whole model, 12 links and 11 joints, in 15 s in Direct mode | |
| 32 | "With no timeline, `features.add()` returns None: name nothing from it" | A | the skill's own `figlib.py` broke this rule in `ellipsoid()` (it names the scale feature), and duck-droids never built an ellipsoid in Direct mode. Fixed in reindeer's copy | fusion-mcp.md: figlib's ellipsoid; check every `.add(...)` result |
| 33 | `analyzeInterference`; explain every clash | W | 162 pairs. One was real: the white bib on the chest sat over both front thighs and their servos (4.7 cm³). The offline checks leave decals out of every collision test. Now there's a static check (decal over another link) with its `--bad decal` case; the rest are explained (covers drawn solid until hollowing; horns 0.25 mm into their cases) | checks.md: decals collide too |
| 34 | Save to the hub, one document open, close it | W | `my-projects / reindeer / reindeer`, version 1, closed | |
| 35 | Look at it from every side | W | Fusion: front, side, back, top, 3/4, and a pose | |
| 36 | each-round.md: change, checks, pictures, compare | W | followed every round | |
| 37 | Never publish the reference's frames | A | followed: frames in `%TEMP%/figrefs/reindeer`, `reports/compare/` git-ignored. But the reference is a branded character toy, which the skill doesn't mention (it speaks of a maker's frames) | phases/1: a branded character is a reference for an original design, never a replica: no tag, no name, no copied shapes |
| 38 | Model servos as boxes, "real shape" later | W | at phase 2 boxes were enough; the makers' models are an open item | |

### Things the tools didn't cover, found on this robot

| # | | what happened | fix |
|---|---|---|---|
| 39 | M | A shell that is structure: the thighs hold their servos and the body's side wall carries each hip's outer boss. The connect check counts only rigid parts | `load_bearing` on a shell (checks.py) |
| 40 | M | Shells that print as one (the haunches on the body) were counted as separate whole walls: 32 g too heavy | `merge_into` in mass_model.py |
| 41 | M | The sweep check compared a joint's interface with `!=`, so an interface naming three parts (with the rear horn) matched nothing, and every joint collided "at 0" | `<=` (also in duck-droids' copy: a follow-up there) |

## What the reindeer is now

- A walking quadruped, 24 cm to the antler tips: 11 ROBOTIS XL330-M288 servos (hip and knee a leg, neck pitch, head yaw, head roll), a Pi Zero 2 W, about 630 g bottom-up.
- `python -I tools/checks.py`: 108 checks, 0 failing, 2 warnings. The belly sits at 0.24 H against the plush's 0.19 (two servos a leg); a trot has only two hooves down, so its balance is dynamic.
- All 10 broken designs fail as they should.
- `gate.py 2` passes.
- The Fusion model is in the hub.
- Not done: the phase-3 detail pass (screws, wiring, connectors, a belly cover), hollow printed parts and print files, the makers' servo models in Fusion, STEP export, a gait or trained walk. Nothing ordered.
