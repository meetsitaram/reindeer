"""The reindeer's motion clips, simulated in MuJoCo and rendered to MP4 (720 x 720, 30 fps).

  python tools/render_motion.py [clip ...]     clips: stand_up sit walk motions (default: all) -> docs/videos/<clip>.mp4
  python tools/render_motion.py --frames       also saves first / middle / last frames to reports/motion_frames/

Every clip is a dynamic simulation, not a puppet: the body is free on the floor, every servo is a position servo
limited to the XL330's stall torque (as tools/sim_stand.py), and the joints follow targets eased between keyframes.
The walk is tools/gait.py's tuned trot (sim/gait_params.json). Joints a clip doesn't name stay at 0, so a joint added
to the spec (the jaw, the antlers) joins the clips that name it and stays still in the rest.

The look is tools/render_pose.py's: each part in its spec colour, decals included (visual only), glossy eyes, on a
white floor with a soft shadow. The camera follows the body from the front right, three-quarters. Plain python.
"""
import math
import os
import sys
import tempfile

import imageio
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import mass_model as MM  # noqa: E402
import sim_stand as SS  # noqa: E402

W = H = 720
FPS = 30
OUT = os.path.join(G.ROOT, 'docs', 'videos')
GLOSSY = ('#08090B', '#0A1420', '#1BA05A', '#17542E')


def rgba(hex_color):
    h = hex_color.lstrip('#')
    return [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)] + [1.0]


def build(spec, tmp):
    """sim_stand's physics, plus every part drawn in its colour; decals are visual only."""
    S = C.Solids(spec)
    R = G.Robot(spec)
    inert = MM.link_inertials(spec, S)
    acts = spec['actuators']
    assets, k = [], 0

    def body_xml(ln, indent):
        nonlocal k
        j = R.parent_joint.get(ln)
        frame = np.array(j['origin']) if j else np.zeros(3)
        m, com, I, _ = inert[ln]
        ev, evec = np.linalg.eigh(I)
        if np.linalg.det(evec) < 0:
            evec[:, 0] *= -1
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, evec.flatten())
        geoms = []
        for i, g in enumerate(spec['links'][ln]['geoms']):
            mm = S.geom.get((ln, i))
            if mm is None or g['role'] not in ('shell', 'struct', 'actuator', 'decal') or mm.volume() < 1e-9:
                continue
            mesh = mm.to_mesh()
            v = np.asarray(mesh.vert_properties[:, :3], float) - frame
            SS.stl(os.path.join(tmp, f'm{k}.stl'), v, np.asarray(mesh.tri_verts, int))
            assets.append(f'<mesh name="m{k}" file="m{k}.stl"/>')
            solid = g['role'] != 'decal' and (ln == spec['root'] or 'shin' in ln or 'thigh' in ln)
            mat = 'gloss' if g['color'] in GLOSSY else 'matte'
            col = ' '.join(f'{c:.3f}' for c in rgba(g['color']))
            geoms.append(f'{indent}  <geom type="mesh" mesh="m{k}" mass="0" material="{mat}" rgba="{col}" '
                         f'contype="{1 if solid else 0}" conaffinity="{1 if solid else 0}"/>')
            k += 1
        pj = R.parent_joint.get(j['parent']) if j else None
        pos = frame - (np.array(pj['origin']) if pj else np.zeros(3)) if j else np.zeros(3)
        lines = [f'{indent}<body name="{ln}" pos="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}">']
        if j:
            a = acts[j['act']]
            ax = {'x': '1 0 0', 'y': '0 1 0', 'z': '0 0 1'}[j['axis']]
            lines.append(f'{indent}  <joint name="{j["name"]}" axis="{ax}" range="{j["lower"]} {j["upper"]}" limited="true" '
                         f'armature="0.00009" damping="{a["damping_nms"]}" frictionloss="{a["friction_nm"]}"/>')
        else:
            lines.append(f'{indent}  <freejoint/>')
            # the sun rides with the robot (mode track: a fixed offset, its direction fixed in the world), so its shadow map
            # follows a walk: fixed at the origin, the shadow was gone once it had walked 40 cm
            lines.append(f'{indent}  <light mode="track" directional="true" diffuse=".6 .6 .6" specular=".05 .05 .05" '
                         f'pos="0 0 2" dir="0.05 0.08 -1" castshadow="true"/>')
        c = com - frame
        lines.append(f'{indent}  <inertial pos="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}" quat="{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}" '
                     f'mass="{m:.5f}" diaginertia="{max(ev[0], 1e-8):.3e} {max(ev[1], 1e-8):.3e} {max(ev[2], 1e-8):.3e}"/>')
        lines += geoms
        for jj in R.children.get(ln, []):
            lines += body_xml(jj['child'], indent + '  ')
        lines.append(f'{indent}</body>')
        return lines

    bodies = body_xml(spec['root'], '    ')
    motors = [f'    <position name="{j["name"]}" joint="{j["name"]}" kp="{SS.KP}" forcerange="-{acts[j["act"]]["peak_nm"]} '
              f'{acts[j["act"]]["peak_nm"]}" forcelimited="true"/>' for j in spec['joints']]
    xml = f"""<mujoco model="reindeer"><compiler meshdir="{tmp}" angle="radian"/>
<option timestep="0.001" gravity="0 0 -9.81"/>
<statistic center="0 0 0.1" extent="0.35"/>
<visual><global offwidth="{W}" offheight="{H}"/><quality shadowsize="2048"/><map znear="0.01" shadowclip="1.2" shadowscale="0.6" haze="0"/>
<headlight ambient=".30 .30 .30" diffuse=".25 .25 .25" specular=".05 .05 .05"/></visual>
<default><geom friction="0.6 0.005 0.0001" condim="3"/></default>
<asset><material name="matte" specular=".05" shininess=".1"/><material name="gloss" specular=".7" shininess=".9"/>
<material name="floor" rgba="1 1 1 1" emission=".25" specular="0" shininess="0"/>
{''.join(assets)}</asset>
<worldbody>
<geom name="floor" type="plane" size="40 40 0.1" material="floor" contype="1" conaffinity="1"/>
{chr(10).join(bodies)}
</worldbody><actuator>
{chr(10).join(motors)}
</actuator></mujoco>"""
    p = os.path.join(tmp, 'reindeer.xml')
    open(p, 'w').write(xml)
    return mujoco.MjModel.from_xml_path(p)


def eased(keys, t):
    """Targets at time t: keys is [(t0, {joint: angle}), ...]; each step eased in over its own gap (cosine)."""
    cur = dict(keys[0][1])
    for (t0, q0), (t1, q1) in zip(keys, keys[1:]):
        if t < t1:
            s = 0.5 - 0.5 * math.cos(math.pi * min(1.0, max(0.0, (t - t0) / max(t1 - t0, 1e-6))))
            ks = set(q0) | set(q1)
            return {n: q0.get(n, 0.0) + (q1.get(n, 0.0) - q0.get(n, 0.0)) * s for n in ks}
        cur = dict(q1)
    return cur


# ---------------------------------------------------------------- the clips' keyframes
LEGS = ('L_front', 'R_front', 'L_rear', 'R_rear')
FOLD_F = dict(hip=-0.5, knee=1.3)      # a front leg folded under the chest (lying)
FOLD_R = dict(hip=0.5, knee=-1.3)      # a rear leg folded under the rump


def legs(front=None, rear=None):
    q = {}
    for leg in LEGS:
        v = front if 'front' in leg else rear
        if v:
            q[f'{leg}_hip'], q[f'{leg}_knee'] = v['hip'], v['knee']
    return q


LYING = legs(FOLD_F, FOLD_R)
# the rear legs folded into a kneel (thigh back, shin folded up under it: it rests on its shins), front legs straight:
# the rump drops about 3 cm and the body tips back. The hips' +/-0.5 don't let the rump reach the floor
SIT = dict(legs(dict(hip=-0.15, knee=0.0), dict(hip=0.5, knee=1.3)))
STAND_UP = [(0.0, LYING), (1.2, LYING), (2.4, legs(None, FOLD_R)), (3.6, {}), (4.6, {'neck_pitch': -0.2}), (5.4, {}), (6.0, {})]
SIT_DOWN = [(0.0, {}), (0.8, {}), (2.4, SIT), (3.2, dict(SIT, neck_pitch=-0.15)), (4.0, dict(SIT, head_yaw=0.8, head_roll=0.25)),
            (5.0, dict(SIT, head_yaw=-0.8, head_roll=-0.25)), (5.8, dict(SIT)), (7.6, {}), (8.4, {})]
MOVES = [(0.0, {}), (0.6, {}),
         (1.4, {'head_roll': 0.30, 'head_yaw': 0.4}), (2.4, {'head_roll': 0.30, 'head_yaw': 0.4}),
         (3.2, {'head_roll': -0.30, 'head_yaw': -0.4}), (4.0, {}),
         (4.6, {'neck_pitch': 0.18}), (5.1, {'neck_pitch': -0.25}), (5.6, {'neck_pitch': 0.18}), (6.2, {}),
         (7.0, {'L_front_hip': -0.6, 'L_front_knee': 1.2, 'neck_pitch': -0.2}), (8.0, {'L_front_hip': -0.6, 'L_front_knee': 1.2, 'neck_pitch': -0.2}),
         (8.8, {}),
         (9.4, {'jaw': 0.45, 'neck_pitch': -0.1}), (9.9, {}), (10.3, {'jaw': 0.45, 'neck_pitch': -0.1}), (10.8, {}),
         (11.4, {'antlers': 0.15}), (11.9, {'antlers': -0.15}), (12.4, {'antlers': 0.15}), (12.9, {}), (13.6, {})]


# each clip's camera: (degrees from the front toward the left, elevation, distance m). The sit from the side, where
# the rump's drop shows; the walk a little further out
CAMS = {'sit': (-80.0, -16.0, 0.48), 'walk': (-60.0, -24.0, 0.55)}


def simulate(m, clip, record_dt=1 / FPS, lying_start=False):
    """[(time, qpos)] at the frame rate, and the run's worst tilt and stall share."""
    d = mujoco.MjData(m)
    names = [m.actuator(i).name for i in range(m.nu)]
    act = {n: i for i, n in enumerate(names)}
    jq = {n: m.jnt_qposadr[m.joint(n).id] for n in names}
    if clip == 'walk':
        import gait as GA
        p, dk = GA.load_params()
        T = GA.WARM + 9.0
        tgt = lambda t: GA.targets(p, t, dk)
    else:
        keys = {'stand_up': STAND_UP, 'sit': SIT_DOWN, 'motions': MOVES}[clip]
        keys = [(t, {n: v for n, v in q.items() if n in act}) for t, q in keys]
        T = keys[-1][0] + 0.4
        tgt = lambda t: eased(keys, t)
    if clip == 'stand_up':                           # start folded, set down from just above the floor
        for n, v in LYING.items():
            d.qpos[jq[n]] = v
        mujoco.mj_forward(m, d)
        low = min(d.geom_xpos[g][2] - m.geom_rbound[g] for g in range(m.ngeom) if m.geom_contype[g] and m.geom_bodyid[g] > 0)
        d.qpos[2] -= low - 0.004
    mujoco.mj_forward(m, d)
    frames, nxt, tilt = [], 0.0, 0.0
    up = np.zeros(9)
    while d.time < T:
        q = tgt(d.time)
        for n, i in act.items():
            lo, hi = m.jnt_range[m.joint(n).id]
            d.ctrl[i] = min(max(q.get(n, 0.0), lo), hi)          # inside the spec's limits, whatever the keyframes say
        mujoco.mj_step(m, d)
        if d.time >= nxt:
            frames.append((d.time, d.qpos.copy()))
            nxt += record_dt
        mujoco.mju_quat2Mat(up, d.qpos[3:7])
        tilt = max(tilt, math.degrees(math.acos(max(-1.0, min(1.0, up[8])))))
    return frames, tilt


# close-ups: (from s, to s, distance m, look-at height m, elevation): the motions clip comes in on the head for the jaw and
# the antlers, which are too small to read from 46 cm
ZOOM = {"motions": [(8.9, 13.3, 0.30, 0.13, -14.0, -28.0)]}   # + azimuth: more from the front


def zoomed(clip, t, dist, look_z, el):
    for t0, t1, d1, z1, e1, a1 in ZOOM.get(clip, []):
        if t0 - 0.6 <= t <= t1 + 0.6:
            s = min(1.0, (t - (t0 - 0.6)) / 0.6, ((t1 + 0.6) - t) / 0.6)
            s = 0.5 - 0.5 * math.cos(math.pi * max(0.0, s))
            return dist + (d1 - dist) * s, look_z + (z1 - look_z) * s, el + (e1 - el) * s, s, a1
    return dist, look_z, el, 0.0, 0.0


def render(m, frames, path, view_az=-50.0, el=-27.0, dist=0.46, keep=(), clip=''):
    """view_az: degrees from the robot's front toward its left (-50: front right), as render_pose's VIEWS; el: camera
    below 0 looks down. MuJoCo's own azimuth is the way the camera looks, so it is turned round here."""
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth, cam.elevation, cam.distance = view_az + 180.0, el, dist
    look = None
    shots = {}
    with imageio.get_writer(path, fps=FPS, codec='libx264', quality=8, macro_block_size=8) as w:
        for k, (t, qpos) in enumerate(frames):
            d.qpos[:] = qpos
            mujoco.mj_forward(m, d)
            dz, lz, ez, sz, az1 = zoomed(clip, t, dist, 0.10, el)
            cam.distance, cam.elevation = dz, ez
            cam.azimuth = (view_az + (az1 - view_az) * sz if sz else view_az) + 180.0
            target = np.array([d.qpos[0], d.qpos[1], lz])
            look = target if look is None else np.r_[0.9 * look[:2] + 0.1 * target[:2], target[2]]   # follows smoothly in plan
            cam.lookat[:] = look
            r.update_scene(d, cam)
            img = r.render().copy()
            r.enable_segmentation_rendering()
            r.update_scene(d, cam)
            seg = r.render()[:, :, 0]
            r.disable_segmentation_rendering()
            img[seg < 0] = 255                                       # the sky white, as render_pose's pictures
            w.append_data(img)
            if k in keep:
                shots[k] = img.copy()
    r.close()
    return shots


def main():
    clips = [a for a in sys.argv[1:] if not a.startswith('--')] or ['stand_up', 'sit', 'walk', 'motions']
    spec = G.load()
    with tempfile.TemporaryDirectory() as tmp:
        m = build(spec, tmp)
    os.makedirs(OUT, exist_ok=True)
    fdir = os.path.join(G.ROOT, 'reports', 'motion_frames')
    os.makedirs(fdir, exist_ok=True)
    for clip in clips:
        frames, tilt = simulate(m, clip)
        n = len(frames)
        keep = (0, n // 4, n // 2, 3 * n // 4, n - 1)
        path = os.path.join(OUT, f'{clip}.mp4')
        az, el, dist = CAMS.get(clip, (-50.0, -27.0, 0.46))
        shots = render(m, frames, path, view_az=az, el=el, dist=dist, keep=keep if '--frames' in sys.argv else (), clip=clip)
        x0, x1 = frames[0][1][:3], frames[-1][1][:3]
        print(f'{clip}: {n} frames, {n / FPS:.1f} s, max tilt {tilt:.1f} deg, body moved {np.linalg.norm(x1[:2] - x0[:2]) * 100:.1f} cm '
              f'(x {100 * (x1[0] - x0[0]):+.1f}), end height {x1[2] * 1000:.0f} mm -> {path}', flush=True)
        if shots:
            from PIL import Image
            row = Image.new('RGB', (len(shots) * 360, 360), 'white')
            for i, k in enumerate(sorted(shots)):
                row.paste(Image.fromarray(shots[k]).resize((360, 360)), (i * 360, 0))
            row.save(os.path.join(fdir, f'{clip}.png'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
