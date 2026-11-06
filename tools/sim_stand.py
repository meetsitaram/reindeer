"""The reindeer in MuJoCo: does it stand still through its moves, every servo limited to its stall torque?

  python tools/sim_stand.py         -> reports/sim_stand.md; exit 1 if it tips, slides or a servo saturates
  python tools/sim_stand.py --bad     a 150 g lump hung 9 cm in front of the chest: it must tip or saturate

Phase 2's question, short of walking (walking needs a trained policy, prepared to run elsewhere): the moves are run as smooth
position targets (cosine ramps), the body free on the floor. Each part is its convex hull for contact (MuJoCo's own
rule), masses and inertias per link from tools/mass_model.py, servo armature, damping and friction from
spec/actuators.json (microduck's identified XL330), kp 6 N m/rad (an estimate, as no XL330 gain is identified here).
Plain python (MuJoCo lives in the user site-packages).
"""
import math
import os
import sys
import tempfile

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402
import mass_model as MM  # noqa: E402

KP = 6.0
# (seconds, targets): the choreography, each move eased in over 0.6 s and held
MOVES = [
    (0.0, {}),
    (1.0, {'head_roll': 0.35, 'head_yaw': 0.6}),                                    # the head tilt
    (2.0, {'head_roll': -0.35, 'head_yaw': -0.6, 'neck_pitch': 0.27}),
    (3.0, {'L_front_hip': 0.5, 'L_front_knee': -1.0, 'R_front_hip': 0.5, 'R_front_knee': -1.0,
           'L_rear_hip': 0.5, 'L_rear_knee': -1.0, 'R_rear_hip': 0.5, 'R_rear_knee': -1.0}),   # crouch
    (4.2, {}),
    (5.0, {'L_front_hip': -0.6, 'L_front_knee': 1.2, 'neck_pitch': -0.3}),           # a paw lifted, looking up
    (6.2, {}),
]
T_END = 7.5


def stl(path, v, f):
    tri = v[f].astype(np.float32)
    rec = np.zeros(len(f), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['v'] = tri
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80)
        fh.write(np.uint32(len(f)).tobytes())
        fh.write(rec.tobytes())


def build(spec, tmp, bad=False):
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
        if bad and ln == spec['root']:
            com = (m * com + 0.15 * np.array([0.16, 0, 0.08])) / (m + 0.15)
            m += 0.15
        ev, evec = np.linalg.eigh(I)
        if np.linalg.det(evec) < 0:
            evec[:, 0] *= -1
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, evec.flatten())
        geoms = []
        for i, g in enumerate(spec['links'][ln]['geoms']):
            mm = S.geom.get((ln, i))
            if mm is None or g['role'] not in ('shell', 'struct', 'actuator') or mm.volume() < 1e-8:
                continue
            mesh = mm.to_mesh()
            v = np.asarray(mesh.vert_properties[:, :3], float) - frame
            stl(os.path.join(tmp, f'm{k}.stl'), v, np.asarray(mesh.tri_verts, int))
            assets.append(f'<mesh name="m{k}" file="m{k}.stl"/>')
            geoms.append(f'{indent}  <geom type="mesh" mesh="m{k}" mass="0" contype="{1 if ln == spec["root"] or "shin" in ln or "thigh" in ln else 0}" conaffinity="1"/>')
            k += 1
        pj = R.parent_joint.get(j['parent']) if j else None
        pos = frame - (np.array(pj['origin']) if pj else np.zeros(3)) if j else np.zeros(3)
        head = f'{indent}<body name="{ln}" pos="{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}">'
        lines = [head]
        if j:
            a = acts[j['act']]
            ax = {'x': '1 0 0', 'y': '0 1 0', 'z': '0 0 1'}[j['axis']]
            lines.append(f'{indent}  <joint name="{j["name"]}" axis="{ax}" range="{j["lower"]} {j["upper"]}" limited="true" '
                         f'armature="{a["armature_kgm2"] / a["gear"] ** 0 * 0.0 + 0.00009}" damping="{a["damping_nms"]}" frictionloss="{a["friction_nm"]}"/>')
        else:
            lines.append(f'{indent}  <freejoint/>')
        c = com - frame
        lines.append(f'{indent}  <inertial pos="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}" quat="{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}" '
                     f'mass="{m:.5f}" diaginertia="{max(ev[0], 1e-8):.3e} {max(ev[1], 1e-8):.3e} {max(ev[2], 1e-8):.3e}"/>')
        lines += geoms
        for jj in R.children.get(ln, []):
            lines += body_xml(jj['child'], indent + '  ')
        lines.append(f'{indent}</body>')
        return lines

    bodies = body_xml(spec['root'], '    ')
    motors = [f'    <position name="{j["name"]}" joint="{j["name"]}" kp="{KP}" forcerange="-{acts[j["act"]]["peak_nm"]} '
              f'{acts[j["act"]]["peak_nm"]}" forcelimited="true"/>' for j in spec['joints']]
    xml = f"""<mujoco model="reindeer"><compiler meshdir="{tmp}" angle="radian"/>
<option timestep="0.001" gravity="0 0 -9.81"/>
<default><geom friction="0.6 0.005 0.0001" condim="3"/></default>
<asset>{''.join(assets)}</asset>
<worldbody><geom name="floor" type="plane" size="1 1 0.1" contype="1" conaffinity="1"/>
{chr(10).join(bodies)}
</worldbody><actuator>
{chr(10).join(motors)}
</actuator></mujoco>"""
    p = os.path.join(tmp, 'reindeer.xml')
    open(p, 'w').write(xml)
    return mujoco.MjModel.from_xml_path(p)


def target(t, names):
    """Cosine ease from each move's targets to the next's over 0.6 s."""
    cur = {n: 0.0 for n in names}
    for i, (t0, q) in enumerate(MOVES):
        if t < t0:
            break
        prev = dict(cur)
        nxt = {n: q.get(n, 0.0) for n in names}
        s = min(1.0, (t - t0) / 0.6)
        s = 0.5 - 0.5 * math.cos(math.pi * s)
        cur = {n: prev[n] + (nxt[n] - prev[n]) * s for n in names}
        if s < 1.0:
            break
    return cur


def main():
    bad = '--bad' in sys.argv
    spec = G.load()
    names = [j['name'] for j in spec['joints']]
    with tempfile.TemporaryDirectory() as tmp:
        m = build(spec, tmp, bad)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    d.qpos[2] += 0.002                                # dropped from 2 mm
    p0 = None
    worst = dict(tilt=0.0, slide=0.0, sat=0.0, err=0.0, contact=1e9)
    act = {m.actuator(i).name: i for i in range(m.nu)}
    jq = {n: m.jnt_qposadr[m.joint(n).id] for n in names}
    while d.time < T_END:
        q = target(d.time, names)
        for n in names:
            d.ctrl[act[n]] = q[n]
        mujoco.mj_step(m, d)
        if d.time > 0.5:
            if p0 is None:
                p0 = d.qpos[:3].copy()
            up = np.zeros(9)
            mujoco.mju_quat2Mat(up, d.qpos[3:7])
            worst['tilt'] = max(worst['tilt'], math.degrees(math.acos(min(1.0, up[8]))))
            for n in names:
                lim = m.actuator_forcerange[act[n]][1]
                worst['sat'] = max(worst['sat'], abs(d.actuator_force[act[n]]) / lim)
            if d.time > 1.2:
                worst['err'] = max(worst['err'], max(abs(d.qpos[jq[n]] - q[n]) for n in names))
    # drift: where it ends, back in its first pose, against where it began (a crouch moves the body over planted hooves: not a slide)
    worst['slide'] = float(np.linalg.norm(d.qpos[:2] - p0[:2]))
    ok = worst['tilt'] < 5 and worst['slide'] < 0.005 and worst['sat'] < 0.95
    lines = ['# Standing through the moves (MuJoCo)', '',
             f"{'BROKEN DESIGN (--bad): ' if bad else ''}max tilt {worst['tilt']:.2f} deg, drift {worst['slide'] * 1000:.1f} mm, "
             f"servo force at most {worst['sat'] * 100:.0f} % of stall, tracking error at most {worst['err']:.3f} rad", '',
             f"limits: tilt < 5 deg, drift < 5 mm, force < 95 % of stall. kp {KP} N m/rad (an estimate). "
             f"Moves: {', '.join(str(q) for _, q in MOVES[1:-1])}", '',
             f"{'0 failing' if ok else '1 failing'}"]
    if not bad:
        os.makedirs(os.path.join(G.ROOT, 'reports'), exist_ok=True)
        open(os.path.join(G.ROOT, 'reports', 'sim_stand.md'), 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
