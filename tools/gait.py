"""A trot for the reindeer in MuJoCo: a parametric gait, tuned by a short, time-boxed search on the CPU.

  python tools/gait.py --tune [--minutes 18]   search, write sim/gait_params.json (the user's rule: no long training here)
  python tools/gait.py                        evaluate sim/gait_params.json and print its numbers

Not a trained policy. Each leg's hip and knee follow sines at one frequency; diagonal pairs (left front with right
rear) are half a cycle apart. All servos are position servos limited to the XL330's stall torque, as in
tools/sim_stand.py (its build(), from spec/robot.json at run time: joints this file doesn't drive stay at 0).

Signs (spec/robot.py): a hip or knee turning positive about +y moves what is below it backward. A stance foot sweeps
backward (hip angle rising) while the knee is straighter; a swing foot comes forward with the knee bent (lifted).

Parameters, each in a box (LO, HI):
  f        step frequency, Hz
  ah_f, ah_r   hip amplitude, front and rear, rad
  ak_f, ak_r   knee amplitude, front and rear, rad (half a sine: it only bends during swing)
  crouch   a standing bend: hip +c, knee -2c (front), hip -c, knee +2c (rear: knees bend the other way)
  oh_f, oh_r   hip offsets, rad (lean the legs to put the CoM over the stance)
  psi      the knee's phase lead over the hip, rad
  dk       which way the knee bends to lift: +1 or -1 (front, rear), fixed by a short sweep before the search

Score: forward travel (m, +x) over the run, less sideways drift, yaw and time saturated; a fall (tilt > 20 deg) ends
the run and costs 1 m. Plain python.
"""
import argparse
import json
import math
import os
import sys
import tempfile
import time

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402
import sim_stand as SS  # noqa: E402

OUT = os.path.join(G.ROOT, 'sim', 'gait_params.json')
NAMES = ['f', 'ah_f', 'ah_r', 'ak_f', 'ak_r', 'crouch', 'oh_f', 'oh_r', 'psi']
LO = np.array([0.8, 0.05, 0.05, 0.0, 0.0, 0.0, -0.3, -0.3, -math.pi])
HI = np.array([3.0, 0.50, 0.50, 1.0, 1.0, 0.35, 0.3, 0.3, math.pi])
START = np.array([1.6, 0.25, 0.25, 0.5, 0.5, 0.15, 0.0, 0.0, -math.pi / 2])
LEGS = {'L_front': 0.0, 'R_rear': 0.0, 'R_front': math.pi, 'L_rear': math.pi}
T_RUN = 5.0
WARM = 0.8                     # stand still, then ramp the gait in over 0.5 s
DT = 0.002                     # 2 ms: a search needs speed; the render runs at 1 ms


def model():
    spec = G.load()
    with tempfile.TemporaryDirectory() as tmp:
        m = SS.build(spec, tmp)
    m.opt.timestep = DT
    return spec, m


def targets(p, t, dk=(1, 1)):
    """{joint: angle} for parameters p (dict) at time t (the gait starts at WARM)."""
    s = min(1.0, max(0.0, (t - WARM) / 0.5))
    w = 2 * math.pi * p['f'] * max(0.0, t - WARM)
    q = {}
    for leg, ph in LEGS.items():
        front = 'front' in leg
        ah, ak, oh = (p['ah_f'], p['ak_f'], p['oh_f']) if front else (p['ah_r'], p['ak_r'], p['oh_r'])
        c = p['crouch'] * (1 if front else -1)
        d = dk[0] if front else dk[1]
        hip = c + s * (oh + ah * math.sin(w + ph))
        lift = max(0.0, math.sin(w + ph + p['psi']))          # bend only in swing
        knee = -2 * c + s * d * ak * lift
        q[f'{leg}_hip'], q[f'{leg}_knee'] = hip, knee
    return q


def rollout(m, p, dk=(1, 1), T=T_RUN, record=None, every=0.0333):
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    act = {m.actuator(i).name: i for i in range(m.nu)}
    lim = {n: m.actuator_forcerange[i][1] for n, i in act.items()}
    rng_ = {n: m.jnt_range[m.joint(n).id] for n in act}       # targets kept inside the joints' limits
    sat_t, fell, nxt = 0.0, False, 0.0
    up = np.zeros(9)
    x0 = None
    while d.time < T:
        q = targets(p, d.time, dk)
        for n, i in act.items():
            d.ctrl[i] = min(max(q.get(n, 0.0), rng_[n][0]), rng_[n][1])
        mujoco.mj_step(m, d)
        if record is not None and d.time >= nxt:
            record.append(d.qpos.copy())
            nxt += every
        if d.time > WARM and x0 is None:
            x0 = d.qpos[:3].copy()
        mujoco.mju_quat2Mat(up, d.qpos[3:7])
        if up[8] < math.cos(math.radians(20)):
            fell = True
            break
        if d.time > WARM and max(abs(d.actuator_force[i]) / lim[n] for n, i in act.items()) > 0.98:
            sat_t += m.opt.timestep
    x0 = x0 if x0 is not None else d.qpos[:3].copy()
    dx, dy = d.qpos[0] - x0[0], d.qpos[1] - x0[1]
    yaw = math.atan2(up[3], up[0])
    dur = max(d.time - WARM, 1e-3)
    score = dx - 0.5 * abs(dy) - 0.05 * abs(yaw) - 0.05 * sat_t / dur - (1.0 if fell else 0.0)
    return dict(score=float(score), speed=float(dx / dur), dx=float(dx), dy=float(dy), yaw_rad=float(yaw),
                fell=fell, saturated_share=float(sat_t / dur), t=float(d.time))


def as_dict(x):
    return {k: float(v) for k, v in zip(NAMES, x)}


def tune(minutes, minutes_spent=0.0):
    spec, m = model()
    t0 = time.time()
    # which way each pair's knee lifts: a quick look at the start point both ways
    best_dk, best = (1, 1), -1e9
    for dk in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        r = rollout(m, as_dict(START), dk, T=3.0)
        print('knee sign', dk, round(r['score'], 3), round(r['speed'], 3), 'fell' if r['fell'] else '', flush=True)
        if r['score'] > best:
            best, best_dk = r['score'], dk
    # a small CMA-ES in the unit box
    n = len(NAMES)
    norm = lambda x: (x - LO) / (HI - LO)
    den = lambda u: LO + np.clip(u, 0, 1) * (HI - LO)
    mean, sigma = norm(START), 0.2
    lam = 16
    mu = lam // 2
    wts = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1))
    wts /= wts.sum()
    mueff = 1 / (wts ** 2).sum()
    cc, cs = 4 / (n + 4), (mueff + 2) / (n + mueff + 5)
    c1, cmu = 2 / ((n + 1.3) ** 2 + mueff), min(1 - 2 / ((n + 1.3) ** 2 + mueff), 2 * (mueff - 2 + 1 / mueff) / ((n + 2) ** 2 + mueff))
    ds = 1 + 2 * max(0, math.sqrt((mueff - 1) / (n + 1)) - 1) + cs
    chin = math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
    C, pc, ps = np.eye(n), np.zeros(n), np.zeros(n)
    rng = np.random.default_rng(0)
    top = dict(score=-1e9)
    gen, evals = 0, 0
    while time.time() - t0 < minutes * 60:
        ev, B = np.linalg.eigh(C)
        D = np.sqrt(np.maximum(ev, 1e-12))
        zs = rng.standard_normal((lam, n))
        ys = zs @ np.diag(D) @ B.T
        xs = mean + sigma * ys
        res = []
        for x in xs:
            r = rollout(m, as_dict(den(x)), best_dk)
            r['x'] = den(x)
            res.append(r)
            evals += 1
            if r['score'] > top['score']:
                top = r
        order = np.argsort([-r['score'] for r in res])
        yw = (ys[order[:mu]] * wts[:, None]).sum(0)
        mean = mean + sigma * yw
        Cinvsqrt = B @ np.diag(1 / D) @ B.T
        ps = (1 - cs) * ps + math.sqrt(cs * (2 - cs) * mueff) * Cinvsqrt @ yw
        hs = np.linalg.norm(ps) / math.sqrt(1 - (1 - cs) ** (2 * (gen + 1))) < (1.4 + 2 / (n + 1)) * chin
        pc = (1 - cc) * pc + hs * math.sqrt(cc * (2 - cc) * mueff) * yw
        art = ys[order[:mu]]
        C = (1 - c1 - cmu) * C + c1 * (np.outer(pc, pc) + (1 - hs) * cc * (2 - cc) * C) + cmu * (art.T * wts) @ art
        sigma *= math.exp((cs / ds) * (np.linalg.norm(ps) / chin - 1))
        gen += 1
        save(spec, top, best_dk, lam, gen, evals, t0, minutes_spent + (time.time() - t0) / 60)
        print(f"gen {gen} ({time.time() - t0:.0f} s): best {top['score']:.3f} speed {top['speed']:.3f} m/s "
              f"dy {top['dy']:.3f} fell {top['fell']}", flush=True)
    print(json.dumps({k: v for k, v in top.items() if k != 'x'}, indent=1))
    return top


def save(spec, top, dk, lam, gen, evals, t0, total_min):
    out = dict(params=as_dict(top['x']), knee_sign=list(dk),
               result={k: v for k, v in top.items() if k != 'x'},
               search=dict(method='CMA-ES (numpy), unit box', population=lam, generations=gen, evaluations=evals,
                           minutes_this_run=round((time.time() - t0) / 60, 1), minutes_all_runs=round(total_min, 1),
                           run_s=T_RUN, timestep=DT, kp=SS.KP,
                           note='position servos at kp %.1f N m/rad, limited to stall: an estimate; duck-droids trainer '
                                'uses 0.55 (microduck identification)' % SS.KP),
               spec_joints=[j['name'] for j in spec['joints']])
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump(out, open(OUT, 'w', encoding='utf-8'), indent=1)


def load_params(which='video'):
    """The gait the videos use: the search's best scaled down (sim/gait_params.json 'video' says why), else the raw best."""
    g = json.load(open(OUT, encoding='utf-8'))
    return (g[which]['params'] if which in g else g['params']), tuple(g['knee_sign'])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tune', action='store_true')
    ap.add_argument('--minutes', type=float, default=18.0)
    ap.add_argument('--spent', type=float, default=0.0, help='minutes of search already run today: the cap is 20 in all')
    a = ap.parse_args()
    if a.tune:
        tune(min(a.minutes, 20.0 - a.spent), a.spent)
        return 0
    spec, m = model()
    p, dk = load_params()
    m.opt.timestep = 0.001
    r = rollout(m, p, dk, T=10.0)
    print(json.dumps(r, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
