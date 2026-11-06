"""The 30-second reveal: what the reindeer is made of, on a slow turn (servos, then the printed skeleton, then the
electronics, then the covers), then its moves, then its face. Portrait 1080 x 1920, 30 fps.

  python tools/reveal_video.py           -> docs/videos/reveal_30s.mp4
  python tools/reveal_video.py --test    a sheet of frames instead (reports/reveal_test.png)

Kinematic: the spec's joints posed by hand-set keyframes, the body lowered each frame onto whatever is lowest. It shows
the moves, not a physics simulation or a trained policy. Rendered offline from the spec's shapes with MuJoCo (the Fusion
bodies have no fillets yet either, so the two look the same). Plain python.
"""
import math
import os
import sys

import imageio.v2 as imageio
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly as A  # noqa: E402
import geom as G  # noqa: E402
import stage as ST  # noqa: E402

FPS, W, H, SECONDS = 30, 1080, 1920, 30
OUT = os.path.join(G.ROOT, 'docs', 'videos', 'reveal_30s.mp4')


def layer(p):
    """0 servos (with their horns), 1 printed skeleton, 2 electronics, 3 covers."""
    if p.kind.startswith('ROBOTIS') and 'servo' in p.kind:
        return 0
    if p.kind.startswith(('Raspberry', 'LiPo', 'servo bus')):
        return 2
    g = p.geoms[0]
    return 3 if p.role in ('shell', 'decal') else 1


def build():
    spec, parts = A.load()
    roles = {g['name']: g['role'] for l in spec['links'].values() for g in l['geoms']}
    for p in parts.values():
        p.role = roles.get(p.geoms[0], 'struct')
    R = G.Robot(spec)
    items = []
    for k, p in parts.items():
        v, f = ST.mesh_of(p.solid)
        items.append((k, v, f, p.color))
    st = ST.Stage(items, W, H)
    # a few points of each link, to find the lowest each frame
    pts = {}
    for k, p in parts.items():
        v, _ = ST.mesh_of(p.solid)
        pts.setdefault(p.link, []).append(v[:: max(1, len(v) // 400)])
    pts = {ln: np.vstack(v) for ln, v in pts.items()}
    return spec, parts, R, st, pts


def keyframes(spec):
    """[(t, {joint: rad}, title)]: the moves, eased between keys."""
    J = {j['name']: j for j in spec['joints']}
    lim = lambda n, f: (J[n]['upper'] if f > 0 else J[n]['lower']) * abs(f) if n in J else 0.0
    legs = lambda h, k: {n: (h if 'hip' in n else k) for n in J if n.endswith(('_hip', '_knee'))}
    ks = [(13.0, {}, 'Head tilt'), (13.8, {'head_roll': lim('head_roll', 0.85)}, 'Head tilt'), (14.8, {'head_roll': lim('head_roll', -0.85)}, 'Head tilt'),
          (15.5, {}, 'Looking round'), (16.4, {'head_yaw': lim('head_yaw', 0.85)}, 'Looking round'), (17.4, {'head_yaw': lim('head_yaw', -0.85)}, 'Looking round'),
          (18.1, {}, 'Nodding'), (18.6, {'neck_pitch': lim('neck_pitch', 0.9)}, 'Nodding'), (19.2, {'neck_pitch': lim('neck_pitch', -0.8)}, 'Nodding'), (19.7, {}, 'Nodding')]
    t = 19.7
    if 'jaw' in J:
        for i in range(3):
            ks += [(t + 0.35, {'jaw': lim('jaw', 0.9) if J['jaw']['upper'] > 0.05 else lim('jaw', -0.9)}, 'Chatting'), (t + 0.7, {}, 'Chatting')]
            t += 0.7
    if 'antlers' in J:
        ks += [(t + 0.5, {'antlers': lim('antlers', 0.9)}, 'Antlers'), (t + 1.1, {'antlers': lim('antlers', -0.9)}, 'Antlers'), (t + 1.6, {}, 'Antlers')]
        t += 1.6
    paw = {'L_front_hip': lim('L_front_hip', -0.95), 'L_front_knee': lim('L_front_knee', 0.85)}
    ks += [(t + 0.7, dict(paw, head_roll=lim('head_roll', 0.6)), 'A paw lifted'), (t + 1.6, dict(paw, head_roll=lim('head_roll', 0.6)), 'A paw lifted'), (t + 2.3, {}, 'A paw lifted')]
    t += 2.3
    crouch = legs(0.5, -1.0)
    ks += [(min(t + 0.8, 26.4), crouch, 'Crouching'), (min(t + 1.6, 26.8), crouch, 'Crouching'), (27.0, {}, 'Crouching')]
    return ks


def pose_at(ks, t):
    if t <= ks[0][0]:
        return {}, None
    for (t0, q0, n0), (t1, q1, n1) in zip(ks, ks[1:]):
        if t0 <= t <= t1:
            s = ST.ease((t - t0) / (t1 - t0))
            names = set(q0) | set(q1)
            return {n: q0.get(n, 0.0) + (q1.get(n, 0.0) - q0.get(n, 0.0)) * s for n in names}, n1
    return {}, None


def grounded(R, pts, q):
    M = R.fk(q)
    low = min(G.apply(M[ln], v)[:, 2].min() for ln, v in pts.items())
    for ln in M:
        M[ln] = M[ln].copy()
        M[ln][2, 3] -= low
    return M


SEEN = {'title': None, 'since': 0.0}


def frame(spec, parts, R, st, pts, t, ks, lo, hi):
    title, sub = '', ''
    q, mv = pose_at(ks, t)
    M = grounded(R, pts, q)
    poses = {k: M[p.link] for k, p in parts.items()}
    alpha = {}
    if t < 1.0:                                          # the whole robot, from the front, with its title
        az, title, sub = 0.0, 'Reindeer', f"a walking toy droid on {len(spec['joints'])} servos"
    elif t < 13.0:                                       # what it's made of, on one slow turn
        u = (t - 1.0) / 12.0
        az = 360.0 * ST.ease(u)
        stage_t = [1.0, 4.0, 7.0, 10.0]
        names = [('Its servos', f"{sum(1 for p in parts.values() if layer(p) == 0)} ROBOTIS XL330-M288"),
                 ('A printed skeleton', 'PLA'), ('Electronics', 'Pi Zero 2 W, LiPo pack, bus board'), ('Printed covers', 'PLA, in its own colours')]
        cur = max(i for i, s in enumerate(stage_t) if t >= s)
        title, sub = names[cur]
        for k, p in parts.items():
            Ly = layer(p)
            # (the servos stay solid from the first: fading them in left two empty frames at 1.0 s)
            alpha[k] = 0.0 if Ly > cur else (1.0 if Ly < cur or Ly == 0 else ST.ease((t - stage_t[cur]) / 0.6))
    elif t < 27.0:                                       # the moves, three-quarters from the front
        az = -35.0 + 10.0 * math.sin((t - 13.0) / 14.0 * math.pi)
        title = mv or ''
    else:                                                # back to its face, and hold
        az = -35.0 * (1 - ST.ease((t - 27.0) / 1.2))
        title, sub = ('Reindeer', 'printed, servo-driven, and still to be built') if t > 28.2 else ('', '')
    st.place(poses, alpha)
    c = (lo + hi) / 2
    st.look(c + np.array([0, 0, 0.018]), st.fit_distance(lo, hi, margin=1.08), az, 6.0)
    img = st.render()
    if title != SEEN['title']:                         # a title fades in when it changes
        SEEN.update(title=title, since=t)
    a = ST.ease((t - SEEN['since']) / 0.35)
    return ST.label(img, title, sub, 'top', alpha=1.0 if t < 1.0 else a)


def main():
    test = '--test' in sys.argv
    spec, parts, R, st, pts = build()
    ks = keyframes(spec)
    lo = np.min([p.lo for p in parts.values()], axis=0)
    hi = np.max([p.hi for p in parts.values()], axis=0)
    hi = hi + np.array([0, 0, 0.03])                    # room above the antlers for the title band
    if test:
        ts = [0.5, 2.5, 5.5, 8.5, 11.5, 14.0, 16.5, 18.6, 21.0, 24.5, 26.5, 29.5]
        ims = []
        for t in ts:
            SEEN.update(title=None)
            frame(spec, parts, R, st, pts, t - 0.5, ks, lo, hi)
            ims.append(frame(spec, parts, R, st, pts, t, ks, lo, hi).resize((270, 480)))
        sheet = ST.Image.new('RGB', (270 * 6, 480 * 2), 'white')
        for i, im in enumerate(ims):
            sheet.paste(im, ((i % 6) * 270, (i // 6) * 480))
        out = os.path.join(G.ROOT, 'reports', 'reveal_test.png')
        sheet.save(out)
        print(out, '(a sheet of portrait frames)')
        return
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    w = imageio.get_writer(OUT, fps=FPS, codec='libx264', quality=7, macro_block_size=8)
    fill = []
    for i in range(FPS * SECONDS):
        img = frame(spec, parts, R, st, pts, i / FPS, ks, lo, hi)
        a = np.asarray(img)
        fill.append(float((np.abs(a.astype(int) - ST.BG).sum(axis=2) > 30).mean()))
        w.append_data(a)
    w.close()
    empty = sum(1 for f in fill if f < 0.02)
    print(OUT, f'{len(fill) / FPS:.1f} s, frames under 2 % filled: {empty}, least filled {min(fill) * 100:.1f} %')


if __name__ == '__main__':
    main()
