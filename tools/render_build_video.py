"""The full build, in the checked order: every part coming in along the path tools/assembly.py passed, named as it
comes, each joint tested when it can first move, then every joint. Portrait 720 x 1280, 30 fps.

  python tools/render_build_video.py          -> docs/videos/build.mp4, and the least-visible parts and empty stretches
  python tools/render_build_video.py --test   a sheet of frames from every section instead (reports/build_test.png)

Rules: ~/.claude/skills/droid-from-spec/references/assembly-videos.md. Parts only (no hands, no driver), a plain pale
background, two-line labels at the top until the head starts and at the bottom after, the first side slow and its
mirror fast, the camera close on the moving part, from the side that shows it most. Rendered offline from the spec's
shapes (no fillets yet, like the Fusion bodies). Plain python.
"""
import math
import os
import sys

import imageio.v2 as imageio
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly as A  # noqa: E402
import geom as G  # noqa: E402
import reveal_video as RV  # noqa: E402
import stage as ST  # noqa: E402

FPS, W, H = 30, 720, 1280
FLY, HOLD = 50, 16                    # frames: a part coming in, and resting before the next (the first side)
FLY_FAST, HOLD_FAST = 22, 6           # the mirror side
UNIT, TEST, EASE_CAM = 50, 60, 14
SHOW_TRAVEL = 0.08                    # m: the last stretch of a checked path that's shown (all of a leg's rise)
VIEWS = [(az, el) for el in (22, -22) for az in (35, -35, 145, -145, 90, -90, 0, 180)]
GREEN = (30, 140, 60)
OUT = os.path.join(G.ROOT, 'docs', 'videos', 'build.mp4')


def T(off):
    M = np.eye(4)
    M[:3, 3] = off
    return M


def step_labels(st, parts):
    ns = st['parts']
    if st['unit']:
        return st['label'], 'built on the bench, into its slot'
    if len(ns) == 1:
        return (st['label'] if ns[0].endswith(' half') else A.pretty(ns[0])), parts[ns[0]].kind
    kinds = sorted({parts[n].kind for n in ns})
    if len(kinds) == 1:
        return st['label'], kinds[0]
    if all(k.startswith('printed') for k in kinds):
        return st['label'], f'{len(ns)} printed parts, PLA'
    return st['label'], ', '.join(dict.fromkeys(k.split(',')[0] for k in kinds))


def leg_joints(sec_key, spec):
    pre = sec_key.replace('leg_', '')
    return [j['name'] for j in spec['joints'] if j['name'].startswith(pre + '_')]


def run(test=False):
    spec, parts, steps = A.plan()
    R = G.Robot(spec)
    items = []
    for k, p in parts.items():
        v, f = ST.mesh_of(p.solid)
        items.append((k, v, f, p.color))
    st = ST.Stage(items, W, H)
    small = ST.Stage(items, 144, 256)
    pts = {}
    for k, p in parts.items():
        v, _ = ST.mesh_of(p.solid)
        pts.setdefault(p.link, []).append(v[:: max(1, len(v) // 300)])
    pts = {ln: np.vstack(v) for ln, v in pts.items()}

    # the plan as beats: what is on, where, and how each beat moves
    on = {}                                           # part -> offset (bench) while it waits, or zero
    beats = []
    head_started = False
    sec_seen = set()
    for i, s in enumerate(steps):
        key = s['sec']['key']
        mirror = key.startswith('leg_R')
        fly, hold = (FLY_FAST, HOLD_FAST) if mirror else (FLY, HOLD)
        if key == 'head':
            head_started = True
        big, sub = step_labels(s, parts)
        first = key not in sec_seen
        sec_seen.add(key)
        if s['unit']:
            beats.append(dict(kind='unit', parts=s['parts'], frm=s['bench'], d=s['way'], n=UNIT, big=big, sub=sub,
                              where='bottom' if head_started else 'top', title=None))
            if key.startswith('leg_'):
                beats.append(dict(kind='test', joints=leg_joints(key, spec), n=TEST, big='Test: hip and knee', sub=s['sec']['title'],
                                  where='bottom' if head_started else 'top', focus=s['parts']))
        else:
            tr = min(s['travel'], SHOW_TRAVEL)
            beats.append(dict(kind='fly', parts=s['parts'], bench=s['bench'], d=s['way'], tr=tr, n=fly + hold, fly=fly, big=big, sub=sub,
                              where='bottom' if head_started else 'top', title=s['sec']['title'] if first else None))
    beats.append(dict(kind='test', joints=[j['name'] for j in spec['joints'] if not j['name'].endswith(('_hip', '_knee'))], n=TEST + 30,
                      big='Test: neck, head, jaw, antlers', sub='every head joint', where='bottom', focus=None))
    beats.append(dict(kind='finale', n=FPS * 9, big='Test: every joint', sub='', where='bottom'))

    lo_all = np.min([p.lo for p in parts.values()], axis=0)
    hi_all = np.max([p.hi for p in parts.values()], axis=0)
    ks = RV.keyframes(spec)

    def best_view(keys, poses):
        small.place(poses)
        lo = np.min([parts[k].lo for k in keys], axis=0)
        hi = np.max([parts[k].hi for k in keys], axis=0)
        c = (lo + hi) / 2 + (poses[keys[0]][:3, 3] if keys[0] in poses else 0)
        best, score = VIEWS[0], -1
        for az, el in VIEWS:
            small.look(c, small.fit_distance(lo, hi, 1.6), az, el)
            _, sg = small.render(seg=True)
            n = sum(small.visible(k, sg) for k in keys)
            if n > score:
                best, score = (az, el), n
        return best, score

    frames_meta, least = [], []
    cam_prev = None
    placed = {}                                       # part -> offset matrix
    sheet = []
    out_frames = 0
    writer = None
    if not test:
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        writer = imageio.get_writer(OUT, fps=FPS, codec='libx264', quality=7, macro_block_size=8)
    fill = []
    cur_title = (None, None, 0)

    for bi, b in enumerate(beats):
        # where the camera goes for this beat
        if b['kind'] == 'fly':
            keys = b['parts']
            seat = {k: T(b['bench']) for k in keys}
            poses = {k: M for k, M in placed.items()}
            poses.update(seat)
            (az, el), vis = best_view(keys, poses)
            least.append((vis, b['big']))
            lo = np.min([parts[k].lo for k in keys], axis=0) + b['bench']
            hi = np.max([parts[k].hi for k in keys], axis=0) + b['bench']
            start_lo, start_hi = lo - b['d'] * b['tr'], hi - b['d'] * b['tr']
            blo, bhi = np.minimum(lo, start_lo), np.maximum(hi, start_hi)
            # context: what it goes onto (parts already on, within 3 cm), so a servo isn't a box on its own
            near = [k for k, off in placed.items() if np.all(parts[k].lo + off[:3, 3] < hi + 0.03) and np.all(parts[k].hi + off[:3, 3] > lo - 0.03)]
            if near:
                clo = np.min([parts[k].lo + placed[k][:3, 3] for k in near], axis=0)
                chi = np.max([parts[k].hi + placed[k][:3, 3] for k in near], axis=0)
                blo, bhi = np.minimum(blo, clo), np.maximum(bhi, chi)
            target = (lo + hi) / 2 * 0.5 + (blo + bhi) / 2 * 0.5
            dist = st.fit_distance(blo, bhi, 1.3, least=0.24)
        elif b['kind'] == 'unit':
            keys = b['parts']
            lo = np.min([parts[k].lo for k in keys], axis=0)
            hi = np.max([parts[k].hi for k in keys], axis=0)
            blo, bhi = np.minimum(lo, lo + b['frm']), np.maximum(hi, hi + b['frm'])
            blo, bhi = np.minimum(blo, lo_all), np.maximum(bhi, hi_all + [0, 0, 0.05])   # the leg's whole rise, the robot, room for the label
            target, dist = (blo + bhi) / 2, st.fit_distance(blo, bhi, 1.08, least=0.35)
            az, el = (-50 if 'leg_R' in str(keys) or keys[0].startswith('R') else 50), 12
        elif b['kind'] == 'test':
            keys = b['focus'] or [k for k, p in parts.items() if p.link in ('head', 'jaw', 'antlers', 'yawlink', 'neck')]
            lo = np.min([parts[k].lo for k in keys], axis=0)
            hi = np.max([parts[k].hi for k in keys], axis=0)
            target = (lo_all + hi_all) / 2 + [0, 0, 0.012]           # the whole robot: a joint test moves the leg against the body
            dist = st.fit_distance(lo_all, hi_all + [0, 0, 0.04], 1.12)
            az, el = (cam_prev[2] if cam_prev else 40), 12
        else:
            target, dist = (lo_all + hi_all) / 2 + np.array([0, 0, 0.015]), st.fit_distance(lo_all, hi_all, 1.1)
            az, el = -35, 6
        cam_to = (np.asarray(target, float), dist, az, el)
        if cam_prev is None:
            cam_prev = cam_to
        for f in range(b['n']):
            u = ST.ease(f / EASE_CAM)
            daz = ((cam_to[2] - cam_prev[2] + 180) % 360) - 180
            cam = (cam_prev[0] + (cam_to[0] - cam_prev[0]) * u, cam_prev[1] + (cam_to[1] - cam_prev[1]) * u, cam_prev[2] + daz * u,
                   cam_prev[3] + (cam_to[3] - cam_prev[3]) * u)
            q = {}
            poses = {}
            big, sub, color = b['big'], b['sub'], (30, 32, 36)
            if b['kind'] == 'fly':
                s_ = ST.ease(f / b['fly'])
                for k in b['parts']:
                    poses[k] = T(b['bench'] - b['d'] * b['tr'] * (1 - s_))
                if b['title'] and f < 18:
                    big, sub = b['title'], ''          # the section's name, with its first part already in view
            elif b['kind'] == 'unit':
                s_ = ST.ease(f / b['n'])
                for k in b['parts']:
                    poses[k] = T(b['frm'] * (1 - s_))
            elif b['kind'] == 'test':
                ph = f / b['n']
                for j in b['joints']:
                    jj = R.joints[j]
                    amp = 0.6 * (jj['upper'] if math.sin(2 * math.pi * ph) > 0 else -jj['lower'])
                    q[j] = amp * math.sin(2 * math.pi * ph) * (1 if not j.endswith('_knee') else -1 if jj['lower'] < -0.5 else 1)
                color = GREEN
            M = R.fk(q) if b['kind'] != 'finale' else None
            if b['kind'] == 'finale':
                t = 13.0 + 14.0 * f / b['n']
                qq, mv = RV.pose_at(ks, t)
                M = RV.grounded(R, pts, qq)
                big, sub, color = ('Test: every joint', mv or '', GREEN) if f < b['n'] - 75 else ('Reindeer', 'built', (30, 32, 36))
                if f >= b['n'] - 75:                   # end on its face, and hold
                    M = RV.grounded(R, pts, {})
                    cam = (cam[0], cam[1], cam[2] * (1 - ST.ease((f - (b['n'] - 75)) / 30)), cam[3])
            allp = {}
            for k, off in placed.items():
                allp[k] = M[parts[k].link] @ off
            for k, off in poses.items():
                allp[k] = M[parts[k].link] @ off
            want = (not test) or (f == min(b['n'] - 1, (b.get('fly') or b['n']) // 2 + 4) and (b['kind'] != 'fly' or b['title'] or bi % 6 == 0))
            if not want:                                  # (a test sheet renders only its sampled frames)
                out_frames += 1
                continue
            st.place(allp)
            st.look(cam[0], cam[1], cam[2], cam[3])
            img = st.render()
            if (big, sub) != cur_title[:2]:
                cur_title = (big, sub, out_frames)
            a = 1.0 if test else ST.ease((out_frames - cur_title[2]) / 9)
            img = ST.label(img, big, sub, b['where'] if b['kind'] != 'finale' else 'bottom', alpha=a, color=color)
            arr = np.asarray(img)
            fill.append(float((np.abs(arr.astype(int) - ST.BG).sum(axis=2) > 30).mean()))
            if test:
                sheet.append(img.resize((180, 320)))
            else:
                writer.append_data(arr)
            out_frames += 1
        cam_prev = (np.asarray(cam[0]), cam[1], cam[2], cam[3])
        if b['kind'] == 'fly':
            for k in b['parts']:
                placed[k] = T(b['bench'])
        elif b['kind'] == 'unit':
            for k in b['parts']:
                placed[k] = np.eye(4)
    if test:
        cols = 8
        rows = math.ceil(len(sheet) / cols)
        sh = Image.new('RGB', (cols * 180, rows * 320), 'white')
        for i, im in enumerate(sheet):
            sh.paste(im, ((i % cols) * 180, (i // cols) * 320))
        p = os.path.join(G.ROOT, 'reports', 'build_test.png')
        sh.save(p)
        print(p, f'(a sheet of {len(sheet)} portrait frames)')
    else:
        writer.close()
    empty, run_, stretches = 0, 0, []
    for i, x in enumerate(fill + [1.0]):
        if x < 0.02:
            run_ += 1
        else:
            if run_:
                stretches.append((i - run_, run_))
            run_ = 0
    least.sort()
    print(f'{out_frames} frames, {out_frames / FPS:.0f} s; empty stretches (under 2 % filled): {len(stretches)}, {sum(n for _, n in stretches) / FPS:.1f} s')
    print('least visible:', [(n, v) for v, n in least[:6]])


if __name__ == '__main__':
    run('--test' in sys.argv)
