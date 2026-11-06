"""How the reindeer goes together: its build order as data, and the PATH check on it.

  python -I tools/assembly.py              the check; report in reports/assembly.md, exit 1 on any FAIL
  python -I tools/assembly.py --bad path   break the order on purpose (a hip servo pushed down into its thigh through the
                                           thigh's closed top): PATH must FAIL

Generic over spec/robot.json. Parts are the spec's geoms (not cuts), with three rules:
  - a servo's horns go in with it: '<joint> flange' and '<joint> rear horn ...' ride with '<joint> actuator ...';
  - covers that close round a servo are printed in halves (SPLIT): a one-piece thigh or shin can't get round a servo
    whose horns stand through its walls, nor a one-piece head round the neck's servos, nor a body round the legs;
  - a part the steps don't name is fitted last, coming in from outside (away from the robot's middle), so a part the
    spec gains (a jaw, an antler bar) is still in the build order.

A leg is built on the bench (below its seat), checked against its own parts, then rises into the body's slot as one
unit, checked against the robot. Screws (geoms with a 'fast' field) go in with their step, grouped by size, along
their own axis, when the spec has them.

PATH: along each part's way in (sampled from its start to its seat), it never overlaps a part already on by more than
it does when seated (seats, horns into plates, are designed overlaps), and its start is clear of everything.
Proven on a broken order with --bad path. Fit is not strength: this says nothing about whether a part holds.
"""
import argparse
import os
import re
import sys

import numpy as np
import manifold3d as mf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402

X, Y, Z = np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])
TOUCH = 0.02e-6              # m3, as checks.py
SAMPLES = 14
BENCH = 0.20                 # m: where a leg is built: below its seat, so it rises into the body's slot

# covers printed in halves: (pattern, axis of the split plane's normal, 'centre' of what)
SPLIT = [(r'^body$', 'y'), (r'^cranium$', 'y'), (r'^muzzle$', 'y'), (r'thigh$', 'x'), (r'shin$', 'x')]
NAMES = {'L': 'Left', 'R': 'Right'}
# fused or riding: a hip boss is part of its side's body half (it's fused into the side wall); the tongue rides on the jaw
MERGE = [(r'^L .*hip boss$', lambda n: 'body left half'), (r'^R .*hip boss$', lambda n: 'body right half'),
         (r'^tongue$', lambda n: 'jaw'),
         # the muzzle's shell overlaps the cranium's: they're printed as one, so each half of the head carries its half
         (r'^muzzle (left|right) half$', lambda n: 'cranium ' + n.split()[1] + ' half')]


# ---------------------------------------------------------------- parts

class Part:
    def __init__(self, name, link, geoms, solid, hard, color, kind):
        self.name, self.link, self.geoms, self.solid, self.hard, self.color, self.kind = name, link, geoms, solid, hard, color, kind
        lo, hi = C.bbox(solid)
        self.lo, self.hi = lo, hi
        self.centre = (lo + hi) / 2

    def __repr__(self):
        return f'Part({self.name})'


def kind_of(g):
    n = g['name']
    if g['role'] == 'actuator':
        return 'ROBOTIS ' + n.split()[-1].replace('_', '-') + '-T servo'
    if 'rear horn' in n:
        return 'ROBOTIS FPX330-H101 idler horn'
    if n.endswith('flange'):
        return 'servo horn, in the servo\'s box'
    if 'Pi Zero' in n:
        return 'Raspberry Pi Zero 2 W'
    if n.startswith('battery') and '(buy)' in n:
        return re.sub(r'^battery ', 'LiPo pack ', re.sub(r' \(buy\).*', '', n))
    if 'bus board' in n:
        return 'servo bus board'
    if 'fast' in g:
        return g['fast']['size']
    if '(buy)' in n:                                 # any other bought part: what the spec calls it
        return re.sub(r' \(buy\)', '', n)
    colour = {'#B87333': 'fawn', '#F3EFE6': 'white', '#C42630': 'red', '#1F8A3C': 'green', '#17542E': 'dark green',
              '#1BA05A': 'green', '#08090B': 'black', '#3A3D42': 'grey', '#1B1C1F': 'black'}.get(g['color'])
    return f'printed, PLA, {colour}' if colour else 'printed, PLA'


def pretty(name):
    """'L_front_hip actuator (buy) XL330_M288' -> 'Left front hip servo'."""
    n = re.sub(r' \(buy\).*', '', name)
    n = n.replace(' actuator', ' servo').replace(' flange', ' horn')
    n = re.sub(r'^([LR])[_ ]', lambda m: NAMES[m.group(1)] + ' ', n)
    n = n.replace('_', ' ')
    return n[0].upper() + n[1:]


def load(spec=None):
    """[Part] for the whole robot, each at its seat (zero pose, world metres)."""
    spec = spec or G.load()
    S = C.Solids(spec)
    joint_of = {}
    for j in spec['joints']:
        joint_of[f"{j['name']} actuator (buy) {j['act']}"] = j['name']
    parts, rides = {}, {}
    for ln, l in spec['links'].items():
        for i, g in enumerate(l['geoms']):
            if g['role'] == 'cut' or S.geom.get((ln, i)) is None:
                continue
            solid = S.geom[(ln, i)]
            hard = S.wall.get((ln, i), solid)
            m = re.match(r'^(\S+) (flange|rear horn .*)$', g['name'])
            if m and not g['name'].startswith(('L ', 'R ')):
                rides.setdefault(m.group(1), []).append((g, solid, hard, ln))
                continue
            split = next((ax for pat, ax in SPLIT if re.search(pat, g['name']) and g['role'] == 'shell'), None)
            if split:
                k = 'xyz'.index(split)
                lo, hi = C.bbox(solid)
                c = (lo[k] + hi[k]) / 2
                n = np.zeros(3)
                n[k] = 1.0
                half = {'x': ('front', 'back'), 'y': ('left', 'right')}[split]
                for side, sg in zip(half, (1, -1)):
                    cut = lambda m_: m_.trim_by_plane((sg * n).tolist(), float(sg * c))
                    s_, h_ = cut(solid), cut(hard)
                    if s_.volume() > 1e-9:
                        parts[f"{g['name']} {side} half"] = Part(f"{g['name']} {side} half", ln, [g['name']], s_, h_, g['color'], kind_of(g))
                continue
            nm = g['name'] if g['name'] not in parts else f"{g['name']} ({ln} {i})"   # (a spec may repeat a name)
            parts[nm] = Part(nm, ln, [g['name']], solid, hard, g['color'], kind_of(g))
    for pat, into in MERGE:                           # parts fused to another, or riding on it
        for name in [n for n in parts if re.search(pat, n)]:
            tgt = into(name)
            if tgt in parts and tgt != name:
                q, p = parts.pop(name), parts[tgt]
                p.solid, p.hard = C.union([p.solid, q.solid]), C.union([p.hard, q.hard])
                p.geoms += q.geoms
                p.lo, p.hi = C.bbox(p.solid)
                p.centre = (p.lo + p.hi) / 2
    for name, p in list(parts.items()):              # the horns go in with their servo
        j = joint_of.get(name)
        if j and j in rides:
            for g, solid, hard, ln in rides.pop(j):
                p.solid, p.hard = C.union([p.solid, solid]), C.union([p.hard, hard])
                p.geoms.append(g['name'])
            p.lo, p.hi = C.bbox(p.solid)
            p.centre = (p.lo + p.hi) / 2
            p.kind += ', with its horns'
    for j, rs in rides.items():                      # a horn with no servo of its own (shouldn't happen): its own part
        for g, solid, hard, ln in rs:
            parts[g['name']] = Part(g['name'], ln, [g['name']], solid, hard, g['color'], kind_of(g))
    return spec, parts


# ---------------------------------------------------------------- the build order

def sections(spec, parts):
    """[dict(key, title, bench, way, steps=[(label, [part names], way)])]. bench: where the section is built (an offset
    from its seat); way: how the finished section comes in (it starts at bench, which is clear of the robot)."""
    names = list(parts)
    used = set()

    def take(pat, link=None):
        out = [n for n in names if n not in used and re.search(pat, n) and (link is None or parts[n].link == link)]
        used.update(out)
        return out

    secs = []

    def sec(key, title, steps, bench=None):
        st = [(lab, ns, w) for lab, ns, w in steps if ns]
        if st:
            secs.append(dict(key=key, title=title, bench=bench, steps=st))

    sec('frame', 'The frame', [
        ('Keel', take(r'^keel'), -Z),
        ('Neck cradle', take(r'^neck cradle'), -Z),
        ('Battery tray', take(r'^battery (post|tray)'), -Z),
    ])
    sec('electronics', 'Electronics', [
        ('Raspberry Pi Zero 2 W', take(r'^Pi Zero'), Z),           # from below, under the keel
        ('Battery', take(r'^battery (?!post|tray)'), -Z),
        ('Bus board', take(r'^bus board'), -Z),
    ])
    sec('neck', 'Neck and head servos', [
        ('Neck pitch servo', take(r'^neck_pitch actuator'), -Z),
        ('Neck bracket', take(r'^neck (horn plate|rear plate|bridge|yaw seat)'), -Z),
        ('Head yaw servo', take(r'^head_yaw actuator'), -Z),
        ('Yaw plate', take(r'^(yaw plate|roll rear wall)'), -Z),
        ('Head roll servo', take(r'^head_roll actuator'), -Z),
        ('Head bracket', take(r'^head (bracket|crown post|face post)'), -X),
        ('Jaw servo', take(r'^jaw actuator'), -X),                  # in through the face, before the head closes
        ('Jaw cradle', take(r'^jaw cradle'), -X),
        ('Antler cradle', take(r'^antler cradle'), [-Z, -Y, -X]),     # the cradle first: the servo sits on it
        ('Antler servo', take(r'^antlers? actuator'), [-Y, -Z, -X]),
        ('Antler bar', take(r'^antler (horn plate|bar)'), -Z),
    ])
    # (the body's halves go on before the legs: a leg rises into its slot from below. Brought in from the side after the
    # body, a thigh's top would have to pass through the side wall; a body half after the legs swept its 5 mm belly strip
    # between the hips through the thighs)
    sec('body', 'Body covers', [
        ('Body, left half', take(r'^body left half'), -Y),
        ('Body, right half', take(r'^body right half'), Y),
        ('Bib', take(r'^bib'), -X),
        ('Tail', take(r'^tail'), X),
    ])
    for side in ('L', 'R'):
        sg = 1 if side == 'L' else -1
        for end in ('front', 'rear'):
            pre = f'{side}_{end}'
            n = f'{side} {end}'
            sec(f'leg_{pre}', f'{NAMES[side]} {end} leg', [
                ('Hip servo', take(rf'^{pre}_hip actuator'), -Z),
                ('Thigh', take(rf'^{n} thigh front half'), -X),
                ('Thigh', take(rf'^{n} thigh back half'), X),
                ('Knee servo', take(rf'^{pre}_knee actuator'), Z),
                ('Shin', take(rf'^{n} shin front half'), -X),
                ('Shin', take(rf'^{n} shin back half'), X),
                ('Hoof', take(rf'^{n} hoof'), Z),
                ('Knee inner plate', take(rf'^{n} knee inner (plate|tab)'), sg * Y),
                ('Knee outer plate', take(rf'^{n} knee (plate|outer tab)'), -sg * Y),
                ('Knee cap', take(rf'^{n} knee cap'), [-sg * Y, -X, X]),
            ], bench=-BENCH * Z)
    sec('head', 'Head', [
        ('Left jaw plate', take(r'^L jaw plate'), [-Y, -X, Z]),           # on the jaw servo's horns, before the head closes
        ('Right jaw plate', take(r'^R jaw plate'), [Y, -X, Z]),
        ('Head, left half', take(r'^cranium left half'), -Y),
        ('Head, right half', take(r'^cranium right half'), Y),
        ('Jaw', take(r'^jaw$'), [(-1, 0, 0.3), (-1, 0, 0.6), -X, (-1, 0, 1), Z]),
        ('Nose', take(r'^nose'), -X),
        ('Eyes', take(r'eye (rim|iris|pupil)'), -X),
        ('Ears', take(r' (inner )?ear$'), -Z),
        ('Antlers', take(r'antler (beam|tine)'), -Z),
        ('Left antler burr', take(r'^L antler (burr|tuft)'), [-Z, -Y, -X, X]),
        ('Right antler burr', take(r'^R antler (burr|tuft)'), [-Z, Y, -X, X]),
    ])
    # whatever the steps don't name (a part the spec gained): last, from outside
    mid = np.mean([p.centre for p in parts.values()], axis=0)
    rest = [n for n in names if n not in used]
    late = []
    for n in rest:
        d = parts[n].centre - mid
        d = d / (np.linalg.norm(d) or 1.0)
        late.append((pretty(n), [n], -d))
        used.add(n)
    sec('more', 'Also', late)
    return secs


def way_in(w):
    """A part's way in: the direction it travels (unit), and where it starts: far enough that it's clear."""
    w = np.asarray(w, float)
    return w / np.linalg.norm(w)


# ---------------------------------------------------------------- the PATH check

def moved(m, d):
    return m.translate([float(v) for v in d])


def overlap(a, b):
    return C.overlap(a, b)


def path_hits(movers, placed, travel, d):
    """For a set of moving parts (hard shapes) going from -travel*d to 0, the worst excess overlap with each placed
    part over its overlap when seated. [(excess m3, mover, placed, at)]"""
    hits = []
    for mv_name, mv in movers:
        seated = {pn: overlap(mv, pm) for pn, pm in placed}
        lo, hi = C.bbox(mv)
        for t in np.linspace(1.0, 0.0, SAMPLES)[:-1]:
            off = -d * travel * t
            m = moved(mv, off)
            mlo, mhi = lo + off, hi + off
            for pn, pm in placed:
                plo, phi = C.bbox(pm)
                if np.any(mlo > phi) or np.any(plo > mhi):
                    continue
                v = overlap(m, pm) - seated[pn]
                if v > TOUCH:
                    hits.append((v, mv_name, pn, t))
    return hits


def travel_for(movers, placed, d, start=0.03, most=0.30):
    """The shortest travel (>= start) at which the movers are clear of every placed part and of its bounding box."""
    if not placed:
        return start
    plo = np.min([C.bbox(m)[0] for _, m in placed], axis=0)
    phi = np.max([C.bbox(m)[1] for _, m in placed], axis=0)
    mlo = np.min([C.bbox(m)[0] for _, m in movers], axis=0)
    mhi = np.max([C.bbox(m)[1] for _, m in movers], axis=0)
    tr = start
    while tr < most:
        off = -d * tr
        if np.any(mlo + off > phi) or np.any(plo > mhi + off):
            return tr + 0.01
        tr += 0.005
    return most


def plan(spec=None, bad=None):
    """The checked plan: [dict(sec, label, parts, way, travel, bench, hits)]."""
    spec, parts = load(spec)
    secs = sections(spec, parts)
    if bad == 'path':                                  # a hip servo pushed down into its thigh after the thigh is on
        for s in secs:
            if s['key'] == 'leg_L_front':
                st = s['steps']
                hip = next(i for i, x in enumerate(st) if x[0] == 'Hip servo')
                item = st.pop(hip)
                st.insert(hip + 2, (item[0], item[1], -Z))     # after both halves, from above
    out = []
    robot = []                                        # (name, hard) on the robot
    for s in secs:
        bench = np.zeros(3) if s['bench'] is None else np.asarray(s['bench'], float)
        here = []                                     # parts of this section so far (on the bench, at seat coordinates)
        for lab, ns, w in s['steps']:
            movers = [(n, parts[n].hard) for n in ns]
            against = here if s['bench'] is not None else robot + here
            # a list: candidate ways, in order of preference; the first the check passes is the one used (and filmed)
            for k, w1 in enumerate(w if isinstance(w, list) else [w]):
                d1 = way_in(w1)
                tr1 = travel_for(movers, against, d1)
                h1 = path_hits(movers, against, tr1, d1)
                if k == 0 or not h1:
                    d, tr, hits = d1, tr1, h1
                if not h1:
                    break
            out.append(dict(sec=s, label=lab, parts=ns, way=d, travel=tr, bench=bench, hits=hits, unit=False))
            here += movers
        if s['bench'] is not None:                    # the finished leg comes in from its bench
            d = way_in(-bench)
            tr = float(np.linalg.norm(bench))
            hits = path_hits([(s['title'], C.union([m for _, m in here]))], robot, tr, d)
            out.append(dict(sec=s, label=s['title'], parts=[n for n, _ in here], way=d, travel=tr, bench=bench, hits=hits, unit=True))
        robot += here
    return spec, parts, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bad')
    a = ap.parse_args()
    spec, parts, steps = plan(bad=a.bad)
    rows, fails = [], 0
    for st in steps:
        ok = not st['hits']
        fails += not ok
        worst = max(st['hits'], key=lambda h: h[0]) if st['hits'] else None
        det = f"{worst[1]} cuts {worst[2]} by {worst[0] * 1e6:.2f} cm3, {worst[3] * st['travel'] * 1000:.0f} mm out" if worst else \
            f"{st['travel'] * 1000:.0f} mm along {np.round(st['way'], 2).tolist()}"
        rows.append(f"| {'PASS' if ok else 'FAIL'} | {st['sec']['title']} | {st['label']} | {', '.join(pretty(n) for n in st['parts'][:6])}{' ...' if len(st['parts']) > 6 else ''} | {det} |")
    every = sum(len(s['parts']) for s in steps if not s['unit'])
    rows.append(f"| {'PASS' if every == len(parts) else 'FAIL'} | ALL | every part fitted once | | {every} fitted of {len(parts)} |")
    fails += every != len(parts)
    if a.bad:
        print(f"--bad {a.bad}: {'CAUGHT' if fails else 'NOT CAUGHT'} ({fails} failing)")
        for r in rows:
            if r.startswith('| FAIL'):
                print('  ', r)
        return 0 if fails else 1
    os.makedirs(os.path.join(G.ROOT, 'reports'), exist_ok=True)
    with open(os.path.join(G.ROOT, 'reports', 'assembly.md'), 'w', encoding='utf-8') as f:
        f.write('# Build order and PATH (`tools/assembly.py`)\n\n'
                f'{len(steps)} steps, {len(parts)} parts, **{fails} failing**. Covers that close round a servo are printed in halves '
                '(SPLIT). Legs are built on the bench, then brought in. Fit is not strength.\n\n'
                '| | section | step | parts | path |\n|---|---|---|---|---|\n' + '\n'.join(rows) + '\n')
    for r in rows:
        if r.startswith('| FAIL'):
            print(r)
    print(f'{len(steps)} steps, {fails} failing -> reports/assembly.md')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
