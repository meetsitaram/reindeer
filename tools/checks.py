"""Physical-validity checks for the reindeer (forked from duck-droids' tools/checks.py, Apache-2.0), run offline on spec/robot.json before anything is built in Fusion.

  python -I tools/checks.py                 all checks, report in reports/checks.md, exit 1 on any FAIL
  python -I tools/checks.py --only sweep    one group (targets mass axis static connect skin sweep poses)
  python -I tools/checks.py --bad torque    break the design on purpose and show the matching check FAILs
                                            (cases: torque collide offaxis floating mass poke)

What each group guards against (the CDX-3R lesson it carries over in brackets):
  targets  proportions match microduck's (its model, scaled to milli's height); the height itself a warning
  mass     the bottom-up mass (tools/mass_model.py: every part from its material) within the mass budget
  axis     every directly driven actuator is coaxial with its joint
  bearing  every bearing on its joint's axis, its two races on the two links the joint moves apart
  static   at the zero pose no rigid part is inside another part, and no skin is inside a non-adjacent link
  covers   no cover (closed on its flat cut faces, as Fusion draws it) passes through a motor of its own link or a
           neighbouring one
           (collision_check: every bought part modelled, in every pose)
  connect  each link is one connected solid and each joint axis passes through both links it joins
           (design_rules 'floating': nothing hangs in thin air)
  skin     internal parts stay inside the skin, except parts meant to show (exposed actuators)
  sweep    each joint swept through its limits: limits are reachable without collisions (motion sweep)
  poses    task poses: static joint torques vs actuator ratings, centre of mass over the feet, no collisions
"""
import argparse
import copy
import json
import os
import sys
import time

import numpy as np
import manifold3d as mf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402
import mass_model as MM  # noqa: E402

ROOT = G.ROOT
G_ACC = 9.81
# m^3: intersections below this count as touching, not colliding. echo's 0.3 cm^3 is a fifth of a bearing on a robot this
# size, and hid real collisions: a shoe in the ankle's cradle, a fork's post in the thigh plate (2026-11-02)
TOUCH = 0.02e-6
SEATED = 0.3e-6           # a part modelled seated a little into a servo of its own link (they are one solid to the checks)
ARM_KEYS, LEG_KEYS = ('shoulder', 'elbow', 'wrist', 'hand'), ('hip', 'knee', 'ankle', 'foot')


# ---------------------------------------------------------------- report

class Report:
    def __init__(self):
        self.rows = []

    def add(self, group, item, ok, detail='', warn=False):
        status = 'PASS' if ok else ('WARN' if warn else 'FAIL')
        self.rows.append(dict(group=group, item=item, status=status, detail=detail))
        return ok

    def failing(self):
        return [r for r in self.rows if r['status'] == 'FAIL']

    def write(self, path, header=''):
        lines = [f'# Physical checks\n', header, '',
                 f"{len(self.rows)} checks: {sum(r['status'] == 'PASS' for r in self.rows)} pass, "
                 f"{sum(r['status'] == 'WARN' for r in self.rows)} warn, **{len(self.failing())} fail**", '']
        for grp in dict.fromkeys(r['group'] for r in self.rows):
            rows = [r for r in self.rows if r['group'] == grp]
            lines += [f'## {grp}', '', '| status | item | detail |', '|---|---|---|']
            lines += [f"| {r['status']} | {r['item']} | {r['detail']} |" for r in rows]
            lines.append('')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))
        with open(path.replace('.md', '.json'), 'w', encoding='utf-8') as f:
            json.dump(self.rows, f, indent=1)


# ---------------------------------------------------------------- solids

def to_manifold(mesh):
    m = mf.Manifold(mf.Mesh(vert_properties=np.asarray(mesh.vertices, np.float32),
                            tri_verts=np.asarray(mesh.faces, np.uint32)))
    if m.status() != mf.Error.NoError:
        raise ValueError(f'not a manifold mesh: {m.status()}')
    return m


def union(ms):
    ms = [m for m in ms if m is not None]
    return mf.Manifold.batch_boolean(ms, mf.OpType.Add) if len(ms) > 1 else (ms[0] if ms else None)


def posed(m, M):
    return m.transform(M[:3, :].astype(np.float64))


def overlap(a, b):
    if a is None or b is None:
        return 0.0
    (alo, ahi), (blo, bhi) = bbox(a), bbox(b)
    if np.any(alo > bhi) or np.any(blo > ahi):
        return 0.0
    return (a ^ b).volume()


def bbox(m):
    b = m.bounding_box()
    return np.array(b[:3]), np.array(b[3:])


WALL = 0.002              # skin wall thickness for collisions (a 2 mm printed PLA shell)


class Solids:
    """Manifolds per geom and per link at the zero pose.

    all:   every solid of the link (skins as solids): for connectivity and the look
    hard:  what can actually collide: rigid parts plus the skins' WALLS (a skin is hollow; parts of other links
           may sit in its cavity as long as they never touch the wall)
    """

    def __init__(self, spec):
        self.spec = spec
        self.geom, self.wall = {}, {}
        for ln, l in spec['links'].items():
            # 'cut' geoms are not parts: they are removed from their targets (a window through a shoe)
            cuts = {}
            for g in l['geoms']:
                if g['role'] == 'cut':
                    m = to_manifold(G.mesh_of(g))
                    for t in g['targets']:
                        cuts[t] = m if t not in cuts else cuts[t] + m
            for i, g in enumerate(l['geoms']):
                if g['role'] == 'cut':
                    self.geom[(ln, i)] = None
                    continue
                self.geom[(ln, i)] = to_manifold(G.mesh_of(g))
                if g['name'] in cuts:
                    self.geom[(ln, i)] = self.geom[(ln, i)] - cuts[g['name']]
                if g['role'] == 'shell':
                    cav = G.inner_of(g, WALL)
                    self.wall[(ln, i)] = self.geom[(ln, i)] - to_manifold(G.mesh_of(cav))
        # shells printed as one piece (merge_into): neither has a wall inside the other (the reindeer's muzzle and cranium)
        self.merged = {}
        for ln, l in spec['links'].items():
            idx = {g['name']: i for i, g in enumerate(l['geoms'])}
            for i, g in enumerate(l['geoms']):
                if g['role'] == 'shell' and g.get('merge_into') in idx:
                    k = idx[g['merge_into']]
                    self.merged.setdefault((ln, i), []).append((ln, k))
                    self.merged.setdefault((ln, k), []).append((ln, i))
        for key, others in self.merged.items():
            for o in others:
                self.wall[key] = self.wall[key] - self.geom[o]
        self.all, self.shell, self.rigid, self.hard = {}, {}, {}, {}
        for ln, l in spec['links'].items():
            idx = range(len(l['geoms']))
            role = lambda i: l['geoms'][i]['role']
            # (a linkage moves by itself: it joins nothing; a tab on a cover is mounted on the cover, not the structure)
            free = lambda i: l['geoms'][i].get('linkage')
            self.all[ln] = union([self.geom[(ln, i)] for i in idx if not free(i)])
            self.shell[ln] = union([self.geom[(ln, i)] for i in idx if role(i) == 'shell'])
            # a 'load_bearing' shell is structure too (the reindeer's thighs hold their servos; its body's side carries each hip's outer boss)
            self.rigid[ln] = union([self.geom[(ln, i)] for i in idx if (role(i) in G.RIGID or l['geoms'][i].get('load_bearing')) and not free(i) and not l['geoms'][i].get('on_cover')])
            self.hard[ln] = union([self.geom[(ln, i)] for i in idx if role(i) in G.RIGID] +
                                  [self.wall[(ln, i)] for i in idx if role(i) == 'shell'])


def exposed_names(spec):
    """Geoms meant to show: exposed actuators, their flanges, and geoms flagged exposed."""
    out = {g['name'] for l in spec['links'].values() for g in l['geoms'] if g.get('exposed')}
    for j in spec['joints']:
        if j['act'] and j['exposed'] == 'flange':        # only the flange shows (in a seam); the actuator is covered
            out.add(f"{j['name']} flange")
        elif j['act'] and j['exposed']:
            out |= interface(j)
    return out


def interface(j):
    """Names of the geoms that are meant to touch at joint j: its actuator and its flange."""
    return {f"{j['name']} actuator (buy) {j['act']}", f"{j['name']} flange"} | set(j.get('interface', ()))


def group(link):
    if any(k in link for k in ARM_KEYS):
        return 'arm'
    if any(k in link for k in LEG_KEYS):
        return 'leg'
    return 'body'


# ---------------------------------------------------------------- checks

def measure(spec, S):
    """The reindeer's landmarks, read from the finished shapes (cuts applied) at the zero pose, as fractions of its
    height (floor to antler tip), the way spec/targets.json measured the reference's."""
    names = {}
    for (ln, i), m in S.geom.items():
        if m is not None:
            names[spec['links'][ln]['geoms'][i]['name']] = (ln, i, m)
    bb = lambda n: bbox(names[n][2])
    G_ = lambda n: spec['links'][names[n][0]]['geoms'][names[n][1]]
    allv = np.vstack([m.to_mesh().vert_properties[:, :3] for (_, _, m) in names.values()])
    H = float(allv[:, 2].max())
    head = [bb(n) for n in ('cranium', 'muzzle', 'jaw') if n in names]   # the chin is the jaw's, once it has one
    ears = [bb(n) for n in ('L ear', 'R ear')]
    rim = G_('L eye rim')
    body = bb('body')
    out = {
        'crown_z': bb('cranium')[1][2],
        'ear_span': max(e[1][1] for e in ears) - min(e[0][1] for e in ears),
        'head_width': bb('cranium')[1][1] - bb('cranium')[0][1],
        'eye_z': rim['c'][2],
        'eye_spacing': 2 * abs(rim['c'][1]),
        'eye_d': 2 * rim['r'],
        'chin_z': min(h[0][2] for h in head),
        'back_z': body[1][2],
        'belly_z': body[0][2],
        'body_length': body[1][0] - body[0][0],
    }
    return H, {k: v / H for k, v in out.items()}


def check_targets(spec, S, rep):
    """Proportions: each landmark over the height, against the reference's fraction (spec/targets.json), +/- tol of it."""
    tg = json.load(open(os.path.join(ROOT, 'spec', 'targets.json'), encoding='utf-8'))
    H, m = measure(spec, S)
    rep.add('targets', 'height', abs(H - tg['height_m']) <= tg['height_tol'] * tg['height_m'],
            f"{H * 1000:.0f} mm (aim {tg['height_m'] * 1000:.0f} mm +/- {tg['height_tol'] * 100:.0f} %)")
    for key, v in tg['fractions'].items():
        want = v['v']
        ok = abs(m[key] - want) <= tg['tol'] * want
        # a gap the parts force is reported, with its reason, as a warning: never a tolerance widened to pass
        rep.add('targets', key, ok, f"{m[key]:.3f} H (reference {want:.3f} H +/- {tg['tol'] * 100:.0f} %; {v['src']})"
                + (f". DEVIATION: {v['deviation']}" if not ok and 'deviation' in v else ''), warn='deviation' in v)
    return m


def check_mass(spec, S, rep):
    tg = json.load(open(os.path.join(ROOT, 'spec', 'targets.json'), encoding='utf-8'))
    inert = MM.link_inertials(spec, S)
    total = sum(v[0] for v in inert.values())
    rep.add('mass', 'total', total <= tg['mass_budget_kg'], f"{total * 1000:.0f} g bottom-up (budget {tg['mass_budget_kg'] * 1000:.0f} g)")


BED = 0.256                     # a Bambu printer's bed and height: X1, P1 and A1 series (2026-11-03, the user: no part too big for one)
PRINTED = ('pla', 'petg', 'tpu', 'pa12')


def check_printable(spec, S, rep):
    """Every printed part fits the bed, in some square orientation: its three sizes, sorted, each within the bed."""
    worst, n = None, 0
    for (ln, i), m in S.geom.items():
        g = spec['links'][ln]['geoms'][i]
        if m is None or g['role'] == 'cut' or g.get('material') not in PRINTED:
            continue
        v = np.asarray(m.to_mesh().vert_properties[:, :3], float)
        if not len(v):
            continue
        n += 1
        size = np.sort(np.ptp(v, axis=0))
        if worst is None or size[-1] > worst[0][-1]:
            worst = (size, g['name'])
        if size[-1] > BED:
            rep.add('printable', g['name'], False, f"{size[-1] * 1000:.0f} mm long: over the {BED * 1000:.0f} mm bed. Split it, or make it metal")
    if worst is not None:
        rep.add('printable', 'largest part', True, f"{n} printed parts; largest {worst[1]}: {worst[0][-1] * 1000:.0f} mm, on a {BED * 1000:.0f} mm bed")


def check_axis(spec, rep):
    for j in spec['joints']:
        if not j['act'] or j['remote']:
            continue
        a, o, c = G.AXIS[j['axis']], np.array(j['origin']), np.array(j['act_c'])
        d = c - o
        off = np.linalg.norm(d - (d @ a) * a)
        rep.add('axis', j['name'], off < 1e-4, f"{j['act']} {off * 1000:.1f} mm off the axis")


def check_bearings(spec, rep):
    """Every bearing centred on its joint's axis, with its two races on the two links that joint moves apart."""
    J = {j['name']: j for j in spec['joints']}
    races = {}
    for ln, l in spec['links'].items():
        for g in l['geoms']:
            if 'on_joint' not in g:
                continue
            j = J[g['on_joint']]
            a, o, c = G.AXIS[j['axis']], np.array(j['origin']), np.array(g['c'])
            off = np.linalg.norm((c - o) - ((c - o) @ a) * a)
            rep.add('bearing', g['name'], off < 1e-4 and g['axis'] == j['axis'] and ln in (j['parent'], j['child']),
                    f'{off * 1000:.1f} mm off the axis, on {ln}')
            side = g['name'].split(' bearing ')[0]
            races.setdefault((j['name'], side), []).append(ln)
    for k, ls in races.items():
        rep.add('bearing', f'{k[1]} races', len(ls) == 2 and ls[0] != ls[1], f'on {ls}')


def check_static(spec, S, R, rep):
    links = list(spec['links'])
    bad = 0
    # rigid vs rigid, every pair of links (adjacent too), except each joint's actuator/flange interface
    allowed = set()
    for j in spec['joints']:
        if j['act']:
            names = sorted(interface(j))
            allowed |= {frozenset((x, y)) for x in names for y in names if x != y}
    rigid = [(ln, i, g) for ln in links for i, g in enumerate(spec['links'][ln]['geoms']) if g['role'] in G.RIGID]
    for x in range(len(rigid)):
        for y in range(x + 1, len(rigid)):
            (la, ia, ga), (lb, ib, gb) = rigid[x], rigid[y]
            if la == lb or frozenset((ga['name'], gb['name'])) in allowed:
                continue
            v = overlap(S.geom[(la, ia)], S.geom[(lb, ib)])
            if v > TOUCH:
                bad += 1
                rep.add('static', f"{ga['name']} / {gb['name']}", False, f'rigid parts overlap {v * 1e6:.1f} cm3')
    # a printed part through a servo of its own link (2026-11-02: a fork's post could run through the next servo
    # down unseen, as the check above skips parts of one link)
    for ln in links:
        gs = [(i, g) for i, g in enumerate(spec['links'][ln]['geoms']) if g['role'] in G.RIGID]
        for ia, ga in gs:
            if ga['role'] != 'actuator':
                continue
            for ib, gb in gs:
                if gb['role'] != 'struct' or gb.get('seat') or frozenset((ga['name'], gb['name'])) in allowed:
                    continue
                v = overlap(S.geom[(ln, ia)], S.geom[(ln, ib)])
                if v > SEATED:
                    bad += 1
                    rep.add('static', f"{ga['name']} / {gb['name']}", False, f'a part runs through its own link\'s servo '
                            f'{v * 1e6:.1f} cm3')
    # skin vs anything of a non-adjacent link
    for x in range(len(links)):
        for y in range(x + 1, len(links)):
            a, b = links[x], links[y]
            if R.adjacent(a, b):
                continue
            v = overlap(S.hard[a], S.hard[b])
            if v > TOUCH:
                bad += 1
                rep.add('static', f'{a} / {b}', False, f'non-adjacent links touch (skin wall or rigid) {v * 1e6:.1f} cm3')
    # a decal (a painted patch, an eye) over another link's rigid parts or covers: decals were in no collision check, and the
    # reindeer's bib sat over both front thighs and their servos until Fusion's interference check showed it (2026-11-05)
    for ln in links:
        for i, g in enumerate(spec['links'][ln]['geoms']):
            if g['role'] != 'decal' or S.geom.get((ln, i)) is None:
                continue
            for other in links:
                if other == ln:
                    continue
                v = overlap(S.geom[(ln, i)], S.hard[other])
                if v > TOUCH:
                    bad += 1
                    rep.add('static', f"{g['name']} / {other}", False, f'a decal over another link: {v * 1e6:.2f} cm3')
    rep.add('static', 'zero pose', bad == 0, f'{bad} overlaps')


def check_covers(spec, S, R, rep):
    """A cover must not pass through a motor of its own link or of a link next to it. The skin tested here is
    closed on the cover's flat cut faces (Fusion draws them as solid faces); the collision walls leave those faces
    open, so a hip lobe's flat inner face sliced through both hip motors unseen (user, 2026-10-31)."""
    acts = [(ln, i, g) for ln, l in spec['links'].items() for i, g in enumerate(l['geoms']) if g['role'] == 'actuator']
    bad = 0
    for ln, l in spec['links'].items():
        for i, g in enumerate(l['geoms']):
            if g['role'] != 'shell' or S.geom.get((ln, i)) is None:
                continue
            skin = S.geom[(ln, i)] - to_manifold(G.mesh_of(G.inner_closed(g, WALL)))
            for o in S.merged.get((ln, i), []):
                skin = skin - S.geom[o]
            for la, ia, ga in acts:
                if la != ln and not R.adjacent(la, ln):
                    continue
                v = overlap(skin, S.geom[(la, ia)])
                if v > TOUCH:
                    bad += 1
                    rep.add('covers', f"{g['name']} / {ga['name']}", False,
                            f'the cover passes through the motor: {v * 1e6:.1f} cm3')
    rep.add('covers', 'motors clear of the covers', bad == 0, f'{bad} cover / motor crossings')


def check_connect(spec, S, rep):
    """Rigid parts of a link must be one structure on their own (covers carry no load: a missing bracket hidden
    under a cover is still missing, 2026-10-31 calibration), and every cover must be mounted on it."""
    # pieces under 0.01 cm3 are slivers the boolean union leaves where cut pieces meet, not parts (2026-10-31: a
    # zero-volume sliver where the right shoe's toe and heel pieces overlap)
    # (a negative volume is a closed void between parts, not a separate piece)
    pieces = lambda m: len([p for p in m.decompose() if p.volume() > 1e-8])
    for ln in spec['links']:
        if S.rigid[ln] is not None:
            n = pieces(S.rigid[ln])
            rep.add('connect', f'{ln} structure', n == 1,
                    f'rigid parts form {n} separate pieces (a bracket or frame is missing)' if n != 1 else '')
        n = pieces(S.all[ln])
        rep.add('connect', f'{ln} covers mounted', n == 1, f'{n} separate pieces' if n != 1 else '')
    t = np.linspace(-0.18, 0.18, 361)
    for j in spec['joints']:
        a, o = G.AXIS[j['axis']], np.array(j['origin'])
        line = o + np.outer(t, a)
        hit = {}
        for side in ('parent', 'child'):
            gs = spec['links'][j[side]]['geoms']
            hit[side] = any(G.inside(g, line).any() for g in gs if g['role'] in G.RIGID) or \
                any(G.inside(g, line).any() for g in gs)
        rigid_hit = {side: any(G.inside(g, line).any() for g in spec['links'][j[side]]['geoms'] if g['role'] in G.RIGID)
                     for side in ('parent', 'child')}
        ok = rigid_hit['parent'] and rigid_hit['child']
        miss = [s for s in ('parent', 'child') if not rigid_hit[s]]
        rep.add('connect', f"{j['name']} axis", ok,
                '' if ok else f"axis passes through no rigid part of the {' and '.join(miss)} "
                              f"({', '.join(j[s] for s in miss)})")


def check_skin(spec, R, rep):
    """Internal parts must be hidden: inside the skins (any link, zero pose), unless meant to show."""
    rng = np.random.default_rng(1)
    shells = [g for l in spec['links'].values() for g in l['geoms'] if g['role'] == 'shell']
    cuts = {}                                             # shell name -> the cuts through it (they uncover)
    for l in spec['links'].values():
        for c in l['geoms']:
            for t in c.get('targets', []) if c['role'] == 'cut' else []:
                cuts.setdefault(t, []).append(c)
    exposed = exposed_names(spec)
    for ln, l in spec['links'].items():
        for g in l['geoms']:
            if g['role'] not in G.RIGID or g['name'] in exposed:
                continue
            # 0.5 %: a 5 mm corner of an actuator through the calf (0.3 %) showed as dark marks in Fusion
            # (2026-10-31); the first limit, 5 %, let it through
            p = G.volume_samples(g, 4000, rng)
            inn = np.zeros(len(p), bool)
            for s in shells:
                hit = G.inside(s, p)
                for c in cuts.get(s['name'], []):
                    hit &= ~G.inside(c, p)
                inn |= hit
            frac = 1 - inn.mean()
            rep.add('skin', g['name'], frac <= 0.005, f'{frac * 100:.2f} % outside the skin')


NEAR = 3                  # links at most this many joints apart: a collision between them is structural


def culprit(spec, S, a, Ma, b, Mb):
    """The pair of geoms (one from link a, one from link b) that overlap most, by name."""
    best = (0, '')
    hard = lambda ln, i: S.wall[(ln, i)] if (ln, i) in S.wall else S.geom[(ln, i)]
    for i, ga in enumerate(spec['links'][a]['geoms']):
        if hard(a, i) is None:                            # a cut, not a part
            continue
        ma = posed(hard(a, i), Ma)
        for k, gb in enumerate(spec['links'][b]['geoms']):
            if hard(b, k) is None:
                continue
            v = overlap(ma, posed(hard(b, k), Mb))
            if v > best[0]:
                best = (v, f"{ga['name']} / {gb['name']}")
    return best[1]


def check_sweep(spec, S, R, rep, steps=25):
    """Sweep each joint alone from the zero pose. Collisions between links near each other in the tree are
    structural (FAIL: the limit cannot be reached in any pose). Collisions with far links (a hanging arm meeting
    the hip as the waist turns) depend on the pose: WARN, and left to self-collision avoidance."""
    for j in spec['joints']:
        if j['name'].startswith('R_'):
            continue                                  # mirror of the left side
        moving = set(R.subtree(j['child']))
        static = [l for l in spec['links'] if l not in moving]
        pairs = [(a, b) for a in moving for b in static if not R.adjacent(a, b)]
        # across the joint itself (parent vs child): rigid parts AND the covers' walls, minus only the actuator /
        # flange interface (user, 2026-10-31: "move every joint to min and max, the body must not come in the
        # way"; until then the covers either side of a joint were allowed to overlap, as fabric)
        P, C = spec['links'][j['parent']]['geoms'], spec['links'][j['child']]['geoms']
        ii = interface(j) if j['act'] else set()
        hard = lambda ln, i: S.wall.get((ln, i)) if (ln, i) in S.wall else \
            (S.geom[(ln, i)] if spec['links'][ln]['geoms'][i]['role'] in G.RIGID else None)
        rig = [(hard(j['parent'], x), hard(j['child'], y), P[x]['name'], C[y]['name'])
               for x in range(len(P)) for y in range(len(C))
               if hard(j['parent'], x) is not None and hard(j['child'], y) is not None
               and not {P[x]['name'], C[y]['name']} <= ii]   # an interface may name more than two parts (a rear horn): reindeer, 2026-11-05
        def hits(qv):
            """[(volume, description, structural)] at angle qv, worst first."""
            M = R.fk({j['name']: qv})
            out = []
            for a, b in pairs:
                v = overlap(posed(S.hard[a], M[a]), S.hard[b])
                if v > TOUCH:
                    out.append((v, f'{a} / {b}: {culprit(spec, S, a, M[a], b, M[b])}',
                                R.distance(a, b) <= NEAR))
            for ma, mb, na, nb in rig:
                v = overlap(ma, posed(mb, M[j['child']]))
                if v > TOUCH:
                    out.append((v, f'{na} / {nb} (across the joint)', True))
            return sorted(out, key=lambda h: (not h[2], -h[0]))
        msgs, warns, ok = [], [], True
        for side, end in (('lower', j['lower']), ('upper', j['upper'])):
            far_seen = False
            for qv in np.linspace(0, end, steps):
                h = hits(qv)
                near = [x for x in h if x[2]]
                if near:
                    ok = False
                    msgs.append(f"{side} {end:+.2f}: blocked at {qv:+.2f} rad by {near[0][1]} "
                                f"({near[0][0] * 1e6:.0f} cm3)" if qv else f'collides at 0: {near[0][1]}')
                    break
                if h and not far_seen:
                    far_seen = True
                    warns.append(f"{side}: {h[0][1]} at {qv:+.2f} rad (pose-dependent)")
        if ok:
            rep.add('sweep', j['name'], not warns, f"[{j['lower']:+.2f}, {j['upper']:+.2f}] rad clear"
                    + ('; ' + '; '.join(warns) if warns else ''), warn=True)
        else:
            rep.add('sweep', j['name'], False, '; '.join(msgs + warns))


# ---------------------------------------------------------------- statics

POSES = {
    # name: (angles (left side; mirrored unless the right is given), category). The body is lowered onto its hooves.
    'stand': ({}, 'continuous'),
    'crouch': ({'L_front_hip': 0.5, 'L_front_knee': -1.0, 'L_rear_hip': 0.5, 'L_rear_knee': -1.0}, 'continuous'),
    'nod down': ({'neck_pitch': 0.18}, 'continuous'),
    'look up': ({'neck_pitch': -0.28}, 'continuous'),
    'head tilt, looking round': ({'head_roll': 0.30, 'head_yaw': 1.0}, 'continuous'),
    'chatting: mouth open, antlers back': ({'jaw': 0.45, 'antlers': -0.15, 'head_roll': 0.2}, 'continuous'),
    'paw lifted (three legs)': ({'L_front_hip': -0.5, 'L_front_knee': 1.2, 'R_front_hip': 0.0, 'R_front_knee': 0.0}, 'peak'),
    'trot, diagonal pair down': ({'L_front_hip': -0.5, 'L_front_knee': 0.9, 'R_front_hip': 0.2, 'R_front_knee': -0.2,
                                  'R_rear_hip': -0.5, 'R_rear_knee': 0.9, 'L_rear_hip': 0.2, 'L_rear_knee': -0.2}, 'peak'),
}
PEAK_MARGIN = 1.5        # peak torque must exceed static torque x this (accelerations, impacts)
FLOOR = 0.002            # a point this close to the floor carries weight (shells and rigid parts: the base plate is what it sits on;
#                          at 1.5 mm and shells only, the body's bottom at exactly 1.5 mm was left out and only the hooves carried it: reindeer round 4)


class Mass:
    """Link masses and centres of mass from the parts' materials (tools/mass_model.py)."""

    def __init__(self, spec, S):
        self.com, self.m = {}, {}
        for ln, (m, com, I, ps) in MM.link_inertials(spec, S).items():
            self.com[ln] = com
            self.m[ln] = m


def statics(spec, R, MS, q, S):
    """The body lowered until its lowest point meets the floor; what lies within FLOOR of it is the support. Joint torques
    (N m) holding the pose still: a joint in a leg on the floor carries that hoof's share of the weight (shared equally
    between the hooves down), any other joint holds everything beyond it."""
    M = R.fk(q)
    pts = {}
    for ln in spec['links']:
        if S.all[ln] is not None:
            pts[ln] = G.apply(M[ln], np.asarray(S.all[ln].to_mesh().vert_properties[:, :3], float))
    drop = min(v[:, 2].min() for v in pts.values())
    base = np.eye(4)
    base[2, 3] = -drop
    M = {ln: base @ m for ln, m in M.items()}
    loads = [(MS.m[l], G.apply(M[l], MS.com[l][None])[0], l) for l in spec['links']]
    mt = sum(x[0] for x in loads)
    com = sum(x[0] * x[1] for x in loads) / mt
    down = {ln: v[v[:, 2] - drop < FLOOR] for ln, v in pts.items()}
    down = {ln: v for ln, v in down.items() if len(v)}
    feet = [ln for ln in down if ln != spec['root']]
    gvec = np.array([0, 0, -G_ACC])
    tau = {}
    for j in spec['joints']:
        a = M[j['parent']][:3, :3] @ G.AXIS[j['axis']]
        o = G.apply(M[j['parent']], np.array(j['origin'])[None])[0]
        sub = set(R.subtree(j['child']))
        stance = [f for f in feet if f in sub]
        t = sum(m * np.cross(c - o, gvec) @ a for m, c, l in loads if l in sub)
        if stance and spec['root'] not in down:
            t += sum(np.cross(np.r_[down[f][:, :2].mean(0), 0.0] - o, -gvec * mt / len(feet)) @ a for f in stance)
        tau[j['name']] = abs(t)
    poly = np.vstack([v[:, :2] for v in down.values()])
    return tau, com, poly, M


def hull_margin(poly, p):
    """Signed distance from p to the edge of the convex hull of poly (positive inside)."""
    from scipy.spatial import ConvexHull
    h = ConvexHull(poly)
    return min(-(eq[:2] @ p + eq[2]) for eq in h.equations)


def capacity(spec, j):
    c = j['capacity']
    if not c:
        return None
    a = spec['actuators'][c['model']]
    k = c['count'] * c['ratio']
    return a['rated_nm'] * k, a['peak_nm'] * k


def check_poses(spec, S, R, rep, collide=True):
    MS = Mass(spec, S)
    worst = {}
    for name, (q, cat) in POSES.items():
        q = G.mirror_pose(q)
        tau, com, poly, M = statics(spec, R, MS, q, S)
        if name.startswith('trot'):
            # two hooves down: a line, not an area. A trot is dynamic: reported, as the duck's single-leg stance is
            rep.add('poses', f'{name}: balance', False, 'two hooves down: dynamic balance, not static', warn=True)
        else:
            margin = hull_margin(poly, com[:2]) if len(poly) >= 3 else -1.0
            # 10 mm: a desk toy is nudged, and its moves accelerate it (a dynamic margin, not proven: tools/sim_stand.py)
            rep.add('poses', f'{name}: balance', margin >= 0.010, f"CoM {margin * 1000:.0f} mm inside what touches the floor")
        out = [f"{k} {v:+.2f} outside [{R.joints[k]['lower']:+.2f}, {R.joints[k]['upper']:+.2f}]"
               for k, v in q.items() if not R.joints[k]['lower'] - 1e-9 <= v <= R.joints[k]['upper'] + 1e-9]
        rep.add('poses', f'{name}: within limits', not out, '; '.join(out) or 'yes')
        for j in spec['joints']:
            cap = capacity(spec, j)
            brief = cat == 'peak'
            need = tau[j['name']] * (PEAK_MARGIN if brief else 1.0)
            have = cap[1] if brief else cap[0]
            r = need / have
            w = worst.get(j['name'])
            if w is None or r > w[0]:
                worst[j['name']] = (r, name, tau[j['name']], have, cat)
        if collide:
            hits = []
            links = list(spec['links'])
            for x in range(len(links)):
                for y in range(x + 1, len(links)):
                    a, b = links[x], links[y]
                    if R.adjacent(a, b):
                        continue
                    v = overlap(posed(S.hard[a], M[a]), posed(S.hard[b], M[b]))
                    if v > TOUCH:
                        hits.append(f'{a} / {b} {v * 1e6:.2f} cm3')
            rep.add('poses', f'{name}: collisions', not hits, '; '.join(hits[:4]) or 'none')
    for jn, (r, pose, t, have, cat) in sorted(worst.items(), key=lambda kv: -kv[1][0]):
        j = next(x for x in spec['joints'] if x['name'] == jn)
        c = j['capacity']
        kind = 'peak' if cat == 'peak' else 'rated'
        fac = f' x {PEAK_MARGIN}' if cat == 'peak' else ''
        rep.add('torque', jn, r <= 1.0,
                f"{t:.2f} N m{fac} in '{pose}' vs {kind} {have:.2f} N m "
                f"({c['count']} x {c['model']}{' x %.1f' % c['ratio'] if c['ratio'] != 1 else ''}): "
                f"{r * 100:.0f} % used")


# ---------------------------------------------------------------- deliberate breakage

def break_spec(spec, case):
    s = copy.deepcopy(spec)
    J = {j['name']: j for j in s['joints']}
    find = lambda ln, name: next(g for g in s['links'][ln]['geoms'] if g['name'] == name)
    if case == 'torque':                                  # the knees and neck on a servo a twentieth as strong
        for n in ('L_front_knee', 'R_front_knee', 'L_rear_knee', 'R_rear_knee', 'neck_pitch'):
            J[n]['capacity'] = dict(model='XL330_M288', count=1, ratio=0.05)
    elif case == 'collide':                               # the left front shin pushed 6 mm in, into the right one
        for g in s['links']['L_front_shin']['geoms']:
            for k in ('c', 'p0', 'p1'):
                if k in g:
                    g[k][1] -= 0.006
            for sec in g.get('sections', []):
                sec['cy'] -= 0.006
    elif case == 'offaxis':
        J['head_yaw']['act_c'][0] += 0.01
    elif case == 'floating':                              # the neck's bridge deleted: the yaw servo stands on nothing
        s['links']['neck']['geoms'] = [g for g in s['links']['neck']['geoms'] if g['name'] != 'neck bridge']
    elif case == 'mass':
        find('body', 'battery 2S, at most 44 x 30 x 8 mm (buy)')['mass'] += 0.5
    elif case == 'poke':                                  # the battery pushed up through the body's back (not the Pi: it is marked
        find('body', 'battery 2S, at most 44 x 30 x 8 mm (buy)')['c'][2] += 0.015   # as seen from below, so the skin check skips it)
    elif case == 'cover':                                 # the left front thigh shrunk round its servo
        for sec in find('L_front_thigh', 'L front thigh')['sections']:
            sec['a'] *= 0.7
    elif case == 'tip':                                   # 300 g hung 15 cm in front of the chest: it must tip
        g = find('body', 'battery 2S, at most 44 x 30 x 8 mm (buy)')
        g['mass'] = 0.30
        g['c'][0] += 0.15
    elif case == 'decal':                                 # the bib moved back 8 mm, over the front thighs, and left uncut (round 7's miss)
        find('body', 'bib')['c'][0] -= 0.008
        for g in s['links']['body']['geoms']:
            if g['role'] == 'cut' and 'bib' in g.get('targets', []):
                g['targets'] = [t for t in g['targets'] if t != 'bib']
    elif case == 'oversize':
        s['links']['body']['geoms'].append(dict(kind='rbox', name='oversize', c=[0.0, 0.0, 0.5], size=[BED + 0.05, 0.02, 0.02], fillet=0.002, color='#FFFFFF', role='struct', exposed=True, material='pla'))
    else:
        raise SystemExit(f'unknown case {case}')
    return s


EXPECT = {'torque': 'torque', 'collide': 'static', 'offaxis': 'axis', 'floating': 'connect', 'mass': 'mass',
          'cover': 'covers', 'poke': 'skin', 'tip': 'poses', 'oversize': 'printable', 'decal': 'static'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', nargs='*')
    ap.add_argument('--bad')
    ap.add_argument('--spec')
    ap.add_argument('--out', default=G.out_path('reports', 'checks.md'))
    a = ap.parse_args()
    spec = G.load(a.spec)
    if a.bad:
        spec = break_spec(spec, a.bad)
        a.only = a.only or [EXPECT[a.bad]]
    groups = a.only or ['targets', 'mass', 'printable', 'axis', 'static', 'covers', 'connect', 'skin', 'sweep', 'poses']
    R = G.Robot(spec)
    t0 = time.time()
    S = Solids(spec)
    rep = Report()
    for grp in groups:
        t = time.time()
        {'targets': lambda: check_targets(spec, S, rep),
         'mass': lambda: check_mass(spec, S, rep),
         'printable': lambda: check_printable(spec, S, rep),
         'axis': lambda: check_axis(spec, rep),
         'bearing': lambda: check_bearings(spec, rep),
         'static': lambda: check_static(spec, S, R, rep),
         'covers': lambda: check_covers(spec, S, R, rep),
         'connect': lambda: check_connect(spec, S, rep),
         'skin': lambda: check_skin(spec, R, rep),
         'sweep': lambda: check_sweep(spec, S, R, rep),
         'poses': lambda: check_poses(spec, S, R, rep),
         'torque': lambda: check_poses(spec, S, R, rep, collide=False)}[grp]()
        print(f'{grp}: {time.time() - t:.1f} s', file=sys.stderr)
    fails = rep.failing()
    if a.bad:
        caught = [r for r in fails if r['group'] == EXPECT[a.bad]]
        print(f"--bad {a.bad}: {'CAUGHT' if caught else 'NOT CAUGHT'} ({len(caught)} failing rows)")
        for r in caught[:5]:
            print('  ', r['group'], r['item'], r['detail'])
        return 0 if caught else 1
    rep.write(a.out, f"spec: `spec/robot.json`, {time.strftime('%Y-%m-%d %H:%M')}, {time.time() - t0:.0f} s")
    for r in rep.rows:
        if r['status'] != 'PASS':
            print(f"{r['status']} {r['group']:8} {r['item']}: {r['detail']}")
    print(f"{len(rep.rows)} checks, {len(fails)} failing -> {a.out}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
