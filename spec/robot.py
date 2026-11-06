"""reindeer: a sitting desk droid with a big head, after the user's plush reindeer video. The one source of truth for
kinematics, servos, masses and outer shape.

  python -I spec/robot.py   ->  spec/robot.json (the checks, renders, simulation and Fusion build all read it)

Never hand-edit robot.json or the Fusion document: change this file and rebuild.

Frame: metres, X forward, Y left, Z up, floor at z = 0. Zero pose: sitting, head level, paws down, tail centred.
Geometry is in WORLD coordinates at the zero pose; each link's frame is its parent joint's origin with world axes.
The left side is written once and mirrored (y -> -y) for L_ links and 'L ' geoms.

The look (2026-11-05, the reference video, spec/targets.json): a sitting fawn-brown reindeer, a head as wide as its
haunches, huge green eyes with white rims, a white muzzle with a dark-green nose, green-lined ears sticking out
sideways, one red and one green antler, a white bib, hooves alternating red and green, a white-tipped tail. Its own
geometry: superellipse lofts and ellipsoids from the measured fractions, no mesh copied.

Moves (the point of a toy droid; the reference itself never moves): nod, look round, tilt the head (the cute one),
lift and wave either front paw, wag the tail. Six ROBOTIS XL330-M288 servos on one Dynamixel bus.

Roles (as duck-droids', from echo):
  shell     outer cover, printed PLA, 2 mm wall
  decal     surface detail (eyes, nose, inner ears): must touch its link
  struct    rigid internal part (printed bracket, boards, battery): never inside another link's rigid parts
  actuator  the bought servo case, in the link that carries it, its shaft on the joint axis
  cut       removed from the named geoms of its link
"""
import copy
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ACT = {k: v for k, v in json.load(open(os.path.join(HERE, 'actuators.json'), encoding='utf-8')).items()
       if not k.startswith('_')}
XL = ACT['XL330_M288']

# colours from the video's frames (sampled by eye; matte PLA in the nearest stock colours)
FAWN, WHITE, RED, GREEN, DARKGREEN = '#B87333', '#F3EFE6', '#C42630', '#1F8A3C', '#17542E'
EYEGREEN, PUPIL, BLACK, DARK, PCB, CELL, STEEL = '#1BA05A', '#08090B', '#1B1C1F', '#3A3D42', '#1F5A36', '#2F7DE1', '#C9CDD2'

LINKS, JOINTS = {}, []
MATERIALS = {
    'pla': dict(density=1240, note='PLA (FDM): shells and brackets; brackets with a fill factor'),
    'pla_skin': dict(density=1240, layer=0.002, note='PLA printed as a 2 mm layer: the bib, the white patches'),
    'electronics': dict(density=1500, note='computer, bus board, regulator: explicit masses'),
    'cells': dict(density=2500, note='battery: explicit mass'),
    'steel': dict(density=7800, note='screws'),
}


# ---------------------------------------------------------------- primitives (as duck-droids' spec/robot.py)

def cyl(name, c, axis, r, h, color=DARK, role='struct', **extra):
    return dict(kind='cyl', name=name, c=list(c), axis=axis, r=r, h=h, color=color, role=role, **extra)


def ell(name, c, r, color=FAWN, role='shell', **extra):
    return dict(kind='ellipsoid', name=name, c=list(c), r=list(r), color=color, role=role, **extra)


def rbox(name, c, size, fillet=0.0005, color=DARK, role='struct', **extra):
    return dict(kind='rbox', name=name, c=list(c), size=list(size), fillet=fillet, color=color, role=role, **extra)


def box(name, lo, hi, color=DARK, role='struct', **extra):
    c = [(a + b) / 2 for a, b in zip(lo, hi)]
    s = [b - a for a, b in zip(lo, hi)]
    return rbox(name, c, s, min(0.0005, min(s) / 2 - 1e-4), color, role, **extra)


def cap(name, p0, p1, r, color=FAWN, role='shell', **extra):
    return dict(kind='capsule', name=name, p0=list(p0), p1=list(p1), r=r, color=color, role=role, **extra)


def loft(name, sections, color=FAWN, role='shell', open=(), clip=None, along='z', **extra):
    """sections (z, cx, cy, a, b, n): superellipse |dx/a|^n + |dy/b|^n = 1 at height z. clip: (point, normal) or a
    list of them: keep what is in front of every plane."""
    g = dict(kind='loft', name=name, sections=sorted([dict(z=z, cx=cx, cy=cy, a=a, b=b, n=n)
                                                      for z, cx, cy, a, b, n in sections], key=lambda s: s['z']),
             color=color, role=role, open=list(open), **extra)
    if along == 'x':            # sections (x, cz, cy, a (half height), b (half width), n), stacked back to front
        g['along'] = 'x'
    if clip:
        planes = [dict(p=list(c[0]), n=list(c[1])) for c in (clip if isinstance(clip, list) else [clip])]
        g['clip'] = planes if isinstance(clip, list) else planes[0]
    return g


def revolve(name, axis, c, profile, color=FAWN, role='shell', **extra):
    return dict(kind='revolve', name=name, axis=axis, c=list(c), profile=[list(p) for p in profile], color=color,
                role=role, **extra)


def plate(name, y0, t, circles, color=DARK, role='struct', n=24, axis='y', **extra):
    """A flat plate: the convex hull of circles ((u, v, r), ...) in the plane square to `axis`, from y0 to y0 + t
    along it (axis 'y': circles (x, z); 'x': (y, z); 'z': (x, y))."""
    pts = sorted({(round(u + r * math.cos(2 * math.pi * k / n), 7), round(v + r * math.sin(2 * math.pi * k / n), 7))
                  for u, v, r in circles for k in range(n)})
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    if axis != 'y':
        extra['axis'] = axis
    return dict(kind='plate', name=name, y0=y0, t=t, pts=[list(p) for p in hull], color=color, role=role, **extra)


def cut(g, *targets):
    return dict(g, role='cut', targets=list(targets))


def above_z(z):
    return (0.0, 0.0, z), (0.0, 0.0, 1.0)


def link(name, budget, geoms, note=''):
    LINKS[name] = dict(name=name, budget=budget, geoms=list(geoms), note=note)


AX = {'x': 0, 'y': 1, 'z': 2}


def joint(name, parent, child, origin, axis, lim, out=+1, long='-z', case_in='parent', act='XL330_M288', exposed=False):
    """A servo joint. origin: the centre of the horn's outer face, on the axis. The case sits behind the horn (against
    `out`, the way the horn faces, +1 / -1 along the axis), its long side running `long` from the shaft. The case is in
    `case_in` (the parent, or the child when the servo rides in the moving link, as the front legs'), the horn in the
    other. Returns the geoms: (case, horn, rear horn), each to be put in its link by the caller via place()."""
    a = ACT[act]
    L, W, H = a['box']
    k = AX[axis]
    shaft = list(origin)
    shaft[k] -= out * (a['horn_h'] + H / 2)                # the shaft at the case's mid-depth: tools/checks.py 'axis'
    c = list(shaft)
    c[AX[long[1]]] += (1 if long[0] == '+' else -1) * a['shaft_x']
    size = [W, W, W]
    size[k], size[AX[long[1]]] = H, L
    j = dict(name=name, parent=parent, child=child, origin=list(origin), axis=axis, lower=lim[0], upper=lim[1],
             act=act, act_c=shaft, out=out, long=long, case_in=case_in, remote=False, exposed=exposed,
             capacity=dict(model=act, count=1, ratio=1.0), interface=[f'{name} rear horn (buy) FPX330-H101'])
    JOINTS.append(j)
    case = rbox(f'{name} actuator (buy) {act}', c, size, 0.0003, BLACK, 'actuator')
    hc = list(origin)
    hc[k] -= out * a['horn_h'] / 2
    horn = cyl(f'{name} flange', hc, axis, a['horn_r'], a['horn_h'] + 0.0005, '#9EA3A8', 'struct')   # 0.5 mm into its plate
    rc = list(shaft)
    rc[k] -= out * (H / 2 + a['horn_h'] / 2)
    rear = cyl(f'{name} rear horn (buy) FPX330-H101', rc, axis, a['horn_r'], a['horn_h'] + 0.0005, '#9EA3A8', 'struct')
    return case, horn, rear


# ================================================================ proportions
# Round 5 (2026-11-05, the user's second video, reindeer_walking.mp4: the same plush stands up and WALKS on four legs):
# a standing quadruped. Fractions of the height H (floor to antler tip) from its 5.0 s frame (near side, 3/4) and 0.0 s
# (front): belly 0.19 H, back 0.45 H, body 0.50 H long, chin 0.36 H, eyes 0.56 H, crown 0.78 H; head as wide as before.
# Rounds 1-4 were a sitting droid from the first video (kept in SKILL_TEST.md); its head carries over, scaled 1.15.
H = 0.24               # floor to antler tip: legs of two XL330s each need 70 mm to the hip (thigh 42, shin 34 + hoof)
ZH = 0.070             # hip axes (y): the thigh's servo bottom must clear the shin's servo top, which turns on a 13.8 mm
ZK = 0.028             # circle round the knee: thigh 42 mm. Knee axes (y): the shin's servo hangs below, 3.5 mm off the floor
XF, XR = 0.040, -0.040 # front and rear hips, 80 mm apart
KEEL = 0.0025          # half the keel's thickness: all four hip horns screw to its faces
LY = (0.0030, 0.0310)  # each leg's sides: its inner wall on the hip servo's horn face, 0.5 mm off the keel
NX = 0.012             # neck pitch axis x: its case lies back from it, 2.7 mm behind the front thighs' round tops
ZNP = 0.088            # neck pitch axis z: low in the chest's top, under the head
ZYAW = 0.1365          # head yaw horn's top face: the yaw case on the neck bracket's bridge (0.107-0.1105)
ZROLL = 0.150          # head roll axis: the roll case across the head on the yaw plate
XROLL = NX + 0.016     # the roll horn's front face
K = 1.15               # the head over the sitting design's (the standing plush's head reads 0.44 H wide)


# ================================================================ body (the root): a horizontal loft, nose to tail along x
SB = [  # (x, cz, cy, a (half height), b (half width), n): back 0.45 H at the rump, sloping to the neck; belly 0.24 H (the
    #   reference's 0.19: two XL330s a leg set the hip at 70 mm)
    (-0.066, 0.084, 0.0, 0.010, 0.015, 2.2),
    (-0.058, 0.0825, 0.0, 0.0240, 0.033, 2.4),
    (-0.040, 0.0830, 0.0, 0.0260, 0.0420, 2.3),   # round 8: fuller and rounder (2.6 read as a tube)
    (-0.015, 0.0810, 0.0, 0.0240, 0.0425, 2.3),
    (0.000, 0.0790, 0.0, 0.0220, 0.0425, 2.3),
    (0.025, 0.0755, 0.0, 0.0185, 0.0420, 2.3),
    (0.045, 0.0730, 0.0, 0.0160, 0.0395, 2.2),   # round 6: the chest's top lower in front, so the muzzle can nod
    (0.058, 0.0715, 0.0, 0.0125, 0.0310, 2.1),
    (0.066, 0.0710, 0.0, 0.0065, 0.0170, 2.0),
]

body = [
    loft('body', SB, FAWN, along='x', load_bearing=True),   # the shell carries each hip's outer side (its boss): it is structure
    cut(box('body neck opening', (NX - 0.030, -0.0195, 0.080), (NX + 0.024, 0.0195, 0.13)), 'body'),
    ell('bib', (0.0625, 0.0, 0.0745), (0.0060, 0.0170, 0.0120), WHITE, 'decal', material='pla_skin'),   # round 8: one clean oval on the chest
    # the tail, perky up at the rump (fixed: no room for a servo in the rump, round 5)
    ell('tail', (-0.068, 0.0, 0.104), (0.009, 0.008, 0.013), FAWN),
    ell('tail tip', (-0.072, 0.0, 0.113), (0.006, 0.006, 0.007), WHITE, 'decal'),
    # structure: the keel, front to back, carries the four hip horns, the neck's cradle and the electronics
    # (round 5: a keel down past the hips met the knees' inner plates as the legs swung: above them, and a disc round each horn)
    box('keel', (XR - 0.012, -KEEL, ZH - 0.0035), (XF + 0.012, KEEL, 0.0765), DARK, exposed=True),
    cyl('keel front disc', (XF, 0, ZH), 'y', 0.0105, 2 * KEEL, DARK, exposed=True),
    cyl('keel rear disc', (XR, 0, ZH), 'y', 0.0105, 2 * KEEL, DARK, exposed=True),
    box('neck cradle floor', (NX - 0.0255, -0.0115, 0.0755), (NX + 0.0010, 0.0115, 0.0780), DARK, exposed=True),
    box('neck cradle back', (NX - 0.0275, -0.0115, 0.0755), (NX - 0.0245, 0.0115, 0.088), DARK, exposed=True),
    box('battery post', (-0.020, -KEEL, 0.0760), (-0.016, KEEL, 0.0908), DARK, exposed=True),
    box('battery tray', (-0.052, -0.023, 0.0898), (-0.016, 0.023, 0.0913), DARK, exposed=True),
    box('Pi Zero 2 W (buy)', (-0.015, -0.0325, 0.0605), (0.015, 0.0325, 0.0666), PCB, material='electronics', mass=0.011, exposed=True),   # under the keel, on standoffs
    box('battery 2S, at most 44 x 30 x 8 mm (buy)', (-0.050, -0.022, 0.0912), (-0.020, 0.022, 0.0992), CELL, material='cells', mass=0.020),
    box('bus board (buy)', (-0.050, -0.021, 0.0991), (-0.020, 0.021, 0.1023), PCB, material='electronics', mass=0.008),   # size to confirm
]
case, horn, rear = joint('neck_pitch', 'body', 'neck', (NX, 0.0145, ZNP), 'y', (-0.28, 0.18), out=+1, long='-x', exposed=True)   # 0.18: the jaw's chin clear of the bib   # round 6: -0.35 put the yaw servo in the back; past 0.27 the muzzle meets the bib
body.append(case)
neck = [horn, rear]
link('body', 0.25, body)

# ================================================================ neck: a U round the pitch servo, the yaw servo on its bridge
BRIDGE = (0.107, 0.1105)        # clear of the pitch case's far corners (26.5 mm out) turned 0.35 rad up
neck += [
    box('neck horn plate', (NX - 0.010, 0.0145, ZNP - 0.010), (NX + 0.010, 0.0175, BRIDGE[1]), FAWN, exposed=True),
    box('neck rear plate', (NX - 0.010, -0.0175, ZNP - 0.010), (NX + 0.010, -0.0145, BRIDGE[1]), FAWN, exposed=True),
    box('neck bridge', (NX - 0.010, -0.0175, BRIDGE[0]), (NX + 0.010, 0.0175, BRIDGE[1]), FAWN, exposed=True),
    box('neck yaw seat', (NX - 0.026, -0.011, BRIDGE[0]), (NX - 0.009, 0.011, BRIDGE[1]), FAWN, exposed=True),
]
case, horn, rear = joint('head_yaw', 'neck', 'yawlink', (NX, 0.0, ZYAW), 'z', (-1.0, 1.0), out=+1, long='-x', exposed=True)
neck.append(case)
link('neck', 0.04, neck)

# ================================================================ yaw link: the roll servo lying across the head
YP = (ZYAW - 0.0005, ZYAW + 0.0025)
yawl = [horn,
        box('yaw plate', (XROLL - 0.032, -0.0255, YP[0]), (XROLL - 0.001, 0.0105, YP[1]), DARK, exposed=True),
        box('roll rear wall', (XROLL - 0.032, -0.010, YP[0]), (XROLL - 0.029, 0.010, ZROLL + 0.010), DARK, exposed=True)]
case, horn, rear = joint('head_roll', 'yawlink', 'head', (XROLL, 0.0, ZROLL), 'x', (-0.30, 0.30), out=+1, long='-y')   # round 8: 0.35 met the neck plate (0.30: a 17 deg tilt)
yawl += [case, rear]
link('yawlink', 0.03, yawl)

# ================================================================ head (the sitting design's, 1.15 x)
HC = (0.030, 0.0, 0.138)                 # 0.78 H crown, 0.44 H wide
HR = (0.044 * K, 0.046 * K, 0.038 * K)
MZ = (HC[0] + 0.034 * K, 0.0, HC[2] - 0.023 * K)
MR = (0.022 * K, 0.020 * K, 0.016 * K)
HEAD_BOTTOM = 0.104


def on_cranium(y, z, d=0.0):
    """The point on the cranium's front at (y, z), and its outward normal; d out along the normal."""
    dy, dz = (y - HC[1]) / HR[1], (z - HC[2]) / HR[2]
    x = HC[0] + HR[0] * math.sqrt(max(0.0, 1 - dy * dy - dz * dz))
    n = ((x - HC[0]) / HR[0] ** 2, (y - HC[1]) / HR[1] ** 2, (z - HC[2]) / HR[2] ** 2)
    k = math.sqrt(sum(v * v for v in n))
    n = tuple(v / k for v in n)
    return tuple(p + d * v for p, v in zip((x, y, z), n)), n


def hp(x, y, z):
    """A point of the sitting design's head, moved and scaled onto this one (its cranium centre was (-0.004, 0, 0.130))."""
    return (HC[0] + (x + 0.004) * K, y * K, HC[2] + (z - 0.130) * K)


EYE_Y, EYE_Z = 0.0198 * K, HC[2] - 0.0035 * K
eye = []
for part, r, d, col in (('rim', 0.0132, 0.0, WHITE), ('iris', 0.0108, 0.0016, EYEGREEN), ('pupil', 0.0072, 0.0030, PUPIL)):
    p, n = on_cranium(EYE_Y, EYE_Z, d * K)
    eye.append(cyl(f'L eye {part}', p, list(n), r * K, 0.004 * K, col, 'decal'))
ear = lambda u, v, r: (u * K, HC[2] + (v - 0.130) * K, r * K)
head = [horn,
        ell('cranium', HC, HR, FAWN),
        ell('muzzle', MZ, MR, WHITE, merge_into='cranium'),   # printed as one with the cranium: no wall between them
        ell('nose', (MZ[0] + MR[0] - 0.003, 0.0, MZ[2] + 0.0035 * K), (0.006 * K, 0.008 * K, 0.005 * K), DARKGREEN, 'decal'),
        cut(box('head underside', (HC[0] - 0.07, -0.07, 0.06), (HC[0] + 0.09, 0.07, HEAD_BOTTOM)), 'cranium', 'muzzle'),   # round 8: the muzzle stops at the mouth; the jaw is below
        cut(cyl('head neck opening', (NX, 0, HEAD_BOTTOM + 0.014), 'z', 0.034, 0.030), 'cranium', 'muzzle'),
        *eye,
        plate('L ear', HC[0] - 0.012 * K - 0.003, 0.006, [ear(0.035, 0.151, 0.0105), ear(0.055, 0.168, 0.0075)], FAWN, 'decal', axis='x'),
        plate('L inner ear', HC[0] - 0.012 * K + 0.0025, 0.0012, [ear(0.038, 0.153, 0.0070), ear(0.053, 0.167, 0.0048)], GREEN, 'decal', axis='x'),
        cap('L antler beam', hp(-0.010, 0.012, 0.158), hp(-0.026, 0.024, 0.212), 0.0062 * K, GREEN, 'decal', color_R=RED),
        cap('L antler tine 1', hp(-0.015, 0.016, 0.180), hp(-0.012, 0.036, 0.196), 0.0052 * K, GREEN, 'decal', color_R=RED),
        cap('L antler tine 2', hp(-0.020, 0.020, 0.196), hp(-0.019, 0.040, 0.212), 0.0050 * K, GREEN, 'decal', color_R=RED),
        box('head bracket', (XROLL - 0.0005, -0.0135, ZROLL - 0.011), (XROLL + 0.0025, 0.0135, ZROLL + 0.012), DARK),   # round 8: narrower, clear of the antler servo; its crown post gone for the antlers' bar
        box('head face post', (XROLL + 0.002, -0.005, ZROLL - 0.008), (HC[0] + HR[0] - 0.0030, 0.005, ZROLL - 0.002), DARK),   # round 8: 1.2 mm under the face showed as a seam
        ]
# round 8 (the user: "there is no mouth action which is important"; "make the two antlers move together with another servo")
# The mouth: the muzzle's lower part is a jaw on its own XL330, its case low in the face, its shaft across (y) behind the
# muzzle, 2.9 mm clear of the yaw case's swept corners. The jaw hangs on two plates, the horn's and the rear horn's.
XJ, ZJ = 0.067, 0.117   # (0.069: its plate met the eyes; (0.064, 0.119: its rear stood 5 mm into the neck's opening; (0.116: its case stood in the cranium's and the muzzle's walls)
case, horn, rear = joint('jaw', 'head', 'jaw', (XJ, 0.0145, ZJ), 'y', (-0.05, 0.45), out=+1, long='-x', exposed=True)   # its rear 1.5 mm in the neck's opening: seen from below only
head += [case,
         box('jaw cradle', (XJ - 0.0245, -0.0115, ZJ + 0.0100), (XJ + 0.0080, 0.0115, ZJ + 0.0125), DARK, exposed=True),
         box('jaw cradle post', (XJ - 0.022, -0.004, ZJ + 0.0120), (XJ - 0.016, 0.004, ZROLL - 0.0075), DARK, exposed=True),
         cut(box('L jaw slot', (XJ - 0.016, 0.0130, 0.090), (XJ + 0.024, 0.0190, ZJ - 0.008)), 'muzzle', 'cranium')]
jaw = [horn, rear,
       plate('L jaw plate', 0.0145, 0.003, [(XJ, ZJ, 0.0095), (XJ + 0.008, 0.0985, 0.005)], WHITE, exposed=True),   # seen in the mouth and through the muzzle's slots: the muzzle's white
       loft('jaw', [(0.056, 0.0975, 0.0, 0.0025, 0.012, 2.2), (0.062, 0.0975, 0.0, 0.0050, 0.018, 2.2),   # its back low: behind the hinge it rises as the mouth opens
                    (0.074, 0.0980, 0.0, 0.0060, 0.0205, 2.2), (0.087, 0.0990, 0.0, 0.0045, 0.0160, 2.2),
                    (0.094, 0.1000, 0.0, 0.0015, 0.0070, 2.0)], WHITE, along='x', load_bearing=True),
       ell('tongue', (0.073, 0.0, 0.1022), (0.011, 0.0095, 0.0011), '#B8303A', 'decal'),
       ]
# The antlers: one XL330 beside the roll servo, its shaft across (y); a bar from its horn up and over to both antlers'
# bases, which come out through slots in the crown, each under a tuft of fur that moves with it. +/-0.15 rad: a nod of
# the antlers, the slots 25 mm long.
XA, ZA = 0.034, 0.1475   # (0.150: the horn plate's front stood out of the cranium)
case, horn, rear = joint('antlers', 'head', 'antlers', (XA, 0.0415, ZA), 'y', (-0.15, 0.15), out=+1, long='-x')
# held at the horn only: a plate on the rear horn would sit where the roll servo's case sweeps (13.8 mm round its axis); the
# antlers are light (about 20 g): an open item to test on the printed head
head += [case,
         box('antler cradle', (XA - 0.0245, 0.0155, ZA - 0.0125), (XA + 0.0095, 0.0385, ZA - 0.0100), DARK),
         box('antler cradle post', (XROLL, 0.0110, ZA - 0.0125), (XROLL + 0.004, 0.0170, ZA - 0.0035), DARK)]
AB = hp(-0.010, 0.012, 0.158)            # the antler beam's base
ant = [horn,
       # the horn plate reaches forward past the servo's front; the arm climbs the face's inside to the bar over the crown
       # (plates, not capsules: trimesh's thin capsules aren't closed solids)
       plate('antler horn plate', 0.0415, 0.003, [(XA, ZA, 0.0090), (0.0475, ZA - 0.002, 0.003)], DARK),
       plate('antler bar arm', 0.0455, 0.004, [(0.0425, ZA - 0.001, 0.003), (AB[1] + 0.001, AB[2] - 0.001, 0.003)], DARK, axis='x'),
       box('antler bar', (AB[0] - 0.004, -AB[1] - 0.004, AB[2] - 0.004), (0.0495, AB[1] + 0.004, AB[2] + 0.002), DARK, exposed=True),   # seen through the slots, under the tufts
       ]
head += [cut(box('L antler slot', (AB[0] - 0.013, AB[1] - 0.0085, AB[2] - 0.004), (AB[0] + 0.013, AB[1] + 0.0085, AB[2] + 0.025)), 'cranium')]
link('head', 0.16, [g for g in head if 'antler beam' not in g['name'] and 'antler tine' not in g['name']])
ant += [g for g in head if 'antler beam' in g['name'] or 'antler tine' in g['name']]
ant.append(ell('L antler burr', (AB[0], AB[1], 0.1838), (0.0140, 0.0098, 0.0030), '#7A4A22'))   # a lid over the slot, 1 mm above the crown: a darker burr, as an antler's base (round 8: a fawn disc read as a hat)
link('jaw', 0.02, jaw)
link('antlers', 0.03, ant)

# ================================================================ legs: each servo rides in its own segment, its horn on the one above
# The thigh holds the hip servo (horn inward, on the keel); the shin holds the knee servo (horn outward, on the thigh's
# outer plate), its rear horn on the thigh's inner plate: both sides of both joints held. Written for the front left leg
# and copied to the rear, then mirrored.
LC, LB = (LY[0] + LY[1]) / 2, (LY[1] - LY[0]) / 2


def leg(pre, X, color_hoof, color_hoof_R):
    hip, knee = f'{pre}_hip', f'{pre}_knee'
    thigh, shin = f'{pre}_thigh', f'{pre}_shin'
    case_h, horn_h, rear_h = joint(hip, 'body', thigh, (X, KEEL, ZH), 'y', (-0.5, 0.5), out=-1, long='-z', case_in='child', exposed=True)   # +/-0.5: a trot's; past it the rounder thigh meets the Pi
    case_k, horn_k, rear_k = joint(knee, thigh, shin, (X, LY[1] + 0.0005, ZK), 'y', (-1.3, 1.3), out=+1, long='-z', case_in='child', exposed=True)
    n = pre.replace('_', ' ')
    LINKS['body']['geoms'] += [
        horn_h,
        # round 7: the bib too (Fusion's interference check found it over the front thighs; decals were in no collision check)
        cut(cyl(f'{n} hip pocket', (X, LC, ZH), 'y', 0.0200, 0.030), 'body', 'bib'),   # round 8: the rounder thigh's top reaches 18.5 mm
        cut(box(f'{n} leg slot', (X - 0.034, 0.0025, 0.0), (X + 0.034, 0.0360, ZH)), 'body', 'bib'),
        # the rear horn's seat, fused into the body's side wall: the shell carries the leg's outer side (on_cover)
        cyl(f'{n} hip boss', (X, (LY[1] + 0.0005 + 0.0410) / 2, ZH), 'y', 0.0075, 0.0410 - LY[1] - 0.0005, FAWN, exposed=True),
        rear_h,
    ]
    th = [case_h, horn_k,
          # round 8: rounder (n 4.5, 35 mm deep) round the same case: n 6 read as boxes
          loft(f'{n} thigh', load_bearing=True, sections=[(ZH - 0.0245, X, LC, 0.0175, LB, 4.5), (ZH + 0.006, X, LC, 0.0175, LB, 4.5),
                              (ZH + 0.0115, X, LC, 0.0120, LB, 3.0), (ZH + 0.0158, X, LC, 0.0045, LB - 0.004, 2.0)],
               color=FAWN, open=('bottom',)),
          cut(cyl(f'{n} thigh horn hole', (X, LY[0], ZH), 'y', 0.0095, 0.006), f'{n} thigh'),
          cut(cyl(f'{n} thigh rear horn hole', (X, LY[1], ZH), 'y', 0.0095, 0.006), f'{n} thigh'),
          # round 6: the plates run 0.5 mm into the thigh's walls (they stood off them, and joined nothing)
          plate(f'{n} knee plate', LY[1] + 0.0005, 0.003, [(X, ZK, 0.0095), (X, ZH - 0.020, 0.0105)], FAWN, exposed=True),
          box(f'{n} knee outer tab', (X - 0.006, LY[1] - 0.0010, 0.0460), (X + 0.006, LY[1] + 0.0010, 0.0525), FAWN, exposed=True),   # into the thigh's wall, above the shin's round top
          plate(f'{n} knee inner plate', 0.0005, 0.0020, [(X, ZK, 0.0095), (X, ZH - 0.028, 0.0105)], FAWN, exposed=True),
          box(f'{n} knee inner tab', (X - 0.006, 0.0005, 0.0460), (X + 0.006, 0.0035, 0.0525), FAWN, exposed=True),   # into the thigh's wall, above the shin's round top
          cyl(f'{n} knee cap', (X, LY[1] + 0.0045, ZK), 'y', 0.0115, 0.002, FAWN, exposed=True),   # round 8: a round knee over the plate and horn
          rear_k]
    sh = [case_k,
          # its top stays within 17.5 mm of the knee, which the thigh's servo clears: boxy there (n 8), hidden by the knee caps
          loft(f'{n} shin', [(0.0015, X, LC, 0.0150, LB, 4.0), (0.012, X, LC, 0.0172, LB, 4.5), (ZK + 0.0020, X, LC, 0.0172, LB, 4.5),
                             (ZK + 0.0095, X, LC, 0.0147, LB, 8.0), (ZK + 0.0125, X, LC, 0.0110, LB, 3.0),
                             (ZK + 0.0158, X, LC, 0.0045, LB - 0.004, 2.0)], FAWN),
          cut(cyl(f'{n} shin horn hole', (X, LY[1], ZK), 'y', 0.0095, 0.006), f'{n} shin'),
          cut(cyl(f'{n} shin rear horn hole', (X, LY[0], ZK), 'y', 0.0095, 0.006), f'{n} shin'),
          # round 8: a domed hoof, wider than the leg, as the plush's
          loft(f'{n} hoof', [(0.0, X + 0.002, LC, 0.0175, LB + 0.0012, 2.4), (0.007, X + 0.002, LC, 0.0195, LB + 0.0015, 2.4),
                             (0.014, X + 0.0015, LC, 0.0190, LB + 0.0013, 2.4), (0.018, X + 0.001, LC, 0.0160, LB + 0.0005, 2.2)],
               color_hoof, 'decal', color_R=color_hoof_R,
               material='pla_skin'),
          ]
    link(thigh, 0.03, th)
    link(shin, 0.03, sh)


leg('L_front', XF, RED, GREEN)
leg('L_rear', XR, GREEN, RED)

# ================================================================ mirror the left side

def mirror_geom(g):
    m = copy.deepcopy(g)
    m['name'] = ('R' + g['name'][1:]) if g['name'][:2] in ('L ', 'L_') else g['name']
    if 'color_R' in m:
        m['color'] = m.pop('color_R')
    for k in ('c', 'p0', 'p1'):
        if k in m:
            m[k][1] = -m[k][1]
    if m['kind'] == 'cyl' and not isinstance(m['axis'], str):
        m['axis'][1] = -m['axis'][1]
    if m['kind'] == 'loft':
        for s in m['sections']:
            s['cy'] = -s['cy']
        for c in ([m['clip']] if isinstance(m.get('clip'), dict) else m.get('clip', [])):
            c['p'][1], c['n'][1] = -c['p'][1], -c['n'][1]
    if m['kind'] == 'plate' and m.get('axis', 'y') == 'y':
        m['y0'] = -(g['y0'] + g['t'])
    if m['kind'] == 'plate' and m.get('axis') in ('x', 'z'):
        m['pts'] = [[p[0], -p[1]] if m['axis'] == 'z' else [-p[0], p[1]] for p in m['pts']]
    if m['role'] == 'cut':
        m['targets'] = ['R ' + t[2:] if t.startswith('L ') else t for t in m['targets']]
    return m


ORIG = copy.deepcopy(LINKS)
strip = lambda g: {k: v for k, v in g.items() if k != 'color_R'}
for ln, l in ORIG.items():
    if ln.startswith('L_'):
        LINKS[ln]['geoms'] = [strip(g) for g in l['geoms']]
        LINKS['R_' + ln[2:]] = dict(l, name='R_' + ln[2:], geoms=[mirror_geom(g) for g in l['geoms']])
    else:
        LINKS[ln]['geoms'] = [strip(g) for g in l['geoms']] + [mirror_geom(g) for g in l['geoms'] if g['name'][:2] in ('L ', 'L_')]
for l in LINKS.values():
    for g in l['geoms']:
        g['name'] = g['name'].replace('L_', 'R_', 1) if l['name'].startswith('R_') and g['name'].startswith('L_') else g['name']
for j in [j for j in JOINTS if j['name'].startswith('L_')]:
    m = copy.deepcopy(j)
    for k in ('name', 'parent', 'child'):
        m[k] = m[k].replace('L_', 'R_', 1)
    m['origin'][1], m['act_c'][1] = -m['origin'][1], -m['act_c'][1]
    m['out'] = -m['out'] if m['axis'] == 'y' else m['out']
    m['interface'] = [n.replace('L_', 'R_', 1) for n in m.get('interface', [])]
    if m['axis'] in ('x', 'z'):
        m['lower'], m['upper'] = -j['upper'], -j['lower']
    JOINTS.append(m)
# the per-side colours: a left part keeps its colour, its mirror takes color_R (applied in mirror_geom)


def material_of(g):
    if g.get('material'):
        return g['material']
    return 'actuator' if g['role'] == 'actuator' else 'pla'


for l in LINKS.values():
    for g in l['geoms']:
        if g['role'] != 'cut':
            g['material'] = material_of(g)

SPEC = dict(name='reindeer', root='body', units='m', height=H, links=LINKS, joints=JOINTS, actuators=ACT,
            materials=MATERIALS)

if __name__ == '__main__':
    with open(os.path.join(HERE, 'robot.json'), 'w', encoding='utf-8') as f:
        json.dump(SPEC, f, indent=1)
    n = sum(len(l['geoms']) for l in LINKS.values())
    print(f'{len(LINKS)} links, {len(JOINTS)} joints, {n} geoms -> spec/robot.json')
