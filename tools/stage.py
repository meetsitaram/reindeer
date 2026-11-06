"""A MuJoCo stage for the reindeer's videos and pictures: each part (tools/assembly.py's) is a mocap body that can be
moved, faded or hidden per frame; plain pale background, no floor; portrait frames; two-line centred labels.

Used by tools/reveal_video.py, tools/render_build_video.py and tools/render_parts_layout.py. Plain python (MuJoCo).
"""
import math
import os
import tempfile

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

BG = (234, 236, 240)                     # one pale colour, no floor (assembly-videos.md)
GLOSSY = ('#08090B', '#1BA05A', '#17542E')
WHITE_DIM = 0.86                         # white parts below full brightness, so they keep their shape on the pale ground
FONTS = r'C:\Windows\Fonts'


def rgba(hex_color, alpha=1.0):
    h = hex_color.lstrip('#')
    c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    if min(c) > 0.85:
        c = [v * WHITE_DIM for v in c]
    return c + [alpha]


def stl(path, v, f):
    tri = v[f].astype(np.float32)
    rec = np.zeros(len(f), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['v'] = tri
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80)
        fh.write(np.uint32(len(f)).tobytes())
        fh.write(rec.tobytes())


def mesh_of(manifold):
    m = manifold.to_mesh()
    return np.asarray(m.vert_properties[:, :3], float), np.asarray(m.tri_verts, int)


class Stage:
    """items: [(key, verts (world, at the seat), faces, hex colour)]."""

    def __init__(self, items, w, h):
        self.w, self.h = w, h
        self.keys = [k for k, *_ in items]
        self.colors = {k: c for k, _, _, c in items}
        tmp = tempfile.mkdtemp(prefix='stage_')
        assets, bodies = [], []
        for i, (k, v, f, c) in enumerate(items):
            if len(f) == 0:
                continue
            stl(os.path.join(tmp, f'p{i}.stl'), v, f)
            assets.append(f'<mesh name="p{i}" file="p{i}.stl"/>')
            mat = 'gloss' if c in GLOSSY else 'matte'
            col = ' '.join(f'{x:.3f}' for x in rgba(c))
            bodies.append(f'<body name="b{i}" mocap="true"><geom name="g{i}" type="mesh" mesh="p{i}" material="{mat}" rgba="{col}" contype="0" conaffinity="0"/></body>')
        xml = f"""<mujoco><compiler meshdir="{tmp}"/>
<visual><global offwidth="{max(w, 64)}" offheight="{max(h, 64)}"/><headlight ambient=".36 .36 .36" diffuse=".46 .46 .46" specular=".08 .08 .08"/>
<quality shadowsize="0"/><map znear="0.001" zfar="50"/></visual>
<asset><material name="matte" specular=".06" shininess=".1"/><material name="gloss" specular=".6" shininess=".85"/>
{''.join(assets)}</asset>
<worldbody><light directional="true" diffuse=".38 .38 .38" specular=".15 .15 .15" dir="-0.5 -0.35 -1" castshadow="false"/>
<light directional="true" diffuse=".14 .14 .16" specular="0 0 0" dir="0.4 0.5 0.7" castshadow="false"/>
{''.join(bodies)}</worldbody></mujoco>"""
        p = os.path.join(tmp, 'stage.xml')
        open(p, 'w').write(xml)
        self.m = mujoco.MjModel.from_xml_path(p)
        self.d = mujoco.MjData(self.m)
        self.body = {k: self.m.body(f'b{i}').mocapid[0] for i, (k, *_ ) in enumerate(items) if len(_[1])}
        self.geom = {k: self.m.geom(f'g{i}').id for i, (k, *_ ) in enumerate(items) if len(_[1])}
        self.base_rgba = {k: self.m.geom_rgba[g].copy() for k, g in self.geom.items()}
        self.r = mujoco.Renderer(self.m, h, w)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.m.vis.global_.fovy = 30

    def place(self, poses, alpha=None):
        """poses: {key: 4x4 or None (hidden)}; alpha: {key: 0..1}. Keys not given are hidden."""
        alpha = alpha or {}
        for k, mid in self.body.items():
            M = poses.get(k)
            if M is None:
                self.d.mocap_pos[mid] = (0, 0, -50)
                continue
            self.d.mocap_pos[mid] = M[:3, 3]
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, np.ascontiguousarray(M[:3, :3]).flatten())
            self.d.mocap_quat[mid] = q
            a = alpha.get(k, 1.0)
            c = self.base_rgba[k].copy()
            c[3] = a
            self.m.geom_rgba[self.geom[k]] = c
            if a <= 0.01:
                self.d.mocap_pos[mid] = (0, 0, -50)
        mujoco.mj_forward(self.m, self.d)

    def look(self, target, distance, az, el):
        """az: degrees from the robot's front (+x) toward its left (+y), where the camera is; el: degrees up."""
        a, e = math.radians(az), math.radians(el)
        dv = np.array([math.cos(a) * math.cos(e), math.sin(a) * math.cos(e), math.sin(e)])
        self.cam.lookat[:] = target
        self.cam.distance = distance
        self.cam.azimuth = math.degrees(math.atan2(-dv[1], -dv[0]))
        self.cam.elevation = -el

    def render(self, seg=False):
        self.r.update_scene(self.d, self.cam)
        img = self.r.render().copy()
        self.r.enable_segmentation_rendering()
        self.r.update_scene(self.d, self.cam)
        s = self.r.render()[:, :, 0].copy()
        self.r.disable_segmentation_rendering()
        img[s < 0] = BG
        return (Image.fromarray(img), s) if seg else Image.fromarray(img)

    def visible(self, key, s):
        return int((s == self.geom[key]).sum()) if key in self.geom else 0

    def fit_distance(self, lo, hi, margin=1.15, least=0.12):
        """A camera distance that keeps the box lo..hi in a portrait frame from any side: its widest plan size against the
        frame's width, its height against the frame's height."""
        lo, hi = np.asarray(lo), np.asarray(hi)
        half_w = np.linalg.norm((hi - lo)[:2]) / 2
        half_h = (hi - lo)[2] / 2
        v = math.radians(15)
        hz = math.atan(math.tan(v) * self.w / self.h)
        return max(least, margin * max(half_w / math.tan(hz), half_h / math.tan(v)))


def font(size, bold=False):
    name = 'segoeuib.ttf' if bold else 'segoeui.ttf'
    try:
        return ImageFont.truetype(os.path.join(FONTS, name), size)
    except OSError:
        return ImageFont.load_default()


def label(img, big, small='', where='top', alpha=1.0, color=(30, 32, 36), bottom_gap=90):
    """Two short centred lines: the large one dark (or green for a test), the small one grey. A thin pale edge keeps
    them readable where a part passes behind. Fades with alpha."""
    if not big and not small:
        return img
    W, H = img.size
    k = W / 720
    f1, f2 = font(int(40 * k), True), font(int(27 * k))
    lay = Image.new('RGBA', img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(lay)
    h1 = d.textbbox((0, 0), big or ' ', font=f1)[3]
    h2 = d.textbbox((0, 0), small or ' ', font=f2)[3]
    y = int(60 * k) if where == 'top' else int(H - bottom_gap * k - h1 - h2 - 20 * k)
    a = int(255 * max(0.0, min(1.0, alpha)))
    for text, f, col, yy in ((big, f1, color, y), (small, f2, (110, 114, 120), y + h1 + int(10 * k))):
        if not text:
            continue
        tw = d.textbbox((0, 0), text, font=f)[2]
        x = (W - tw) // 2
        d.text((x, yy), text, font=f, fill=(*BG, a), stroke_width=max(2, int(3 * k)), stroke_fill=(*BG, a))
        d.text((x, yy), text, font=f, fill=(*col, a))
    out = img.convert('RGBA')
    out.alpha_composite(lay)
    return out.convert('RGB')


def ease(t):
    t = max(0.0, min(1.0, t))
    return 0.5 - 0.5 * math.cos(math.pi * t)
