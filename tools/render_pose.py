"""Offline renders of the replica in any pose, from the spec's own shapes (cuts applied, each part in its colour),
posed with the same kinematics the checks use, drawn with MuJoCo's renderer. For when Fusion cannot save images
(2026-10-31: with the PC's screen asleep its viewport saved nothing) and for poses Fusion is slow to set.

  python tools/render_pose.py           -> reports/poses.png (the joint-range poses below)
  python tools/render_pose.py --views   -> reports/compare/replica_<view>.png, framed as build.render's (for
                                           tools/compare.py when Fusion cannot render; same robot box within 2 px)

Plain python (MuJoCo lives in the user site-packages, which -I skips); it reads only our own files.
"""
import math
import os
import sys
import tempfile

import mujoco
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402

GLOSSY = ('#08090B', '#0A1420')
VIEWS = {'front': (0, 0), 'side': (90, 0), 'right': (-90, 0), 'r34': (-55, 6), 'back': (180, 0), '34': (60, 8), 'hero': (40, 8)}


def rgba(hex_color):
    h = hex_color.lstrip('#')
    return [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)] + [1.0]


def stl(path, v, f):
    """Binary STL (MuJoCo reads it; normals are recomputed)."""
    tri = v[f].astype(np.float32)
    rec = np.zeros(len(f), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['v'] = tri
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80)
        fh.write(np.uint32(len(f)).tobytes())
        fh.write(rec.tobytes())


class Scene:
    def __init__(self, spec=None):
        self.spec = spec or G.load()
        self.S = C.Solids(self.spec)
        self.R = G.Robot(self.spec)
        self.parts = []                                   # (link, verts, faces, colour, role)
        for (ln, i), m in self.S.geom.items():
            if m is None:
                continue
            g = self.spec['links'][ln]['geoms'][i]
            mesh = m.to_mesh()
            self.parts.append((ln, np.asarray(mesh.vert_properties[:, :3], float),
                               np.asarray(mesh.tri_verts, int), g['color'], g['role']))

    def model(self, q, tmp, hide=()):
        M = self.R.fk(q)
        if 'L_foot' in M:                                 # stand on the left foot where it stands at the zero pose
            M = self.R.fk(q, base=np.linalg.inv(M['L_foot']))
        assets, geoms = [], []
        for k, (ln, v, f, color, role) in enumerate(self.parts):
            if len(f) == 0 or role in hide:
                continue
            stl(os.path.join(tmp, f'p{k}.stl'), G.apply(M[ln], v), f)
            assets.append(f'<mesh name="p{k}" file="p{k}.stl"/>')
            mat = 'gloss' if color in GLOSSY else 'matte'
            geoms.append(f'<geom type="mesh" mesh="p{k}" material="{mat}" rgba="{" ".join(f"{c:.3f}" for c in rgba(color))}"/>')
        xml = f"""<mujoco><compiler meshdir="{tmp}"/>
<visual><global offwidth="1600" offheight="1600"/><headlight ambient=".42 .42 .42" diffuse=".45 .45 .45" specular=".1 .1 .1"/>
<quality shadowsize="0"/></visual>
<asset><material name="matte" specular=".05" shininess=".1"/><material name="gloss" specular=".7" shininess=".9" reflectance="0"/>
{''.join(assets)}</asset>
<worldbody><light directional="true" diffuse=".35 .35 .35" specular=".2 .2 .2" dir="-0.6 -0.3 -1" castshadow="false"/>
{''.join(geoms)}</worldbody></mujoco>"""
        path = os.path.join(tmp, 'scene.xml')
        with open(path, 'w') as fh:
            fh.write(xml)
        return mujoco.MjModel.from_xml_path(path)

    def render(self, q, view='front', w=800, h=600, height_m=1.9, target_z=0.86, target_y=0.0, hide=(), target_x=0.0, background=255):
        with tempfile.TemporaryDirectory() as tmp:
            m = self.model(q, tmp, hide)
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            az, el = (math.radians(a) for a in VIEWS[view])
            dv = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])
            x = np.cross([0, 0, 1], dv)
            x /= np.linalg.norm(x)
            y = np.cross(dv, x)
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.lookat[:] = [target_x, target_y, target_z]
            cam.distance = 4.0
            cam.azimuth = math.degrees(math.atan2(-dv[1], -dv[0]))
            cam.elevation = -math.degrees(el)
            m.vis.global_.orthographic = 1
            m.vis.global_.fovy = height_m
            r = mujoco.Renderer(m, h, w)
            r.update_scene(d, cam)
            img = r.render().copy()
            r.enable_segmentation_rendering()
            r.update_scene(d, cam)
            seg = r.render()[:, :, 0]
            r.close()
        img[seg < 0] = background
        return Image.fromarray(img)


COVERS = ('shell', 'decal')
CUTE = dict(neck_pitch=0.2, head_roll=0.33, head_yaw=0.35, L_front_hip=-0.6, L_front_knee=1.2)
POSES = [
    # (title, angles, view, picture height m, target z m, target y m, roles hidden)
    ('standing', {}, '34', 0.27, 0.12, 0.0, ()),
    ('head tilt, paw up', CUTE, '34', 0.27, 0.12, 0.0, ()),
    ('looking round, nodding', dict(head_yaw=0.9, neck_pitch=0.4), 'front', 0.27, 0.12, 0.0, ()),
    ('trot', dict(L_front_hip=-0.5, L_front_knee=0.9, R_rear_hip=-0.5, R_rear_knee=0.9, R_front_hip=0.2, L_rear_hip=0.2), 'side', 0.26, 0.12, 0.0, ()),
    ('inside, front', {}, 'front', 0.27, 0.12, 0.0, COVERS),
    ('inside, side', {}, 'side', 0.27, 0.12, 0.0, COVERS),
]


def main():
    sc = Scene()
    if '--views' in sys.argv:
        for v in ('side', '34', 'front', 'back'):
            out = os.path.join(G.ROOT, 'reports', 'compare', f'replica_{v}.png')
            sc.render({}, v, 800, 800, 0.27, 0.12).save(out)
            print(out, flush=True)
        with open(os.path.join(G.ROOT, 'reports', 'compare', 'replica_source.txt'), 'w', encoding='utf-8') as fh:
            fh.write('offline render of the spec shapes')    # compare.py's panel label; build.render removes it
        return
    tiles = []
    for title, q, view, hm, tz, ty, hide in POSES:
        im = sc.render(q, view, 640, 480, hm, tz, ty, hide)
        ImageDraw.Draw(im).text((8, 6), title, fill=(0, 0, 0))
        tiles.append(im)
        print(title, flush=True)
    cols = 2
    rows = math.ceil(len(tiles) / cols)
    sheet = Image.new('RGB', (cols * 640, rows * 480), 'white')
    for k, im in enumerate(tiles):
        sheet.paste(im, ((k % cols) * 640, (k // cols) * 480))
    out = os.path.join(G.ROOT, 'reports', 'poses.png')
    sheet.save(out)
    print(out)


if __name__ == '__main__':
    main()
