"""Every part of the reindeer once, flat on a plain background, at one scale, nothing touching, in its real colours:
big printed parts first, then smaller printed parts, then servos and bought parts (then screws, when the spec has them).

  python tools/render_parts_layout.py      -> docs/parts_layout.png, and its count beside tools/assembly.py's parts

Drawn from the same parts as the build order (tools/assembly.py: covers in their printed halves, horns with their
servo), so it can't drift from the design. Plain python (MuJoCo).
"""
import os
import sys

import numpy as np
from PIL import ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly as A  # noqa: E402
import geom as G  # noqa: E402
import stage as ST  # noqa: E402

W_PX, H_PX = 2400, 1700
ROW_W = 0.62                 # m of parts per row
GAP = 0.010                  # m between parts
OUT = os.path.join(G.ROOT, 'docs', 'parts_layout.png')


def flat(v):
    """Turn a part so its thinnest size is vertical, and its longest runs along x."""
    ext = np.ptp(v, axis=0)
    order = np.argsort(ext)                          # thinnest first
    perm = [order[2], order[1], order[0]]            # longest -> x, middle -> y, thinnest -> z
    v = v[:, perm]
    if np.linalg.det(np.eye(3)[perm]) < 0:           # a turn, never a mirror: a left half must stay a left half
        v[:, 2] = -v[:, 2]
    return v - v.min(axis=0)


def category(p):
    if 'fast' in p.kind or p.kind.startswith('M'):
        return 3, 'Screws'
    if not p.kind.startswith('printed'):
        return 2, 'Servos and bought parts'
    size = np.ptp(np.vstack([p.lo, p.hi]), axis=0).max()
    return (0, 'Printed parts, large') if size > 0.045 else (1, 'Printed parts, small')


def main():
    spec, parts = A.load()
    rows = []
    for k, p in parts.items():
        v, f = ST.mesh_of(p.solid)
        v = flat(v)
        rows.append((category(p), -np.ptp(v[:, 0]) * np.ptp(v[:, 1]), k, v, f, p.color))
    rows.sort(key=lambda r: (r[0][0], r[1]))
    items, x, y, rowh, cat_now, heads = [], 0.0, 0.0, 0.0, None, []
    for (ci, cname), _, k, v, f, c in rows:
        e = np.ptp(v, axis=0)
        if cname != cat_now:                         # each kind starts a new row, under its heading
            if cat_now is not None:
                y += rowh + GAP * 2.5
            x, rowh, cat_now = 0.0, 0.0, cname
            heads.append((cname, y))
            y += 0.018
        if x + e[0] > ROW_W:
            x, y, rowh = 0.0, y + rowh + GAP, 0.0
        items.append((k, v + np.array([x, -0.0 - y - e[1], 0.0]) * [1, 1, 1] + [0, 0, 0], f, c))
        x += e[0] + GAP
        rowh = max(rowh, e[1])
    total_h = y + rowh
    # placed with y going down the page: flip to world y
    st = ST.Stage([(k, v * [1, 1, 1], f, c) for k, v, f, c in items], W_PX, H_PX)
    st.place({k: np.eye(4) for k, *_ in items})
    st.m.vis.global_.orthographic = 1
    span_h = max(total_h, ROW_W * H_PX / W_PX) * 1.14
    st.m.vis.global_.fovy = span_h
    st.cam.lookat[:] = [ROW_W / 2, -total_h / 2, 0.0]
    st.cam.distance = 2.0
    st.cam.azimuth = 90.0
    st.cam.elevation = -90.0
    img = st.render()
    d = ImageDraw.Draw(img)
    px = H_PX / span_h                               # pixels per metre
    f1 = ST.font(30, True)
    for name, yy in heads:
        yp = H_PX / 2 + (yy - total_h / 2) * px
        d.text((W_PX / 2 - ROW_W / 2 * px, yp - 2), name, font=f1, fill=(60, 62, 68))
    d.text((20, H_PX - 50), f'{len(items)} parts, one scale: 10 cm = {0.1 * px:.0f} px. Covers as printed, in halves; servos with their horns.',
           font=ST.font(26), fill=(90, 94, 100))
    d.line([(W_PX - 40 - 0.1 * px, H_PX - 34), (W_PX - 40, H_PX - 34)], fill=(60, 62, 68), width=4)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    img.save(OUT)
    print(OUT, f'{len(items)} parts drawn; tools/assembly.py has {len(parts)}')


if __name__ == '__main__':
    main()
