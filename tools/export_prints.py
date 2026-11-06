"""One STL per printed part, from the spec's own shapes (cuts applied; covers hollow at their 2 mm wall), into prints/<link>/.

  python -I tools/export_prints.py      -> prints/, and prints/PRINTS.md (each part: size, volume, grams, colour)

These are the spec's shapes, not Fusion's yet (Fusion has the same shapes, not rounded: phase 4's open item). Bought parts
('(buy)' in the name) and the servo horns are left out.
"""
import os
import sys

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks as C  # noqa: E402
import geom as G  # noqa: E402

OUT = os.path.join(G.ROOT, 'prints')


def main():
    spec = G.load()
    S = C.Solids(spec)
    rows, n = [], 0
    for ln, l in spec['links'].items():
        for i, g in enumerate(l['geoms']):
            m = S.geom.get((ln, i))
            if m is None or g['role'] in ('cut', 'actuator') or '(buy)' in g['name'] or 'flange' in g['name']:
                continue
            if g.get('material') not in ('pla', 'pla_skin'):
                continue
            solid = S.wall[(ln, i)] if g['role'] == 'shell' else m
            mesh = solid.to_mesh()
            tm = trimesh.Trimesh(np.asarray(mesh.vert_properties[:, :3], float) * 1000, np.asarray(mesh.tri_verts, int))
            if tm.volume <= 0:
                continue
            os.makedirs(os.path.join(OUT, ln), exist_ok=True)
            name = g['name'].replace(' ', '_')
            tm.export(os.path.join(OUT, ln, f'{name}.stl'))
            size = np.sort(np.ptp(tm.vertices, axis=0))[::-1]
            rows.append(f"| {ln} | {g['name']} | {g['color']} | {size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} | {tm.volume / 1000:.1f} | {tm.volume / 1000 * 1.24:.1f} |")
            n += 1
    lines = ['# Print files', '', f'{n} printed parts from `spec/robot.py` (`python -I tools/export_prints.py`), PLA, in millimetres. '
             'Covers are their 2 mm wall. Not yet rounded, and not yet joined into print pieces with screw holes: '
             'phase 4 open items.', '', '| link | part | colour | size mm | cm3 | g (PLA, solid) |', '|---|---|---|---|---|---|'] + rows
    open(os.path.join(OUT, 'PRINTS.md'), 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    print(f'{n} printed parts -> prints/, 0 failing')


if __name__ == '__main__':
    main()
