"""The reference beside the reindeer, at one height, with the target lines across both, and close-ups.

  python tools/compare.py      -> reports/compare/compare_<view>.png, compare_head.png (LOCAL ONLY: they hold the
                                  reference's frames, which are not ours to publish; reports/compare is git-ignored)

The frames come from the user's video by tools/reference_frames.py into %TEMP%/figrefs/reindeer, never the repo.
Plain python (MuJoCo for the renders).
"""
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geom as G  # noqa: E402
import render_pose as RP  # noqa: E402

REF = os.path.join(os.environ.get('TEMP', '/tmp'), 'figrefs', 'reindeer')
OUT = os.path.join(G.ROOT, 'reports', 'compare')
# (reference frame, our view): walking 0.0 s from the front; walking 5.0 s, its right side three-quarters from the front
PAIRS = {'front': ('walk_0.0.png', 'front'), 'side': ('walk_5.0.png', 'r34')}   # the walking video (round 5)
PX = 640                                          # picture height, both panels


def silhouette(im):
    a = np.asarray(im.convert('HSV')).astype(float) / 255
    m = (a[..., 1] > 0.28) | (a[..., 2] < 0.45)
    lab, n = ndimage.label(m)
    m = lab == (1 + np.argmax(ndimage.sum(m, lab, range(1, n + 1))))
    return ndimage.binary_fill_holes(ndimage.binary_closing(m, iterations=3))


def fit(im, mask, H_px):
    """Crop to the subject (floor to top), scale to H_px tall, pad to a square-ish panel; return the panel."""
    ys, xs = np.nonzero(mask)
    top, bot = ys.min(), ys.max()
    cx = (xs.min() + xs.max()) / 2
    k = H_px / (bot - top)
    w = int(im.width * k)
    im = im.resize((w, int(im.height * k)), Image.LANCZOS)
    pan = Image.new('RGB', (int(H_px * 0.9), int(H_px * 1.1)), 'white')
    x0 = int(pan.width / 2 - cx * k)
    y0 = int(H_px * 1.05 - bot * k)
    pan.paste(im, (x0, y0))
    return pan


def lines(pan, H_px, marks):
    d = ImageDraw.Draw(pan)
    for name, f in marks:
        y = int(H_px * 1.05 - f * H_px)
        d.line([(0, y), (pan.width, y)], fill=(0, 140, 255), width=1)
        d.text((3, y - 11), f'{name} {f:.2f}', fill=(0, 90, 200))
    return pan


def main():
    os.makedirs(OUT, exist_ok=True)
    tg = json.load(open(os.path.join(G.ROOT, 'spec', 'targets.json'), encoding='utf-8'))
    marks = [(k[:-2], v['v']) for k, v in tg['fractions'].items() if k.endswith('_z')]
    sc = RP.Scene()
    for key, (frame, view) in PAIRS.items():
        p = os.path.join(REF, frame)
        if not os.path.exists(p):
            sys.exit(f'{p} missing: run tools/reference_frames.py first')
        ref = Image.open(p).convert('RGB')
        a = fit(ref, silhouette(ref), PX)
        r = sc.render({}, view, 1000, 1000, 0.26, 0.12, background=255)
        b = fit(r, np.asarray(r.convert('L')) < 250, PX)
        a, b = lines(a, PX, marks), lines(b, PX, marks)
        sheet = Image.new('RGB', (a.width + b.width, a.height), 'white')
        sheet.paste(a, (0, 0))
        sheet.paste(b, (a.width, 0))
        ImageDraw.Draw(sheet).text((4, 4), f'reference {frame} (local only)', fill=(0, 0, 0))
        ImageDraw.Draw(sheet).text((a.width + 4, 4), f'reindeer, {view} view, same height', fill=(0, 0, 0))
        out = os.path.join(OUT, f'compare_{key}.png')
        sheet.save(out)
        # the head close up: top 45 % of both
        h = int(PX * 0.62)
        crop = Image.new('RGB', (a.width + b.width, h), 'white')
        crop.paste(a.crop((0, 0, a.width, h)), (0, 0))
        crop.paste(b.crop((0, 0, b.width, h)), (a.width, 0))
        crop.save(os.path.join(OUT, f'compare_{key}_head.png'))
        print(out)


if __name__ == '__main__':
    main()
