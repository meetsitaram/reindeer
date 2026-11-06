"""Frames from the user's reference video, at exact times, into %TEMP%/figrefs/reindeer (never the repo).

  python -I tools/reference_frames.py [video]     default C:/Users/meets/Downloads/reindeer.mp4

Uses the ffmpeg that ships with imageio-ffmpeg (no ffmpeg on PATH here). Also writes a contact sheet at 2 frames/s to
choose frames by looking.
"""
import os
import subprocess
import sys

import imageio_ffmpeg

VIDEO = sys.argv[1] if len(sys.argv) > 1 else r'C:\Users\meets\Downloads\reindeer.mp4'
OUT = os.path.join(os.environ.get('TEMP', '/tmp'), 'figrefs', 'reindeer')
TIMES = {'front_0.0.png': 0.0, 'side_3.1.png': 3.1, 'side_1.6.png': 1.6, 'back_2.5.png': 2.5, 'quarter_0.8.png': 0.8}

ff = imageio_ffmpeg.get_ffmpeg_exe()
os.makedirs(OUT, exist_ok=True)
for name, t in TIMES.items():
    subprocess.run([ff, '-loglevel', 'error', '-y', '-ss', str(t), '-i', VIDEO, '-frames:v', '1',
                    os.path.join(OUT, name)], check=True)
subprocess.run([ff, '-loglevel', 'error', '-y', '-i', VIDEO, '-vf', 'fps=2,scale=320:-1,tile=6x2',
                '-frames:v', '1', os.path.join(OUT, 'contact_sheet.jpg')], check=True)
print('frames in', OUT)
