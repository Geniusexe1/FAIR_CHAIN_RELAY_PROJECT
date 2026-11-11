#!/usr/bin/env python3
"""Add a live relay-state strip under the Gazebo recording.

    .venv/bin/python rssi/ros/state_strip.py <run base path> [--speedup 40]

Reads <run>_gazebo_raw.mkv, <run>_gazebo_t0.txt and <run>_relay_states.jsonl; writes <run>_gazebo.mp4.
"""
import argparse, bisect, json, subprocess
import imageio_ffmpeg
from PIL import Image, ImageDraw, ImageFont

COL = {"PARKED": (107, 114, 128), "DEPLOYING": (217, 119, 6), "RELAYING": (21, 128, 61),
       "RETURNING": (124, 58, 237), "HOLDING": (185, 28, 28)}
MEANING = {"PARKED": "idle near the GCS", "DEPLOYING": "driving to a relay spot", "RELAYING": "carrying the link",
           "RETURNING": "not needed, going back", "HOLDING": "not needed, waiting"}
W, H = 690, 118


def strip(s, clock, font, small):
    im = Image.new("RGB", (W, H), (24, 26, 31))
    d = ImageDraw.Draw(im)
    if not s:
        d.text((10, 10), f"{clock}   waiting for the coordinator", fill=(230, 230, 230), font=small)
        return im
    d.text((10, 6), f"{clock}   route: {' > '.join(s['route'])}", fill=(230, 230, 230), font=small)
    for i, (n, st) in enumerate(sorted(s["state"].items())):
        y = 32 + i * 42
        d.rounded_rectangle((10, y, 150, y + 34), 5, fill=COL[st])
        d.text((18, y + 7), st, fill=(255, 255, 255), font=font)
        x_, y_ = s["xy"][n]
        d.text((162, y + 1), f"{n}  at ({x_:.1f}, {y_:.1f})", fill=(235, 235, 235), font=small)
        d.text((162, y + 17), MEANING[st], fill=(170, 175, 185), font=small)
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("base"); ap.add_argument("--speedup", type=float, default=40)
    ap.add_argument("--crop", default="690:1232:270:68")
    a = ap.parse_args()
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    t0 = float(open(a.base + "_gazebo_t0.txt").read())
    rows = [json.loads(x) for x in open(a.base + "_relay_states.jsonl")]
    ts = [r["t"] for r in rows]
    out = subprocess.run([ff, "-hide_banner", "-i", a.base + "_gazebo_raw.mkv", "-map", "0:v", "-f", "null", "-"],
                         capture_output=True, text=True).stderr
    n = int(out.rsplit("frame=", 1)[1].split()[0])
    font = ImageFont.truetype("DejaVuSans-Bold.ttf", 14)
    small = ImageFont.truetype("DejaVuSans.ttf", 13)
    # the strip is made at the raw video's 2 fps, so both streams speed up the same way
    p = subprocess.Popen([ff, "-hide_banner", "-loglevel", "error", "-y", "-i", a.base + "_gazebo_raw.mkv",
                          "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", "2", "-i", "-",
                          "-filter_complex", f"[0:v]crop={a.crop},scale={W}:-2[g];[g][1:v]vstack,setpts=PTS/{a.speedup},fps=30",
                          "-c:v", "libx264", "-preset", "medium", "-crf", "28", "-pix_fmt", "yuv420p", "-an",
                          a.base + "_gazebo.mp4"], stdin=subprocess.PIPE)
    import time
    for i in range(n):
        t = t0 + i / 2
        k = bisect.bisect_right(ts, t) - 1
        p.stdin.write(strip(rows[k] if k >= 0 else None, time.strftime("%H:%M:%S", time.localtime(t)), font, small).tobytes())
    p.stdin.close()
    p.wait()
    print(f"wrote {a.base}_gazebo.mp4 ({n} raw frames, x{a.speedup})")


if __name__ == "__main__":
    main()
