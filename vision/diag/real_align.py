"""Draw the projected bays onto REAL stills, so the projection can be judged.

    python vision/diag/real_align.py <stills_dir> [--every 20] [--out DIR]
                                     [--shift DX,DY] [--limit N]

`paint_align.py` measures the projection against the bay paint a Webots frame
renders, sub-pixel, and `--self-test` proves the estimator first. None of that
transfers here: Sofia's bays are largely unpainted, there is no ground truth,
and the pose itself is half-estimated (position from consumer GPS, heading from
`tools/dji_yaw.py`). So this deliberately reports something weaker and honest --
it draws where the projection says each bay is and lets the eye judge, exactly
the argument `detect_real.py` makes for unlabelled real data.

**What to look for, in this order.** A projection that is right puts bay
rectangles along the kerb, aligned with the parked cars and pointing the way the
street runs. Then:

  * a **constant** offset in the same direction on every frame is the GPS bias
    (1-3 m against a 2.2 m bay). It is correctable -- `--shift` applies a trial
    correction so a candidate can be checked by eye before it is committed --
    and it does not indict the yaw.
  * an offset whose direction **rotates with the flight leg** is a yaw error,
    and means `dji_yaw.py` is not finished. This is the failure worth catching:
    it is invisible on a single frame and obvious across four legs, which is why
    `--every` samples the whole flight rather than the first few frames.
  * bays that are the wrong SIZE, or that drift outwards from the image centre,
    are an intrinsics error -- the wrong camera, or an altitude that is not the
    height above the street.

The output is JPEG overlays plus a printed per-frame count of bays in shot.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cameras                                                  # noqa: E402
import cv2                                                      # noqa: E402
import numpy as np                                              # noqa: E402
import score_occupancy as so                                    # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

GREEN = (80, 230, 80)
CYAN = (230, 230, 60)


def frame_bays(pose, bays, cam, shift=(0.0, 0.0)):
    """Bays with at least one corner in shot, as pixel rings.

    Returns (bay, ring_px, fully_inside). `fully_inside` is the rule the sim
    scorer uses to accept a view; it is reported rather than enforced here
    because seeing the clipped ones is what makes a systematic offset visible.
    """
    out = []
    for b in bays:
        ring = [so.project(x + shift[0], y + shift[1], pose, cam=cam)
                for x, y in b["ring"]]
        us = [p[0] for p in ring]
        vs = [p[1] for p in ring]
        if max(us) < 0 or min(us) >= cam.w or max(vs) < 0 or min(vs) >= cam.h:
            continue
        out.append((b, ring,
                    min(us) >= 0 and max(us) < cam.w
                    and min(vs) >= 0 and max(vs) < cam.h))
    return out


def draw(img, hits, half=2):
    """Outline each bay; solid where fully in shot, dotted where clipped."""
    vis = img.copy()
    for b, ring, full in hits:
        pts = np.array([[int(round(u)), int(round(v))] for u, v in ring],
                       dtype=np.int32)
        cv2.polylines(vis, [pts], True, GREEN if full else CYAN,
                      6 if full else 3, cv2.LINE_AA)
        cx = int(sum(p[0] for p in ring) / len(ring))
        cy = int(sum(p[1] for p in ring) / len(ring))
        cv2.putText(vis, b["id"], (cx - 40, cy), cv2.FONT_HERSHEY_SIMPLEX,
                    1.1, GREEN if full else CYAN, 3, cv2.LINE_AA)
    return cv2.resize(vis, (vis.shape[1] // half, vis.shape[0] // half),
                      interpolation=cv2.INTER_AREA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--every", type=int, default=20,
                    help="sample every Nth frame (spread over the whole flight, "
                         "so a leg-dependent error shows)")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--shift", default="0,0",
                    help="trial ENU correction DX,DY in metres, to test a "
                         "candidate GPS bias by eye before committing it")
    ap.add_argument("--cam", default="dji")
    a = ap.parse_args()

    cam = cameras.get(a.cam)
    shift = tuple(float(v) for v in a.shift.split(","))
    out = a.out or os.path.join(a.stills, "align")
    os.makedirs(out, exist_ok=True)

    poses = json.load(open(os.path.join(a.stills, "poses.json"),
                           encoding="utf-8"))
    bays = so.load_all_bays()
    print(f"{len(bays)} bays, {len(poses)} poses, {cam!r}"
          + (f", trial shift {shift}" if any(shift) else ""))

    picked = poses[::a.every]
    if a.limit:
        picked = picked[:a.limit]
    n_full = n_any = 0
    for pose in picked:
        if "yaw" not in pose or "x" not in pose:
            continue
        img = cv2.imread(os.path.join(a.stills, pose["file"]))
        if img is None:
            continue
        hits = frame_bays(pose, bays, cam, shift)
        full = sum(1 for _, _, f in hits if f)
        n_full += full
        n_any += len(hits)
        name = os.path.splitext(pose["file"])[0]
        cv2.imwrite(os.path.join(out, f"align_{name}.jpg"), draw(img, hits))
        print(f"  {pose['file']}  yaw {math.degrees(pose['yaw']):7.2f} deg "
              f"({pose.get('yaw_src', '?')})  {len(hits):3d} bays in shot, "
              f"{full:3d} fully")
    print(f"\n{len(picked)} frames -> {out}"
          f"   ({n_any} bay views, {n_full} fully in shot)")
    print("LOOK at these before trusting any number computed from this pose set.")


if __name__ == "__main__":
    main()
