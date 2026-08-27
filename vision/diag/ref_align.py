"""Project OSM BUILDINGS and ROADS into a real frame, to test the pose itself.

    python vision/diag/ref_align.py <stills_dir> [--every N] [--frames a,b,c]
                                    [--alt-scale S] [--out DIR]

`real_align.py` draws the parking bays and asks "do they land on the cars?".
When they do not, that question cannot tell you WHY: the pose could be wrong, or
the bay geometry could be. This script breaks the tie by drawing something whose
ground truth is not in doubt -- a building footprint. A roof is unmistakable in a
nadir photograph and its OSM outline is independent of Sofiaplan's parking data.

  * buildings land right, bays do not  -> the pose is good, the BAY DATA is off
  * buildings are displaced too        -> the projection/pose is wrong, and any
                                          conclusion drawn about the bays is void

`--alt-scale S` multiplies the pose altitude before projecting. It is the knob
for the one error `real_align.py` cannot show: `rel_alt` is height above the
TAKEOFF point, not above the ground being photographed, so on sloping ground
every projection is stretched radially about the nadir point. A scale error looks
like a good fit near the image centre and a growing miss towards the edges --
which is exactly what a bad global shift is mistaken for.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import cameras                                                   # noqa: E402
import score_occupancy as so                                     # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data")


def load_layer(path, want):
    """[(kind, [(x, y), ...]), ...] in ENU metres."""
    out = []
    for ft in json.load(open(path, encoding="utf-8"))["features"]:
        g = ft["geometry"]
        rings = [g["coordinates"][0]] if g["type"] == "Polygon" else [g["coordinates"]]
        for ring in rings:
            pts = [so.to_enu(c[0], c[1]) for c in ring]
            out.append((want, pts, ft["properties"]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--every", type=int, default=20)
    ap.add_argument("--frames", help="comma-separated frame numbers instead of --every")
    ap.add_argument("--alt-scale", type=float, default=1.0)
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--out")
    a = ap.parse_args()

    import cv2

    cam = cameras.get(a.cam)
    poses = [p for p in json.load(open(os.path.join(a.stills, "poses.json"),
                                      encoding="utf-8")) if "yaw" in p and "x" in p]
    if a.frames:
        want = {int(v) for v in a.frames.split(",")}
        poses = [p for p in poses if int(p["file"][6:10]) in want]
    else:
        poses = poses[::a.every]

    areas = load_layer(os.path.join(DATA, "block_areas.geojson"), "area")
    roads = load_layer(os.path.join(DATA, "block_roads.geojson"), "road")
    bays = so.load_all_bays()
    out = a.out or os.path.join(a.stills, "ref")
    os.makedirs(out, exist_ok=True)
    print(f"{len(areas)} areas, {len(roads)} roads, {len(bays)} bays, "
          f"{len(poses)} frames, {cam!r}, alt-scale {a.alt_scale}")

    for pose in poses:
        img = cv2.imread(os.path.join(a.stills, pose["file"]))
        if img is None:
            continue
        h, w = img.shape[:2]
        p = dict(pose)
        p["alt"] = pose["alt"] * a.alt_scale

        def to_px(pts):
            return [so.project(x, y, p, cam=cam) for x, y in pts]

        def onscreen(uv):
            return any(-w < u < 2 * w and -h < v < 2 * h for u, v in uv)

        drawn = {"building": 0, "green": 0, "road": 0, "bay": 0}
        for kind, pts, props in areas:
            uv = to_px(pts)
            if not onscreen(uv):
                continue
            building = bool(props.get("building"))
            col = (0, 200, 255) if building else (120, 200, 120)   # BGR
            drawn["building" if building else "green"] += 1
            cv2.polylines(img, [_np(uv)], True, col, 6 if building else 3)
        for kind, pts, props in roads:
            uv = to_px(pts)
            if not onscreen(uv):
                continue
            drawn["road"] += 1
            cv2.polylines(img, [_np(uv)], False, (255, 120, 0), 4)
        for b in bays:
            uv = to_px(b["ring"])
            if not onscreen(uv):
                continue
            drawn["bay"] += 1
            cv2.polylines(img, [_np(uv)], True, (0, 0, 255), 4)

        cv2.putText(img, f"{pose['file']}  yaw {math.degrees(pose['yaw']):.1f} deg  "
                    f"alt {p['alt']:.1f} m (x{a.alt_scale})", (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 255, 255), 4)
        cv2.imwrite(os.path.join(out, "ref_" + pose["file"]),
                    cv2.resize(img, (w // 2, h // 2)),
                    [cv2.IMWRITE_JPEG_QUALITY, 88])
        print(f"  {pose['file']}  buildings {drawn['building']}, greens "
              f"{drawn['green']}, roads {drawn['road']}, bays {drawn['bay']}")

    print(f"\n-> {out}")
    print("ORANGE = OSM building, GREEN = green area, BLUE = road centerline, "
          "RED = parking bay.")
    print("Buildings are the control: if a ROOF outline sits on its roof, the "
          "pose is right and a displaced bay is a data fact.")


def _np(uv):
    import numpy as np
    return np.array([[int(round(u)), int(round(v))] for u, v in uv], dtype="int32")


if __name__ == "__main__":
    main()
