"""Choose a small set of frames that covers every bay a flight saw, for annotation.

    python vision/pick_frames.py pics/dji/stills/DJI_..._0035_D --out vision/data/annot_0035

A flight is 137 frames and the same bay appears in a dozen of them, so annotating
everything is mostly redundant work and creates duplicates that have to be merged
back out. This picks a near-minimal covering set by greedy set cover: repeatedly
take the frame that adds the most not-yet-covered bays.

Frames are copied at FULL resolution and under their original names, because
`import_annotations.py` matches an annotation back to its pose by filename and
unprojects using the real pixel coordinates. Do not resize them.

Deliberately NO overlay of the current bay rectangles is drawn on these frames.
Being in the wrong place is the reason this exercise exists, so showing them would
anchor the annotator to the very geometry being replaced. `vision/diag/real_align.py`
renders that overlay separately if you want to see the before.
"""
import argparse
import json
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cameras                                                   # noqa: E402
import score_occupancy as so                                     # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")


def visible(poses, bays, cam):
    """{frame_file: {bay_id, ...}} -- bays fully in shot, the scorer's own rule."""
    w, h, _ = so._intrinsics(cam)
    out = {}
    for pose in poses:
        got = set()
        for b in bays:
            ring = [so.project(x, y, pose, cam=cam) for x, y in b["ring"]]
            if any(p is None for p in ring):
                continue
            us = [p[0] for p in ring]
            vs = [p[1] for p in ring]
            if min(us) < 0 or max(us) >= w or min(vs) < 0 or max(vs) >= h:
                continue
            got.add(str(b["id"]))
        if got:
            out[pose["file"]] = got
    return out


def cover(vis, limit=0):
    """Greedy set cover -> [(frame, newly_covered), ...]."""
    todo = set().union(*vis.values()) if vis else set()
    picked = []
    while todo:
        best, gain = None, 0
        for f, s in vis.items():
            n = len(s & todo)
            if n > gain:
                best, gain = f, n
        if not best:
            break
        picked.append((best, gain))
        todo -= vis[best]
        if limit and len(picked) >= limit:
            break
    return picked, todo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--limit", type=int, default=0, help="cap the number of frames")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    cam = cameras.get(a.cam)
    poses = json.load(open(os.path.join(a.stills, "poses.json"), encoding="utf-8"))
    poses = [p for p in poses if "yaw" in p and "x" in p]
    bays = so.load_all_bays()
    vis = visible(poses, bays, cam)
    total = len(set().union(*vis.values())) if vis else 0
    print(f"{len(poses)} poses, {total} bays seen at least once")

    picked, missed = cover(vis, a.limit)
    os.makedirs(a.out, exist_ok=True)
    covered = 0
    for f, gain in picked:
        covered += gain
        shutil.copy2(os.path.join(a.stills, f), os.path.join(a.out, f))
        print(f"  {f}  +{gain:>3} bays  ({covered}/{total})")

    # The poses travel with the frames: the importer needs them, and a folder of
    # JPEGs with no poses cannot be turned back into ground coordinates by anyone.
    keep = {f for f, _ in picked}
    json.dump([p for p in poses if p["file"] in keep],
              open(os.path.join(a.out, "poses.json"), "w", encoding="utf-8"), indent=1)

    print(f"\n{len(picked)} frames -> {a.out}  (covers {covered}/{total} bays"
          + (f", {len(missed)} unreachable" if missed else "") + ")")
    print("Upload the JPEGs to makesense.ai, draw POLYGONS on the real bays, export "
          "as COCO or VGG JSON, then:\n"
          f"  python vision/import_annotations.py {a.out} --annot <export.json>")


if __name__ == "__main__":
    main()
