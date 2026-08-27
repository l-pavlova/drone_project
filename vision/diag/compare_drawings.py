"""Draw two sets of hand-drawn bay outlines onto the frame they came from.

    python vision/diag/compare_drawings.py vision/data/annot_0035 frame_0013 \
           --a vision/data/annot_0035/bay_outlines.json \
           --b vision/data/annot_0035/gate20.json \
           --out /tmp/cmp_0013.png

Two annotation passes over the same flight disagreed by a median 2.13 m, and no
distance statistic can say why: a drawing 2 m from another drawing is either the
same bay drawn differently or the NEIGHBOURING bay drawn correctly, and those call
for opposite responses. Proximity cannot separate them; the picture can.

Also draws the published Sofiaplan rectangles, because the question underneath is
which of the three -- if any -- is on the parking.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import cameras                                                  # noqa: E402
import score_occupancy as so                                    # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")


def load_rings(path):
    doc = json.load(open(path, encoding="utf-8"))
    bays = doc.get("bays") or doc
    return {str(k): [tuple(p) for p in v["ring"]] for k, v in bays.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("frame", help="e.g. frame_0013")
    ap.add_argument("--a", help="first outline set (drawn A)")
    ap.add_argument("--b", help="second outline set (drawn B)")
    ap.add_argument("--published", action="store_true",
                    help="also draw Sofiaplan's own rectangles")
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--out", required=True)
    ap.add_argument("--scale", type=float, default=0.5)
    args = ap.parse_args()

    from PIL import Image, ImageDraw

    name = args.frame if args.frame.endswith(".jpg") else args.frame + ".jpg"
    poses = json.load(open(os.path.join(args.stills, "poses.json"), encoding="utf-8"))
    pose = next((p for p in poses if p["file"] == name), None)
    if pose is None:
        sys.exit(f"no pose for {name}")
    cam = cameras.get(args.cam)

    img = Image.open(os.path.join(args.stills, name)).convert("RGB")
    dr = ImageDraw.Draw(img)

    layers = []
    if args.published:
        layers.append(("published", {b["id"]: b["ring"] for b in so.load_all_bays()},
                       (255, 210, 0)))
    if args.a:
        layers.append(("A", load_rings(args.a), (0, 170, 255)))
    if args.b:
        layers.append(("B", load_rings(args.b), (255, 60, 60)))

    w, h, _ = so._intrinsics(cam)
    for tag, rings, col in layers:
        n = 0
        for bid, ring in rings.items():
            px = [so.project(x, y, pose, cam=cam) for x, y in ring]
            if any(p is None for p in px):
                continue
            us = [p[0] for p in px]
            vs = [p[1] for p in px]
            if max(us) < 0 or min(us) >= w or max(vs) < 0 or min(vs) >= h:
                continue
            dr.line([tuple(p) for p in px] + [tuple(px[0])], fill=col, width=6)
            dr.text((sum(us) / len(us), sum(vs) / len(vs)), bid, fill=col)
            n += 1
        print(f"{tag}: {n} outline(s) in this frame  colour={col}")

    if args.scale != 1.0:
        img = img.resize((int(img.width * args.scale), int(img.height * args.scale)))
    img.save(args.out)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
