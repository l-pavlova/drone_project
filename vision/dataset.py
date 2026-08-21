"""Turn a flown survey into a labelled detection dataset - no hand labelling.

    python vision/dataset.py <survey_area> [--overlay N] [--yolo] [--crops]

The sim knows exactly where every car is (`sim/worlds/<area>.cars.json`, written
by generate_world.py) and exactly where the camera was for every frame
(`poses.json`). Put the two through the same `project()` the scorer uses and the
bounding boxes fall out - thousands of them, pixel-exact, for free, and they can
never disagree with the world because both come from the same placement loop.

That is the whole argument for training a detector on sim data: not that Webots
looks like Sofia, but that the labels are FREE and PERFECT here and expensive
and approximate everywhere else.

Three outputs, all optional except the manifest:
  * `boxes.json` - the manifest: per frame, the visible cars as pixel boxes.
  * `--yolo`     - the same thing as YOLO label .txt files next to an images.txt
                   listing the frames, so a training run can symlink rather than
                   copy 2000 PNGs.
  * `--crops`    - per-bay crops with occupied/free labels, for the crop
                   classifier baseline and for the real-footage labelling UI.
  * `--overlay N`- N frames with the boxes drawn on, which is how you CHECK the
                   labels rather than trusting them.

Boxes include the car's ROOF, not just its ground footprint. A car 12 m off
nadir at 30 m has its roof displaced ~0.5 m (9 px) outward from its footprint,
so a footprint-only box is systematically tight on exactly the cars furthest
from the image centre. Heights are published exterior heights, approximate to a
few cm - which is well under a pixel - and the `--overlay` check is what
confirms the result hugs the car.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from PIL import Image, ImageDraw

import score_occupancy as so

# Published exterior heights (m) for the Webots proto vehicles the generator
# parks. Only used to lift the roof rectangle; a few cm of error here is far
# below one pixel at survey altitude.
CAR_H = {
    "TeslaModel3Simple":          1.44,
    "BmwX5Simple":                1.75,
    "CitroenCZeroSimple":         1.61,
    "ToyotaPriusSimple":          1.47,
    "LincolnMKZSimple":           1.48,
    "RangeRoverSportSVRSimple":   1.80,
    "MercedesBenzSprinterSimple": 2.36,
}
MIN_VISIBLE = 0.55      # keep a box only if this much of it is inside the frame.
#   A car half out of shot is a real detection target; a car 5% in shot is a
#   label that teaches the model to hallucinate from a bumper.
MIN_PX = 8              # drop boxes smaller than this on either side


def rect_corners(cx, cy, ang, L, W):
    """The four corners of a body rectangle, in local metres.

    Deliberately re-implemented rather than imported from generate_world.py:
    importing that module BUILDS A WORLD as a side effect (tools/
    check_consistency.py documents the same constraint). Six lines is cheaper
    than the import, and check_consistency compares the two.
    """
    c, s = math.cos(ang), math.sin(ang)
    hl, hw = L / 2.0, W / 2.0
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def project_z(px, py, pz, pose):
    """project() for a point ABOVE the ground - the roof corners need it.

    score_occupancy.project() assumes z = 0 because bays are painted on the
    ground; the maths is otherwise identical, so this repeats only the one line
    that differs (the vertical offset from the camera).
    """
    def ang(key, default=0.0):
        val = pose.get(key)
        return default if val is None else val

    d, r, b = so.camera_axes(pose["yaw"], ang("roll"),
                             ang("pitch") + ang("cam_pitch", math.pi / 2) - math.pi / 2,
                             ang("cam_roll"))
    f = so.IMG_W / (2.0 * math.tan(so.FOV / 2.0))
    wx, wy, wz = px - pose["x"], py - pose["y"], pz - pose["alt"]
    z = wx * d[0] + wy * d[1] + wz * d[2]
    if z < 0.01:
        z = 0.01
    return (so.IMG_W / 2.0 + f * (wx * r[0] + wy * r[1] + wz * r[2]) / z,
            so.IMG_H / 2.0 + f * (wx * b[0] + wy * b[1] + wz * b[2]) / z)


def car_box(car, pose):
    """Axis-aligned pixel box of one car in one frame, or None if not usable."""
    corners = rect_corners(car["x"], car["y"], car["ang"], car["L"], car["W"])
    h = CAR_H.get(car["model"], 1.5)
    pts = [so.project(x, y, pose) for x, y in corners]
    pts += [project_z(x, y, h, pose) for x, y in corners]
    us = [p[0] for p in pts]
    vs = [p[1] for p in pts]
    x0, x1, y0, y1 = min(us), max(us), min(vs), max(vs)
    full = (x1 - x0) * (y1 - y0)
    if full <= 0:
        return None
    cx0, cx1 = max(0.0, x0), min(float(so.IMG_W), x1)
    cy0, cy1 = max(0.0, y0), min(float(so.IMG_H), y1)
    if cx1 - cx0 < MIN_PX or cy1 - cy0 < MIN_PX:
        return None
    if ((cx1 - cx0) * (cy1 - cy0)) / full < MIN_VISIBLE:
        return None
    return [round(cx0, 1), round(cy0, 1), round(cx1, 1), round(cy1, 1)]


def build(area):
    out_dir = os.path.join(so.ROOT, "sim", "output", area)
    poses = json.load(open(os.path.join(out_dir, "poses.json"), encoding="utf-8"))
    cars_file = os.path.join(so.ROOT, "sim", "worlds", f"{area}.cars.json")
    cars = json.load(open(cars_file, encoding="utf-8"))

    frames = []
    for pose in poses:
        idx = so.pose_idx(pose)
        boxes = []
        for car in cars:
            box = car_box(car, pose)
            if box:
                boxes.append({"bay_id": car["bay_id"], "model": car["model"],
                              "box": box})
        frames.append({"frame_idx": idx,
                       "image": f"frame_{idx:03d}.png",
                       "pose": {k: pose[k] for k in ("x", "y", "alt", "yaw")
                                if k in pose},
                       "boxes": boxes})
    return frames


def write_yolo(area, frames, dest):
    """YOLO-format labels + an images.txt pointing at the existing frames.

    Frames are NOT copied: a survey is up to 2000 PNGs and a dataset that
    duplicates them goes stale the moment the world is re-flown.
    """
    lab = os.path.join(dest, "labels")
    os.makedirs(lab, exist_ok=True)
    src = os.path.join(so.ROOT, "sim", "output", area)
    listing = []
    for fr in frames:
        stem = os.path.splitext(fr["image"])[0]
        with open(os.path.join(lab, stem + ".txt"), "w", encoding="utf-8") as fh:
            for b in fr["boxes"]:
                x0, y0, x1, y1 = b["box"]
                fh.write(f"0 {((x0 + x1) / 2) / so.IMG_W:.6f} "
                         f"{((y0 + y1) / 2) / so.IMG_H:.6f} "
                         f"{(x1 - x0) / so.IMG_W:.6f} "
                         f"{(y1 - y0) / so.IMG_H:.6f}\n")
        listing.append(os.path.join(src, fr["image"]))
    open(os.path.join(dest, "images.txt"), "w", encoding="utf-8").write(
        "\n".join(listing) + "\n")
    open(os.path.join(dest, "classes.txt"), "w", encoding="utf-8").write("car\n")


def write_crops(area, frames, dest):
    """Per-bay crops labelled occupied/free, for the crop-classifier baseline."""
    so.SURVEY_AREA = area
    so.OUT = os.path.join(so.ROOT, "sim", "output", area)
    so.GT_FILE = os.path.join(so.ROOT, "sim", "worlds",
                              "ground_truth.json" if area == "fmi_block"
                              else f"{area}.ground_truth.json")
    bays = so.load_bays()
    poses = {so.pose_idx(p): p for p in
             json.load(open(os.path.join(so.OUT, "poses.json"), encoding="utf-8"))}
    for label in ("occupied", "free"):
        os.makedirs(os.path.join(dest, "crops", label), exist_ok=True)
    n = 0
    for fr in frames:
        pose = poses[fr["frame_idx"]]
        img = Image.open(os.path.join(so.OUT, fr["image"])).convert("RGB")
        for b in bays:
            ring = [so.project(x, y, pose) for x, y in b["ring"]]
            us, vs = [p[0] for p in ring], [p[1] for p in ring]
            if min(us) < 0 or max(us) >= so.IMG_W or min(vs) < 0 or max(vs) >= so.IMG_H:
                continue        # partial bays make ambiguous crops
            label = "occupied" if b["occupied"] else "free"
            img.crop((int(min(us)), int(min(vs)),
                      int(max(us)) + 1, int(max(vs)) + 1)).save(
                os.path.join(dest, "crops", label,
                             f"{area}_f{fr['frame_idx']:03d}_b{b['id']}.png"))
            n += 1
    return n


def write_overlays(area, frames, dest, count):
    """Draw the boxes on the frames - the check that the labels are real.

    Spread across the flight rather than taken from the start: the first frames
    of a patrol are all one street, and labels that are wrong only at high yaw
    would pass a check that never leaves it.
    """
    os.makedirs(os.path.join(dest, "overlay"), exist_ok=True)
    src = os.path.join(so.ROOT, "sim", "output", area)
    withbox = [f for f in frames if f["boxes"]]
    if not withbox:
        return 0
    step = max(1, len(withbox) // count)
    for fr in withbox[::step][:count]:
        img = Image.open(os.path.join(src, fr["image"])).convert("RGB")
        img = img.resize((so.IMG_W * 3, so.IMG_H * 3), Image.NEAREST)
        draw = ImageDraw.Draw(img)
        for b in fr["boxes"]:
            x0, y0, x1, y1 = [c * 3 for c in b["box"]]
            draw.rectangle([x0, y0, x1, y1], outline=(0, 255, 0), width=2)
            draw.text((x0 + 2, y0 + 2), str(b["bay_id"]), fill=(0, 255, 0))
        img.save(os.path.join(dest, "overlay", f"overlay_{fr['frame_idx']:03d}.png"))
    return min(count, len(withbox[::step]))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    flags = [a for a in sys.argv[1:] if a.startswith("-")]
    if not args:
        raise SystemExit("usage: python vision/dataset.py <survey_area> "
                         "[--overlay N] [--yolo] [--crops]")
    area = args[0]
    n_overlay = 0
    if "--overlay" in sys.argv:
        i = sys.argv.index("--overlay")
        n_overlay = int(sys.argv[i + 1]) if i + 1 < len(sys.argv) else 6

    dest = os.path.join(so.ROOT, "vision", "datasets", area)
    os.makedirs(dest, exist_ok=True)
    frames = build(area)
    boxes = sum(len(f["boxes"]) for f in frames)
    json.dump({"survey_area": area, "img_w": so.IMG_W, "img_h": so.IMG_H,
               "frames": frames},
              open(os.path.join(dest, "boxes.json"), "w"), indent=1)
    print(f"{area}: {len(frames)} frames, {boxes} car boxes "
          f"({boxes / max(1, len(frames)):.1f} per frame) -> {dest}")

    if "--yolo" in flags:
        write_yolo(area, frames, dest)
        print(f"  yolo labels + images.txt written")
    if "--crops" in flags:
        n = write_crops(area, frames, dest)
        print(f"  {n} per-bay crops written")
    if n_overlay:
        n = write_overlays(area, frames, dest, n_overlay)
        print(f"  {n} overlay(s) written - LOOK AT THESE before training on it")


if __name__ == "__main__":
    main()
