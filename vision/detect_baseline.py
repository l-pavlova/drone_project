"""Zero-shot detector baseline: what does an off-the-shelf model see in our frames?

    python vision/detect_baseline.py <survey_area> [--model yolov8m.pt] [--scale N]
                                     [--conf 0.25] [--limit N] [--overlay N]

The thesis needs a learned detector, and the first honest question is not "how do
we train one" but "how far does a pretrained one already get". The supervisor's
`kyovchev/ground-vehicles-localization` runs exactly this configuration -
`YOLO("yolov8m.pt")`, COCO class 2 (car), no fine-tuning - on real Sofia drone
footage, so it is also the directly comparable prior result.

Two numbers come out, and they are different questions:

  * **detection** - precision/recall against the sim's EXACT boxes
    (`vision/dataset.py`, projected from the world's own car placements). This
    says whether the model can see a Webots car from 30 m.
  * **occupancy** - the product-level number. Detections are unprojected to the
    ground, assigned to the bay they land in, and voted across frames exactly as
    `score_occupancy.py` votes its classifier, then scored against
    `ground_truth.json`. This is the number that is comparable with the
    heuristic's, and the only one that says anything about the actual task.

A model can do well at one and badly at the other: missing a car that three
other frames also see costs almost no occupancy accuracy, while one detection
landing in the neighbouring bay costs two bays at once.

**COCO cars are photographed from the side.** Ours are 72x29 px seen from
directly above, in a renderer. Expect this to be hard, and expect `--scale` to
matter: the sim camera is 400x240 (a deliberate, documented limitation against
the 8 MP IMX219 the hardware plan calls for), so upscaling before inference is
the cheapest way to find out whether resolution or domain is the binding
constraint.
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from PIL import Image

import score_occupancy as so
from detect_occupancy import ASSIGN_MAX_M, bay_votes_from_dets

CAR_CLASS = 2           # COCO 'car', the class the supervisor's pipeline filters to
#   ASSIGN_MAX_M and the detections->bays rule now live in detect_occupancy.py,
#   because the real-footage path needs the identical rule and a second copy of
#   it would be the exact duplication hazard TODO #3 exists to retire. This
#   file's occupancy numbers are what check the extraction did not change it.


def iou(a, b):
    ix0, iy0 = max(a[0], b[0]), max(a[1], b[1])
    ix1, iy1 = min(a[2], b[2]), min(a[3], b[3])
    if ix1 <= ix0 or iy1 <= iy0:
        return 0.0
    inter = (ix1 - ix0) * (iy1 - iy0)
    ar = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ar if ar > 0 else 0.0


def detect(model, img, scale):
    """Run the detector on one frame, returning boxes in ORIGINAL pixel coords."""
    if scale != 1:
        img = img.resize((img.width * scale, img.height * scale), Image.LANCZOS)
    res = model(np.array(img), classes=[CAR_CLASS], verbose=False)[0]
    out = []
    for box, conf in zip(res.boxes.xyxy.tolist(), res.boxes.conf.tolist()):
        out.append(([c / scale for c in box], conf))
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if not args:
        raise SystemExit("usage: python vision/detect_baseline.py <survey_area> "
                         "[--model M] [--scale N] [--conf C] [--limit N]")
    area = args[0]

    def opt(name, default, cast=str):
        return cast(sys.argv[sys.argv.index(name) + 1]) if name in sys.argv else default

    model_name = opt("--model", "yolov8m.pt")
    scale = opt("--scale", 1, int)
    conf_th = opt("--conf", 0.25, float)
    limit = opt("--limit", 0, int)

    so.SURVEY_AREA = area
    so.OUT = os.path.join(so.ROOT, "sim", "output", area)
    so.GT_FILE = os.path.join(so.ROOT, "sim", "worlds",
                              "ground_truth.json" if area == "fmi_block"
                              else f"{area}.ground_truth.json")
    poses = json.load(open(os.path.join(so.OUT, "poses.json"), encoding="utf-8"))
    if limit:
        poses = poses[:limit]
    bays = so.load_bays()
    truth = {f["frame_idx"]: f["boxes"] for f in
             json.load(open(os.path.join(so.ROOT, "vision", "datasets", area,
                                         "boxes.json"), encoding="utf-8"))["frames"]}

    from ultralytics import YOLO
    model = YOLO(model_name)
    print(f"{area}: {model_name}, scale x{scale}, conf {conf_th}, "
          f"{len(poses)} frames")

    tp = fp = fn = 0
    votes = {}          # bay id -> [bool, ...] one per frame that could see it
    for pose in poses:
        idx = so.pose_idx(pose)
        path = os.path.join(so.OUT, f"frame_{idx:03d}.png")
        img = Image.open(path).convert("RGB")
        dets = [(b, c) for b, c in detect(model, img, scale) if c >= conf_th]

        # ---- detection scoring against the sim's exact boxes
        gt_boxes = [b["box"] for b in truth.get(idx, [])]
        used = set()
        for box, _ in dets:
            best, best_j = 0.0, None
            for j, g in enumerate(gt_boxes):
                if j in used:
                    continue
                v = iou(box, g)
                if v > best:
                    best, best_j = v, j
            if best >= 0.5:
                used.add(best_j)
                tp += 1
            else:
                fp += 1
        fn += len(gt_boxes) - len(used)

        # ---- occupancy: where did each detection LAND, and in whose bay?
        for s in bay_votes_from_dets(dets, bays, pose):
            votes.setdefault(s["bay_id"], []).append(s["occupied"])

    prec = tp / (tp + fp) if tp + fp else 0.0
    rec = tp / (tp + fn) if tp + fn else 0.0
    print(f"\ndetection  (IoU>=0.5 vs the sim's exact boxes)")
    print(f"  TP={tp} FP={fp} FN={fn}   precision {100*prec:.1f}%  recall {100*rec:.1f}%")

    gt = {b["id"]: b["occupied"] for b in bays}
    otp = otn = ofp = ofn = 0
    for bay, vs in votes.items():
        pred = sum(vs) * 2 > len(vs)
        actual = gt[bay]
        otp += pred and actual
        otn += (not pred) and (not actual)
        ofp += pred and not actual
        ofn += (not pred) and actual
    n = otp + otn + ofp + ofn
    print(f"\noccupancy  (majority vote per bay, the number comparable with classify())")
    print(f"  {n} bays voted, {len(bays) - n} never fully in shot")
    print(f"  TP={otp} TN={otn} FP={ofp} FN={ofn}   "
          f"accuracy {100 * (otp + otn) / n:.1f}%" if n else "  no bays voted")
    print(f"\n  (heuristic classify() scores 100% on this world - see CLAUDE.md)")


if __name__ == "__main__":
    main()
