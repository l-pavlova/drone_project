"""Zero-shot detector on REAL drone stills, where there is no ground truth.

    python vision/detect_real.py <stills_dir> [--conf 0.10] [--imgsz 640]
                                 [--tile N] [--limit N] [--overlay N]

`detect_baseline.py` is the sim's version of this question and scores two
numbers, because the sim hands out exact boxes and a ground_truth.json.  Real
footage has neither, so this script deliberately reports something weaker and
honest: WHAT the model fires on, how often, how confidently, and at what size --
plus overlays, because with no labels the eye is the only judge available.

The comparison that matters is against the sim result on record: COCO-pretrained
YOLOv8m detects 0 of 52 cars in Webots frames, and upscaling does not help, so
the failure was attributed to DOMAIN (COCO cars are photographed from the side)
rather than resolution.  These stills are 3840x2160 at 11.7 mm/px -- a car is
~385 px long -- and they are real photographs, so they separate the two
explanations: if the model sees cars here, the sim's renderer/resolution is the
problem; if it still does not, nadir itself is.

--tile N runs inference on an NxN grid of overlapping crops instead of the whole
frame.  Ultralytics letterboxes a 3840-wide frame down to --imgsz, so at the
default 640 a 385 px car arrives as 64 px; tiling is how you ask the question at
native scale without a 3840 imgsz that will not fit in memory.
"""
import argparse
import json
import os
import sys
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from boxes import iou, read_yolo  # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

# COCO vehicle classes. `car` is what the supervisor's pipeline filters to, but
# a nadir car misread as a truck or a bus is a very different result from one
# not seen at all, so all four are counted and reported separately.
VEHICLE = {2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}
# ...but keyed by NAME, because a fine-tuned single-class model numbers its own
# classes from 0 and `car` there is COCO's `person`. Matching on the index made
# every per-frame count, size statistic and overlay colour wrong for our own
# detector while the `--labels` scoring (which already matched on the name) was
# right -- so the bug was invisible in the reported recall/precision.
VEHICLE_NAMES = set(VEHICLE.values())


def is_vehicle(cls_idx, names):
    return names[cls_idx] in VEHICLE_NAMES


def tiles(w, h, n, overlap=0.2):
    """NxN overlapping crops. Overlap so a car on a tile seam is whole somewhere."""
    tw, th = int(w / (n - (n - 1) * overlap)), int(h / (n - (n - 1) * overlap))
    sx, sy = int(tw * (1 - overlap)), int(th * (1 - overlap))
    for j in range(n):
        for i in range(n):
            x0, y0 = min(i * sx, w - tw), min(j * sy, h - th)
            yield x0, y0, x0 + tw, y0 + th


def nms(dets, thr=0.5):
    """Merge duplicates from overlapping tiles. dets: [x0,y0,x1,y1,conf,cls]."""
    out = []
    for d in sorted(dets, key=lambda d: -d[4]):
        keep = True
        for k in out:
            ix0, iy0 = max(d[0], k[0]), max(d[1], k[1])
            ix1, iy1 = min(d[2], k[2]), min(d[3], k[3])
            if ix1 <= ix0 or iy1 <= iy0:
                continue
            inter = (ix1 - ix0) * (iy1 - iy0)
            ar = ((d[2] - d[0]) * (d[3] - d[1]) + (k[2] - k[0]) * (k[3] - k[1]) - inter)
            if ar > 0 and inter / ar > thr:
                keep = False
                break
        if keep:
            out.append(d)
    return out


def score(records, labels, stills, thr=0.5):
    """Detection recall/precision against hand-drawn YOLO labels.

    Only frames that HAVE a label file are scored -- a frame nobody labelled is
    not evidence of an empty frame, and counting it as one would invent false
    positives out of unlabelled ground truth.

    Two recalls are reported.  `car` alone is the number the supervisor's
    pipeline would get.  `any vehicle class` folds in truck/bus/motorcycle,
    because a car called a truck is a different failure from a car not seen --
    the box is right and only the label is wrong, which a fine-tune fixes
    trivially.
    """
    import cv2
    scored = tp_car = tp_veh = 0
    gt_total = det_car = 0
    for r in records:
        lp = os.path.join(labels, os.path.splitext(r["file"])[0] + ".txt")
        if not os.path.exists(lp):
            continue
        img = cv2.imread(os.path.join(stills, r["file"]))
        H, W = img.shape[:2]
        gts = read_yolo(lp, W, H)
        scored += 1
        gt_total += len(gts)
        cars = [d for d in r["dets"] if d["cls"] == "car"]
        vehs = [d for d in r["dets"] if d["cls"] in VEHICLE.values()]
        det_car += len(cars)
        for g in gts:
            hit_c = any(iou(g, d["box"]) >= thr for d in cars)
            hit_v = any(iou(g, d["box"]) >= thr for d in vehs)
            tp_car += hit_c
            tp_veh += hit_v
    if not scored:
        print("\n  --labels: no label file matched any frame")
        return
    print(f"\n  scored against {scored} labelled frames, {gt_total} true cars "
          f"(IoU >= {thr})")
    print(f"    recall, class 'car' only : {tp_car}/{gt_total} = "
          f"{100 * tp_car / gt_total:.1f}%")
    print(f"    recall, any vehicle class: {tp_veh}/{gt_total} = "
          f"{100 * tp_veh / gt_total:.1f}%")
    if det_car:
        print(f"    precision of 'car' boxes : {tp_car}/{det_car} = "
              f"{100 * tp_car / det_car:.1f}%  "
              f"({det_car - tp_car} fired on something that is not a car)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills", help="directory of frame_*.jpg (tools/dji_stills.py)")
    ap.add_argument("--model", default="yolov8m.pt")
    ap.add_argument("--conf", type=float, default=0.10)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--tile", type=int, default=1, help="NxN tiled inference")
    ap.add_argument("--limit", type=int, help="only the first N stills")
    ap.add_argument("--overlay", type=int, default=0, help="write N annotated frames")
    ap.add_argument("--out", help="where overlays/report go (default <stills>/detect)")
    ap.add_argument("--labels", help="dir of YOLO .txt ground truth; scores "
                    "detection recall/precision on the frames that have one")
    a = ap.parse_args()

    import cv2
    from ultralytics import YOLO

    files = sorted(f for f in os.listdir(a.stills) if f.startswith("frame_")
                   and f.lower().endswith((".jpg", ".png")))
    if a.limit:
        files = files[:a.limit]
    if not files:
        sys.exit(f"no frame_*.jpg in {a.stills}")
    out = a.out or os.path.join(a.stills, "detect")
    os.makedirs(out, exist_ok=True)

    model = YOLO(a.model)
    names = model.names
    print(f"{a.model} on {len(files)} stills, conf {a.conf}, imgsz {a.imgsz}"
          f"{f', {a.tile}x{a.tile} tiles' if a.tile > 1 else ''}")

    cls_count = Counter()          # every class the model fires on, not just vehicles
    per_frame = []
    confs = defaultdict(list)
    sizes = []
    records = []

    for n, fn in enumerate(files):
        img = cv2.imread(os.path.join(a.stills, fn))
        h, w = img.shape[:2]
        crops = list(tiles(w, h, a.tile)) if a.tile > 1 else [(0, 0, w, h)]
        dets = []
        for (x0, y0, x1, y1) in crops:
            r = model.predict(img[y0:y1, x0:x1], conf=a.conf, imgsz=a.imgsz,
                              verbose=False)[0]
            for b in r.boxes:
                bx = b.xyxy[0].tolist()
                dets.append([bx[0] + x0, bx[1] + y0, bx[2] + x0, bx[3] + y0,
                             float(b.conf[0]), int(b.cls[0])])
        if a.tile > 1:
            dets = nms(dets)

        veh = 0
        for d in dets:
            c = d[5]
            cls_count[c] += 1
            confs[c].append(d[4])
            if is_vehicle(c, names):
                veh += 1
                sizes.append((d[2] - d[0], d[3] - d[1]))
        per_frame.append(veh)
        records.append({"file": fn, "dets": [
            {"box": [round(v, 1) for v in d[:4]], "conf": round(d[4], 3),
             "cls": names[d[5]]} for d in dets]})

        if a.overlay and n < a.overlay:
            vis = img.copy()
            for d in dets:
                col = (0, 220, 0) if is_vehicle(d[5], names) else (0, 140, 255)
                cv2.rectangle(vis, (int(d[0]), int(d[1])), (int(d[2]), int(d[3])),
                              col, 4)
                cv2.putText(vis, f"{names[d[5]]} {d[4]:.2f}",
                            (int(d[0]), int(d[1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.4, col, 3)
            cv2.imwrite(os.path.join(out, "ov_" + fn),
                        cv2.resize(vis, (w // 2, h // 2)),
                        [cv2.IMWRITE_JPEG_QUALITY, 88])
        if (n + 1) % 20 == 0:
            print(f"  {n + 1}/{len(files)} ... {sum(per_frame)} vehicle dets so far",
                  flush=True)

    tot_veh = sum(per_frame)
    print(f"\n{tot_veh} vehicle detections over {len(files)} stills "
          f"({tot_veh / len(files):.2f} per frame); "
          f"{sum(1 for v in per_frame if v == 0)} stills with none")
    if sizes:
        ws = sorted(s[0] for s in sizes)
        hs = sorted(s[1] for s in sizes)
        print(f"  box size median {ws[len(ws) // 2]:.0f} x {hs[len(hs) // 2]:.0f} px "
              f"(a 4.5 m car at 30 m is ~385 px)")
    print("\n  class breakdown (all classes, not just vehicles):")
    for c, k in cls_count.most_common(12):
        cs = sorted(confs[c])
        tag = "  <- vehicle" if is_vehicle(c, names) else ""
        print(f"    {names[c]:16s} {k:5d}   conf med {cs[len(cs) // 2]:.2f} "
              f"max {cs[-1]:.2f}{tag}")

    if a.labels:
        score(records, a.labels, a.stills)

    rep = os.path.join(out, f"detections_conf{a.conf}_sz{a.imgsz}_t{a.tile}.json")
    json.dump(records, open(rep, "w"), indent=1)
    print(f"\n  -> {rep}")
    if a.overlay:
        print(f"  -> {a.overlay} overlays in {out}")


if __name__ == "__main__":
    main()
