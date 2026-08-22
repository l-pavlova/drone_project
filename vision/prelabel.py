"""Pre-label one split of a dataset with a trained model, for a human to CORRECT.

    python vision/prelabel.py <dataset_dir> --split train
                              --model vision/runs/<run>/weights/best.pt
                              [--conf 0.25] [--imgsz 1024] [--overlay N]

These are PREDICTIONS, not labels. Correcting a box is roughly 3-5x faster than
drawing one, which is the entire point -- measured on flight 0035: 62 corrections
against 386 boxes that would otherwise have been drawn from scratch, about 0.6
edits per frame on frames averaging 3.9 cars.

**It writes into ONE split, and that is the whole design.** The val half of a
benchmark must stay hand-drawn: where a model is confidently wrong in a plausible
way a reviewer tends to agree, so pre-labelling the frames you then measure
against bakes the model's own opinion into its yardstick. Measured 2026-08-22 --
on the 34-frame merged val set `probe2` appeared to beat `merged1`, and on the 9
frames it had never pre-labelled it did not.

`--conf` trades deletions against misses. At the operating point (0.25) a
reviewer deletes few and draws a few. Dropping to 0.10 finds a few percent more
cars at three times the false boxes, which is not obviously a win: a missing car
is one label short, while a wrong box left in actively teaches the next model
that tarmac is a car.

Run `vision/make_split.py` first -- it creates the dataset and chooses the split
from the flight geometry. This script deliberately does NOT split anything; an
earlier version did, with a hardcoded frame index that was correct only for
flight 0035 and would have silently mis-split any other flight.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

sys.stdout.reconfigure(encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dataset", help="dataset dir holding images/<split>")
    ap.add_argument("--split", default="train",
                    help="which split to pre-label (default train). Pre-labelling "
                         "'val' destroys the benchmark -- see the module docstring.")
    ap.add_argument("--model", default="vision/runs/probe2/weights/best.pt")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=1024,
                    help="MUST match the model's training size; below it the model "
                         "quietly loses recall")
    ap.add_argument("--skip", action="append", default=[],
                    help="labels dir(s) whose frames are already done by hand")
    ap.add_argument("--overlay", type=int, default=0,
                    help="write N annotated previews for quick triage")
    a = ap.parse_args()

    import cv2
    from ultralytics import YOLO

    img_dir = os.path.join(a.dataset, "images", a.split)
    lab_dir = os.path.join(a.dataset, "labels", a.split)
    if not os.path.isdir(img_dir):
        sys.exit(f"no {img_dir} -- run vision/make_split.py first")
    os.makedirs(lab_dir, exist_ok=True)

    done = set()
    for d in a.skip:
        if os.path.isdir(d):
            done |= {os.path.splitext(f)[0] for f in os.listdir(d)
                     if f.endswith(".txt")}
    files = sorted(f for f in os.listdir(img_dir)
                   if f.lower().endswith((".jpg", ".jpeg", ".png"))
                   and os.path.splitext(f)[0] not in done)
    if not files:
        sys.exit("nothing left to pre-label")

    if a.split == "val":
        print("!! pre-labelling the VAL split -- this set can no longer be used "
              "as a clean benchmark for this model or its descendants.")

    model = YOLO(a.model)
    print(f"{a.model} over {len(files)} frames of {a.dataset}/images/{a.split} "
          f"at conf {a.conf}, imgsz {a.imgsz}"
          + (f" (skipping {len(done)} already done)" if done else ""))

    out_prev = os.path.join(a.dataset, "preview")
    if a.overlay:
        os.makedirs(out_prev, exist_ok=True)

    boxes = empty = 0
    for n, fn in enumerate(files):
        img = cv2.imread(os.path.join(img_dir, fn))
        H, W = img.shape[:2]
        r = model.predict(img, conf=a.conf, imgsz=a.imgsz, verbose=False)[0]
        rows = []
        for b in r.boxes:
            x0, y0, x1, y1 = b.xyxy[0].tolist()
            rows.append(f"0 {((x0 + x1) / 2) / W:.6f} {((y0 + y1) / 2) / H:.6f} "
                        f"{(x1 - x0) / W:.6f} {(y1 - y0) / H:.6f}")
        # An empty file, never a missing one: missing reads as "nobody looked",
        # empty as "checked, no cars" -- and ultralytics trains on the latter.
        with open(os.path.join(lab_dir, os.path.splitext(fn)[0] + ".txt"),
                  "w", encoding="utf-8") as fh:
            fh.write("\n".join(rows) + ("\n" if rows else ""))
        boxes += len(rows)
        empty += not rows

        if a.overlay and n < a.overlay:
            vis = img.copy()
            for b in r.boxes:
                x0, y0, x1, y1 = (int(v) for v in b.xyxy[0].tolist())
                cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 220, 0), 5)
                cv2.putText(vis, f"{float(b.conf[0]):.2f}", (x0, y0 - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 220, 0), 4)
            cv2.imwrite(os.path.join(out_prev, "pre_" + fn),
                        cv2.resize(vis, (W // 2, H // 2)),
                        [cv2.IMWRITE_JPEG_QUALITY, 88])
        if (n + 1) % 25 == 0:
            print(f"  {n + 1}/{len(files)} ... {boxes} boxes", flush=True)

    print(f"\n  {len(files)} frames pre-labelled into {lab_dir}: {boxes} boxes, "
          f"{boxes / len(files):.1f} per frame, {empty} with none")
    if a.overlay:
        print(f"  {a.overlay} previews in {out_prev}")
    print("  THESE ARE PREDICTIONS. Correct them, then:")
    print(f"    python vision/check_labels.py {a.dataset} --overlay 6")


if __name__ == "__main__":
    main()
