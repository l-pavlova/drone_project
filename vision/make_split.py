"""Build a labelling-ready dataset from a stills directory, with a SPATIAL split.

    python vision/make_split.py <stills_dir> --out vision/data/<name>
                                [--every 4] [--val-from N | --suggest]
                                [--skip LABELS_DIR] [--scale 0.5]

Hop 2 of the pipeline (docs/training.md §5b). It existed only as inline session
code until 2026-08-22, which made the single most important methodological choice
in the dataset -- the train/val split -- unreproducible from the repo.

**The split must be SPATIAL, not random, and this script exists to enforce it.**
A survey flight revisits its own ground: the 0035 flight runs east, doubles back
along the same street, then heads northwest, and frames 55-85 sit within 3-6 m of
frames 5-30 -- the same parked cars from a slightly different angle. Split those
at random and one photo of a car lands in train while another lands in val, so
the model scores brilliantly by memorising it. Everything downstream -- every
recall figure, every decision about whether to label more -- is then measuring
nothing.

So the split is by *leg of the flight*, and the script REPORTS the separation it
achieved rather than asserting it: `--val-from N` puts frames >= N in val, and
the printed `min train-val distance` is the check. A value below the camera
footprint (~25 m along track) means train and val see the same ground and the
split is not doing its job.

`--suggest` scans every candidate split point and reports the separation each
would give, so the choice is made from the geometry rather than by eye.

Frames are COPIED, not moved: the stills directory stays the archive.
"""
import argparse
import json
import math
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from boxes import frame_no  # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")


def load_poses(stills):
    p = os.path.join(stills, "poses.json")
    if not os.path.exists(p):
        sys.exit(f"no poses.json in {stills} -- run tools/dji_stills.py first")
    recs = json.load(open(p, encoding="utf-8"))
    return {r["file"]: r for r in recs if "x" in r}


def separation(poses, train, val):
    """Smallest distance between any train frame and any val frame, in metres.

    This is the number that says whether the split is real. Compare it against
    the along-track camera footprint (~25 m at 30 m altitude with this lens):
    below that, a train frame and a val frame are photographing the same tarmac.
    """
    best = float("inf")
    for a in train:
        pa = poses.get(a)
        if not pa:
            continue
        for b in val:
            pb = poses.get(b)
            if not pb:
                continue
            d = math.hypot(pa["x"] - pb["x"], pa["y"] - pb["y"])
            best = min(best, d)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills", help="directory of frame_*.jpg + poses.json")
    ap.add_argument("--out", help="dataset dir to create (omit with --suggest)")
    ap.add_argument("--every", type=int, default=4,
                    help="keep every Nth frame (default 4). At 1 Hz the stills "
                         "are ~4 m apart against a 25 m footprint -- about 6 "
                         "looks at the same ground -- so labelling every frame "
                         "is mostly re-labelling the same cars.")
    ap.add_argument("--val-from", type=int,
                    help="frames with an index >= this go to val")
    ap.add_argument("--val-range",
                    help="A-B (inclusive) -- val is this frame range instead of a "
                         "tail. Needed when the ground a flight has NOT been seen "
                         "on sits in the middle of it, which is the normal case "
                         "for a second flight over a block already surveyed.")
    ap.add_argument("--buffer-m", type=float, default=0.0,
                    help="drop TRAIN frames within this many metres of any val "
                         "frame. A spatial split needs a buffer zone: val and "
                         "train blocks that are adjacent in TIME are ~4 m apart "
                         "in SPACE at 1 Hz, so the boundary leaks even though the "
                         "blocks themselves are far apart. 25 (the along-track "
                         "footprint) is the principled value.")
    ap.add_argument("--exclude", action="append", default=[],
                    help="A-B (inclusive) -- drop these frames entirely. Repeat "
                         "for several ranges: a real flight's non-nadir stretches "
                         "are not one contiguous block (flight 0074 tilts up at "
                         "81-83 and again from 111). Use it for those, and for "
                         "frames that overlap ANOTHER dataset's val set: training "
                         "on them leaks that benchmark.")
    ap.add_argument("--suggest", action="store_true",
                    help="report the separation every candidate split would give, "
                         "and write nothing")
    ap.add_argument("--skip", action="append", default=[],
                    help="labels dir(s) whose frames are already handled")
    ap.add_argument("--prefix", default="",
                    help="prepend to every copied filename, e.g. 'f0034_'. Frame "
                         "stems must be unique ACROSS flights, and every flight's "
                         "stills start at frame_0000 -- so a second flight without "
                         "a prefix silently collides on merge.")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="resize copies (YOLO labels are normalised, so a "
                         "downscaled copy yields identical label numbers)")
    a = ap.parse_args()

    poses = load_poses(a.stills)
    done = set()
    for d in a.skip:
        if os.path.isdir(d):
            done |= {os.path.splitext(f)[0] for f in os.listdir(d)
                     if f.endswith(".txt")}
    files = sorted(f for f in os.listdir(a.stills)
                   if f.startswith("frame_") and f.lower().endswith(".jpg")
                   and os.path.splitext(f)[0] not in done)
    kept = files[::a.every]
    print(f"{len(files)} candidate frames, keeping every {a.every} -> {len(kept)}"
          + (f" (skipping {len(done)} already labelled)" if done else ""))

    def rng(spec):
        lo, hi = (int(v) for v in spec.split("-"))
        return lo, hi

    for spec in a.exclude:
        lo, hi = rng(spec)
        dropped = [f for f in kept if lo <= frame_no(f) <= hi]
        kept = [f for f in kept if not (lo <= frame_no(f) <= hi)]
        print(f"  excluded {len(dropped)} frame(s) in {spec}")

    if a.suggest:
        print("\n  split point   train/val   min train-val distance")
        for cut in range(0, frame_no(kept[-1]) + 1, max(1, a.every)):
            tr = [f for f in kept if frame_no(f) < cut]
            va = [f for f in kept if frame_no(f) >= cut]
            if len(va) < 3 or len(tr) < 5:
                continue
            sep = separation(poses, tr, va)
            flag = "  <-- ok" if sep >= 25 else ("  (too close)" if sep < 25 else "")
            print(f"  >= {cut:<10d} {len(tr):3d}/{len(va):<3d}    {sep:8.1f} m{flag}")
        print("\n  25 m is the along-track camera footprint: below it, train and "
              "val see the same ground.")
        return

    if a.val_from is None and not a.val_range:
        sys.exit("--val-from N or --val-range A-B is required "
                 "(or use --suggest to choose)")
    if not a.out:
        sys.exit("--out is required")


    if a.val_range:
        lo, hi = rng(a.val_range)
        val = [f for f in kept if lo <= frame_no(f) <= hi]
        train = [f for f in kept if not (lo <= frame_no(f) <= hi)]
    else:
        train = [f for f in kept if frame_no(f) < a.val_from]
        val = [f for f in kept if frame_no(f) >= a.val_from]

    if a.buffer_m > 0:
        keep_tr = []
        for t in train:
            pt = poses.get(t)
            if pt and min((math.hypot(pt["x"] - poses[v]["x"],
                                      pt["y"] - poses[v]["y"])
                           for v in val if v in poses), default=1e9) < a.buffer_m:
                continue
            keep_tr.append(t)
        print(f"  buffer {a.buffer_m:.0f} m dropped "
              f"{len(train) - len(keep_tr)} train frame(s) near val")
        train = keep_tr
    sep = separation(poses, train, val)

    for s in ("train", "val"):
        os.makedirs(os.path.join(a.out, "images", s), exist_ok=True)
        os.makedirs(os.path.join(a.out, "labels", s), exist_ok=True)

    if a.scale != 1.0:
        import cv2
    for split, group in (("train", train), ("val", val)):
        for fn in group:
            src = os.path.join(a.stills, fn)
            dst = os.path.join(a.out, "images", split, a.prefix + fn)
            if a.scale == 1.0:
                shutil.copy2(src, dst)
            else:
                img = cv2.imread(src)
                h, w = img.shape[:2]
                cv2.imwrite(dst, cv2.resize(img, (int(w * a.scale), int(h * a.scale)),
                                            interpolation=cv2.INTER_AREA),
                            [cv2.IMWRITE_JPEG_QUALITY, 90])

    open(os.path.join(a.out, "classes.txt"), "w", encoding="utf-8").write("car\n")
    open(os.path.join(a.out, "labels.txt"), "w", encoding="utf-8").write("car\n")
    open(os.path.join(a.out, "data.yaml"), "w", encoding="utf-8").write(
        f"path: {os.path.abspath(a.out).replace(os.sep, '/')}\n"
        "train: images/train\nval: images/val\nnames:\n  0: car\n")
    open(os.path.join(a.out, ".gitignore"), "w", encoding="utf-8").write(
        "images/\ncheck/\nreview/\npreview/\npending/\n*.cache\n")
    json.dump({"stills": os.path.abspath(a.stills), "every": a.every,
               "val_from": a.val_from, "val_range": a.val_range,
               "exclude": list(a.exclude), "buffer_m": a.buffer_m,
               "scale": a.scale, "prefix": a.prefix,
               "train": train, "val": val,
               "min_train_val_distance_m": round(sep, 2)},
              open(os.path.join(a.out, "split.json"), "w", encoding="utf-8"), indent=1)

    print(f"  train {len(train)} / val {len(val)}  ->  {a.out}")
    if not val or not train:
        # An empty side makes `sep` inf, and printing that as ">= 25 m, genuinely
        # spatial" would claim a separation nothing was measured against. A
        # train-only flight is a legitimate thing to build (its frames feed
        # training and it is benchmarked against ANOTHER flight's hand-drawn val
        # set) -- but it must say so rather than borrow the reassurance.
        print("  NO val split -- train-only dataset. Benchmark it against a "
              "hand-drawn val set from a different flight, and check that flight's "
              "val frames are >= 25 m from these before training on them.")
        print(f"  split recorded in {a.out}/split.json")
        print("\n  next: pre-label with vision/prelabel.py, correct by hand, then")
        print(f"        python vision/check_labels.py {a.out} --split train --overlay 6")
        return
    print(f"  min train-val distance: {sep:.1f} m", end="")
    if sep < 25:
        print("  ** WARNING: below the ~25 m along-track footprint -- train and "
              "val may see the same cars, which makes the val score meaningless. "
              "Re-run with --suggest and pick a better split point. **")
    else:
        print("  (>= the ~25 m footprint: the split is genuinely spatial)")
    print(f"  split recorded in {a.out}/split.json")
    print("\n  next: label images/<split> per vision/data/dji_0035/LABELING.md, then")
    print(f"        python vision/check_labels.py {a.out} --overlay 6")


if __name__ == "__main__":
    main()
