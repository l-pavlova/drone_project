"""Merge labelled datasets into one training set, preserving each source's split.

    python vision/merge_datasets.py vision/data/dji_0035 vision/data/dji_0035_r2 \
           --out vision/data/dji_merged [--benchmark vision/data/dji_0035]

Hop 10 of the pipeline (docs/training.md §5b). Like hop 2 this existed only as
inline session code until 2026-08-22.

Two things it must get right, and both are about provenance rather than about
copying files:

**Splits are preserved, never recomputed.** Each source set's train/val division
was chosen spatially for that flight (see make_split.py). Pooling the frames and
re-splitting would silently undo that work and let near-duplicate frames land on
opposite sides.

**Pre-labelled frames are marked.** A set produced by `prelabel.py` and corrected
by hand carries the model's own opinion: where the model was confidently wrong in
a plausible way, a reviewer may have agreed, and training on that reinforces the
bias. Frames labelled from scratch are the only ones free of that circularity, so
`--benchmark` records which they are in `provenance.json`, and
`--benchmark-only-val` can force them into val to keep an uncontaminated
evaluation set.

Measured consequence, 2026-08-22: on the merged val set (34 frames) `probe2` beat
`merged1`, and on the 9 uncontaminated frames it did not -- because 25 of those 34
carry labels `probe2` itself drew. Without the provenance record that comparison
could not have been made at all.
"""
import argparse
import json
import os
import shutil
import sys

sys.stdout.reconfigure(encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sources", nargs="+", help="dataset dirs to merge")
    ap.add_argument("--out", required=True)
    ap.add_argument("--benchmark", action="append", default=[],
                    help="source dir(s) whose labels are hand-drawn from scratch; "
                         "recorded in provenance.json as the clean benchmark")
    ap.add_argument("--benchmark-only-val", action="store_true",
                    help="force every benchmark frame into val, so the "
                         "uncontaminated set is never trained on")
    a = ap.parse_args()

    for s in ("train", "val"):
        os.makedirs(os.path.join(a.out, "images", s), exist_ok=True)
        os.makedirs(os.path.join(a.out, "labels", s), exist_ok=True)

    bench = {os.path.abspath(b) for b in a.benchmark}
    prov = {}
    counts = {"train": [0, 0], "val": [0, 0]}
    collisions = []

    for src in a.sources:
        src_abs = os.path.abspath(src)
        is_bench = src_abs in bench
        for split in ("train", "val"):
            ldir = os.path.join(src, "labels", split)
            if not os.path.isdir(ldir):
                continue
            for lf in sorted(f for f in os.listdir(ldir) if f.endswith(".txt")):
                stem = os.path.splitext(lf)[0]
                img = None
                for ext in (".jpg", ".jpeg", ".png"):
                    cand = os.path.join(src, "images", split, stem + ext)
                    if os.path.exists(cand):
                        img = cand
                        break
                if img is None:
                    print(f"  ! no image for {src}/{split}/{lf} -- skipped")
                    continue
                dst_split = "val" if (is_bench and a.benchmark_only_val) else split
                if stem in prov:
                    collisions.append(stem)
                    continue
                shutil.copy2(img, os.path.join(a.out, "images", dst_split,
                                               os.path.basename(img)))
                shutil.copy2(os.path.join(ldir, lf),
                             os.path.join(a.out, "labels", dst_split, lf))
                n = sum(1 for line in open(os.path.join(ldir, lf),
                                           encoding="utf-8") if line.strip())
                prov[stem] = {"source": src.replace(os.sep, "/"),
                              "source_split": split, "split": dst_split,
                              "boxes": n,
                              "hand_drawn": is_bench}
                counts[dst_split][0] += 1
                counts[dst_split][1] += n

    open(os.path.join(a.out, "classes.txt"), "w", encoding="utf-8").write("car\n")
    open(os.path.join(a.out, "labels.txt"), "w", encoding="utf-8").write("car\n")
    open(os.path.join(a.out, "data.yaml"), "w", encoding="utf-8").write(
        f"path: {os.path.abspath(a.out).replace(os.sep, '/')}\n"
        "train: images/train\nval: images/val\nnames:\n  0: car\n")
    open(os.path.join(a.out, ".gitignore"), "w", encoding="utf-8").write(
        "images/\ncheck/\n*.cache\n")
    json.dump({"sources": [s.replace(os.sep, "/") for s in a.sources],
               "benchmark_sources": sorted(bench),
               "benchmark_only_val": a.benchmark_only_val,
               "frames": prov},
              open(os.path.join(a.out, "provenance.json"), "w", encoding="utf-8"),
              indent=1)

    hand = sum(1 for v in prov.values() if v["hand_drawn"])
    hand_val = sum(1 for v in prov.values() if v["hand_drawn"] and v["split"] == "val")
    print(f"merged {len(a.sources)} set(s) -> {a.out}")
    for k, (f, b) in counts.items():
        print(f"  {k}: {f} frames, {b} boxes")
    print(f"  total: {sum(v[0] for v in counts.values())} frames, "
          f"{sum(v[1] for v in counts.values())} boxes")
    print(f"  hand-drawn (clean benchmark): {hand} frames, {hand_val} of them in val")
    if collisions:
        print(f"  ! {len(collisions)} frame name(s) appeared in more than one source "
              f"and only the first was kept: {', '.join(collisions[:6])}"
              + (" ..." if len(collisions) > 6 else ""))
        print("    Frame stems must be unique across sources -- two flights both "
              "starting at frame_0000 will collide. Prefix them per flight.")
    print(f"  provenance -> {a.out}/provenance.json")


if __name__ == "__main__":
    main()
