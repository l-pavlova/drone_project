"""Validate and eyeball hand-drawn YOLO labels.

    python vision/check_labels.py vision/data/dji_0035 [--overlay N] [--split S]

Hand labels are the one thing in this project that cannot be regenerated, and
the one thing with no upstream check: the sim's labels come from the same loop
that placed the cars, so they cannot disagree with the world, but a human with a
mouse can put a box anywhere.  So this does what `vision/dataset.py --overlay`
does for the sim -- draws the labels back onto the frames -- plus the format
checks a hand-made file needs and a generated one does not.

A missing .txt and an empty .txt mean different things and are reported
separately: missing = nobody has looked at this frame yet, empty = a human
looked and there were no cars.  Only the second is usable.
"""
import argparse
import os
import sys
from collections import Counter

sys.stdout.reconfigure(encoding="utf-8")

IMG_EXT = (".jpg", ".jpeg", ".png")
# A 4.5 m car at 30 m over a 45 m footprint is ~0.10 of the frame width. These
# bounds are loose sanity rails around that, not a spec -- a van at the frame
# edge showing its whole flank is legitimately bigger.
MIN_FRAC, MAX_FRAC = 0.01, 0.45


def check_split(root, split, overlay, out):
    idir = os.path.join(root, "images", split)
    ldir = os.path.join(root, "labels", split)
    if not os.path.isdir(idir):
        return None
    imgs = sorted(f for f in os.listdir(idir) if f.lower().endswith(IMG_EXT))
    missing, empty, boxes, bad = [], [], 0, []
    per_img = {}

    for fn in imgs:
        lp = os.path.join(ldir, os.path.splitext(fn)[0] + ".txt")
        if not os.path.exists(lp):
            missing.append(fn)
            continue
        lines = [l.strip() for l in open(lp, encoding="utf-8") if l.strip()]
        if not lines:
            empty.append(fn)
        rows = []
        for n, l in enumerate(lines, 1):
            f = l.split()
            if len(f) != 5:
                bad.append(f"{fn}:{n} has {len(f)} fields, expected 5")
                continue
            try:
                cls = int(f[0])
                cx, cy, w, h = (float(v) for v in f[1:])
            except ValueError:
                bad.append(f"{fn}:{n} not numeric: {l!r}")
                continue
            if cls != 0:
                bad.append(f"{fn}:{n} class {cls}, only 0 (car) is valid")
            for name, v in (("cx", cx), ("cy", cy), ("w", w), ("h", h)):
                if not 0.0 <= v <= 1.0:
                    bad.append(f"{fn}:{n} {name}={v} outside 0..1 "
                               f"(are these pixels instead of normalised?)")
            if w <= 0 or h <= 0:
                bad.append(f"{fn}:{n} zero-area box")
            elif not (MIN_FRAC <= w <= MAX_FRAC and MIN_FRAC <= h <= MAX_FRAC):
                bad.append(f"{fn}:{n} box {w:.3f}x{h:.3f} of frame -- "
                           f"suspicious for a car (expect ~0.10)")
            if cx - w / 2 < -0.01 or cx + w / 2 > 1.01 or \
               cy - h / 2 < -0.01 or cy + h / 2 > 1.01:
                bad.append(f"{fn}:{n} box extends outside the image")
            rows.append((cx, cy, w, h))
        boxes += len(rows)
        per_img[fn] = rows

    labelled = len(imgs) - len(missing)
    print(f"\n[{split}] {len(imgs)} images, {labelled} labelled, "
          f"{len(missing)} not yet, {len(empty)} deliberately empty")
    print(f"        {boxes} boxes"
          + (f", {boxes / labelled:.1f} per labelled image" if labelled else ""))
    if missing:
        print(f"        missing: {', '.join(missing[:6])}"
              + (f" ... (+{len(missing) - 6})" if len(missing) > 6 else ""))
    if bad:
        print(f"  {len(bad)} PROBLEM(S):")
        for b in bad[:25]:
            print(f"    {b}")
        if len(bad) > 25:
            print(f"    ... (+{len(bad) - 25} more)")
    elif labelled:
        print("        format OK")

    if overlay:
        import cv2
        os.makedirs(out, exist_ok=True)
        drawn = 0
        for fn, rows in per_img.items():
            if drawn >= overlay:
                break
            img = cv2.imread(os.path.join(idir, fn))
            H, W = img.shape[:2]
            for (cx, cy, w, h) in rows:
                x0, y0 = int((cx - w / 2) * W), int((cy - h / 2) * H)
                x1, y1 = int((cx + w / 2) * W), int((cy + h / 2) * H)
                cv2.rectangle(img, (x0, y0), (x1, y1), (0, 220, 0), 5)
            cv2.putText(img, f"{fn}  {len(rows)} car(s)", (30, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 220, 0), 4)
            cv2.imwrite(os.path.join(out, f"lab_{split}_{fn}"),
                        cv2.resize(img, (W // 2, H // 2)),
                        [cv2.IMWRITE_JPEG_QUALITY, 88])
            drawn += 1
        print(f"        -> {drawn} overlays in {out}")
    return len(imgs), labelled, boxes, len(bad)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root", help="dataset dir holding images/ and labels/")
    ap.add_argument("--overlay", type=int, default=0,
                    help="draw the labels back onto N frames per split")
    ap.add_argument("--split", help="only this split")
    a = ap.parse_args()

    root = os.path.abspath(a.root)
    splits = [a.split] if a.split else ["train", "val"]
    out = os.path.join(root, "check")
    tot = Counter()
    for s in splits:
        r = check_split(root, s, a.overlay, out)
        if r:
            tot["imgs"] += r[0]; tot["lab"] += r[1]
            tot["boxes"] += r[2]; tot["bad"] += r[3]

    print(f"\ntotal: {tot['lab']}/{tot['imgs']} images labelled, "
          f"{tot['boxes']} boxes, {tot['bad']} problem(s)")
    if tot["bad"]:
        print("Fix the problems above before training -- a bad label is worse "
              "than a missing one, because it is silently learned.")
    sys.exit(1 if tot["bad"] else 0)


if __name__ == "__main__":
    main()
