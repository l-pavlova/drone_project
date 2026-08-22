#!/usr/bin/env python3
"""Which of `classify()`'s five tests actually earn their place, and does
normalising the intensity ones against a per-frame reference help?

    python vision/diag/classifier_ablation.py fmi_block_4st fmi_block_4st_sun
    python vision/diag/classifier_ablation.py --all      # every flown world

WHY THIS EXISTS. The 2026-08-21 shadow experiment showed `fmi_block_4st` falling
from 100% to 44.1% under a low sun on an otherwise identical scene, and the
cause was not shade: every free bay's core dropped from brightness 85 to 72,
under the absolute floor `T_BRIGHT_LO` of 82. Free cores sit at 85 in an 82-102
envelope, so that floor has a **3.5% margin** - which is the real finding. Any
fix that keeps an absolute-intensity test must estimate scene illumination to
within a couple of percent, and nothing will do that on real footage. So before
tuning a normalisation, measure whether the fragile test is needed at all.

HOW. Feature extraction is the expensive part and does not depend on the
classifier, so views are extracted once per world and cached; after that a
variant evaluates in milliseconds. The vote is a faithful copy of
`score_occupancy.main()`'s (strict majority, fuller views first) - the point is
to compare classifiers, so everything around them must stay identical.

The `current` variant is the harness's own self-test: it must reproduce each
world's committed accuracy, or the harness is measuring something else.
"""
import collections
import json
import math
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "vision"))

# score_occupancy reads sys.argv at import time to choose its survey area and
# builds module-level paths from it. Neutralise that: this harness drives the
# area itself and only wants the functions.
_argv, sys.argv = sys.argv, ["score_occupancy.py"]
import score_occupancy as so  # noqa: E402

sys.argv = _argv

CACHE = os.path.join(HERE, ".ablation_cache")


# ---------------------------------------------------------------- extraction

def frame_reference(gray):
    """Per-frame illumination estimates, all from one greyscale frame.

    Three candidates, kept together because choosing between them is the point:
      median - simple, but moves with how much dark road is in view
      p60    - biased towards the bright ground plane, so steadier
      mode   - the tone occupying the most pixels. The scene is three flat
               surfaces (ground ~121, road ~50, bay pad ~85) and the ground is a
               ~56% plurality, so the mode IS the ground's tone and is nearly
               independent of what else is in frame. Bin width 2 to survive
               dither.
    """
    hist, edges = np.histogram(gray, bins=128, range=(0, 256))
    k = int(hist.argmax())
    return {
        "f_median": float(np.median(gray)),
        "f_p60": float(np.percentile(gray, 60)),
        "f_mode": float((edges[k] + edges[k + 1]) / 2.0),
    }


# Per-channel mode of `fmi_block_4st`, i.e. what the ground plane looks like
# under the lighting every committed threshold was calibrated against.
REF_RGB = (121.0, 121.0, 121.0)


def channel_modes(img):
    """The dominant tone of each channel independently.

    Per CHANNEL and not just overall, because the low-sun world does not merely
    dim the scene - it tints it. Free-bay `core_chroma` RISES from 11.0 to 14.0
    as the light drops, past the 12.7 threshold, because a shallower sun means
    proportionally more of the surface's light comes from the tinted ambient
    sky. That is a change in the light's COLOUR, and no amount of intensity
    scaling can undo it; only a per-channel correction can.
    """
    out = []
    for c in range(3):
        hist, edges = np.histogram(img[:, :, c], bins=128, range=(0, 256))
        k = int(hist.argmax())
        out.append(float((edges[k] + edges[k + 1]) / 2.0))
    return out


def grey_world(img):
    """Correct the light's COLOUR CAST while leaving its level alone.

    The two failures are independent and deserve independent corrections: a
    shallower sun both dims the scene (brightness 85 -> 72) and tints it
    (chroma 11 -> 14). Equalising the channel means removes the tint and cannot
    disturb luminance, so unlike `white_balance` it has nothing to say about
    brightness and cannot mis-scale it when the reference is wrong.
    """
    f = img.astype(np.float32)
    means = [max(float(f[:, :, c].mean()), 1.0) for c in range(3)]
    target = sum(means) / 3.0
    for c in range(3):
        f[:, :, c] *= target / means[c]
    return np.clip(f, 0, 255).astype(np.uint8)


def white_balance(img):
    """Map this frame's ground plane onto the calibration world's ground plane.

    One multiply per channel, so it corrects cast and level together and is a
    no-op on the world the thresholds were tuned for. Everything downstream -
    every absolute cutoff in `region_stats` and every threshold in `classify` -
    then applies unchanged, which is the point: this is a normalisation, not a
    re-tuning.

    A channel whose mode is near zero would blow up the gain, so the scale is
    clamped; a frame that dark has no usable bay in it anyway.
    """
    modes = channel_modes(img)
    f = img.astype(np.float32)
    for c in range(3):
        gain = REF_RGB[c] / max(modes[c], 8.0)
        f[:, :, c] *= min(gain, 4.0)
    return np.clip(f, 0, 255).astype(np.uint8)


def extract(area, norm=False):
    """[(bay_id, gt, [view_feature, ...]), ...] for one flown survey area.

    `norm` white-balances each frame to the calibration illuminant before any
    feature is measured, so the pixel cutoffs buried in `region_stats` (dark
    < 55, paint > 140) are normalised too - which the threshold-scaling variants
    cannot reach.
    """
    out = os.path.join(ROOT, "sim", "output", area)
    gt_name = "ground_truth.json" if area == "fmi_block" else f"{area}.ground_truth.json"
    poses = json.load(open(os.path.join(out, "poses.json"), encoding="utf-8"))
    gt = json.load(open(os.path.join(ROOT, "sim", "worlds", gt_name), encoding="utf-8"))
    feats = json.load(open(os.path.join(ROOT, "data", "block_bays.geojson"),
                           encoding="utf-8"))["features"]
    bays = []
    for f in feats:
        bid = str(f["properties"].get("id"))
        if bid not in gt:
            continue
        ring = [so.to_enu(lon, lat) for lon, lat in f["geometry"]["coordinates"][0][:-1]]
        bays.append({"id": bid, "ring": ring, "occupied": gt[bid]})

    # Views are gathered per bay but frames are read per frame: holding 1976
    # decoded frames would be gigabytes, so walk frames in order and fan each
    # one out to the bays it sees.
    views = collections.defaultdict(list)
    for pose in poses:
        for b in bays:
            ring_px = [so.project(x, y, pose) for x, y in b["ring"]]
            us = [p[0] for p in ring_px]
            vs = [p[1] for p in ring_px]
            if max(us) < 0 or min(us) >= so.IMG_W or max(vs) < 0 or min(vs) >= so.IMG_H:
                continue
            off = math.hypot(sum(us) / 4 - so.IMG_W / 2, sum(vs) / 4 - so.IMG_H / 2)
            views[so.pose_idx(pose)].append((b["id"], off, ring_px))

    per_bay = collections.defaultdict(list)
    for i in sorted(views):
        path = os.path.join(out, f"frame_{i:03d}.png")
        if not os.path.exists(path):
            continue
        img = np.array(Image.open(path).convert("RGB"))
        ref = frame_reference(img.astype(np.float32).mean(axis=2))
        if norm == "wb":
            img = white_balance(img)
        elif norm == "grey":
            img = grey_world(img)

        batch = []
        for bid, off, ring_px in views[i]:
            feat = so.bay_features(img, ring_px)
            if feat is not None:
                batch.append((bid, dict(feat, off=off, **ref)))
        # A reference tied to a KNOWN MATERIAL, computed after the fact: the
        # median core tone of every bay this frame can see. The bay pad is one
        # material present in every frame that matters, so unlike the frame mode
        # it cannot be hijacked by how much road or grass happens to be in
        # shot. It survives parked cars because it is a median - the same
        # robustness argument the occupancy vote already relies on - and fails
        # only if a clear majority of the bays in one frame are occupied.
        if batch:
            med = float(np.median([f["core_brightness"] for _b, f in batch]))
            for _b, f in batch:
                f["f_bays"] = med
        for bid, f in batch:
            per_bay[bid].append(f)

    return [(b["id"], b["occupied"], per_bay.get(b["id"], [])) for b in bays]


def load(area, norm=False, refresh=False):
    os.makedirs(CACHE, exist_ok=True)
    tag = f".{norm}" if norm else ""
    path = os.path.join(CACHE, f"{area}{tag}.json")
    if os.path.exists(path) and not refresh:
        return json.load(open(path, encoding="utf-8"))
    rows = extract(area, norm)
    json.dump(rows, open(path, "w", encoding="utf-8"))
    return rows


# ---------------------------------------------------------------- variants
#
# The calibration reference is `fmi_block_4st`'s ground plane, measured at 121.0
# by all three estimators. A relative variant divides by it, so under the
# lighting the thresholds were calibrated for it is EXACTLY today's classifier -
# a strict generalisation rather than a re-tuning.
REF_CAL = 121.0

# Each reference has its own calibration value, because each measures a
# different surface: the frame estimators land on the ground plane (121.0),
# while `f_bays` measures the bay pad itself (84.7). Dividing by the right one
# is what makes every relative variant a no-op on the calibration world.
REF_FOR = {"f_mode": 121.0, "f_p60": 121.0, "f_median": 121.0, "f_bays": 84.7}


def make_variant(drop=(), scale_by=None):
    """A classify() with tests dropped and/or intensity thresholds normalised.

    `drop` names tests to omit. `scale_by` is a frame-reference key; when given,
    every intensity-valued threshold is multiplied by ref/REF_CAL. The two
    FRACTION thresholds are not scaled - a fraction is already dimensionless -
    but the pixel cutoffs that PRODUCE those fractions live inside
    `region_stats` and stay absolute here. That is a real limitation of this
    harness and is reported rather than hidden: it means the `dark` and `paint`
    rows understate what a full implementation could do.
    """
    def f(feat):
        k = (feat[scale_by] / REF_FOR[scale_by]) if scale_by else 1.0
        tests = {
            "chroma": feat["core_chroma"] > so.T_CHROMA * k,
            "dark": feat["core_dark_frac"] > so.T_DARK,
            "paint": feat["core_paint_frac"] > so.T_PAINT,
            "bright": not (so.T_BRIGHT_LO * k < feat["core_brightness"] < so.T_BRIGHT_HI * k),
            "std": feat["core_std"] > so.T_STD * k,
        }
        return any(v for name, v in tests.items() if name not in drop)
    return f


# (classifier, needs_white_balanced_features)
VARIANTS = {
    "current (absolute)": (make_variant(), False),
    "drop bright": (make_variant(drop=("bright",)), False),
    "drop chroma": (make_variant(drop=("chroma",)), False),
    "drop dark": (make_variant(drop=("dark",)), False),
    "drop paint": (make_variant(drop=("paint",)), False),
    "drop std": (make_variant(drop=("std",)), False),
    "rel/f_mode": (make_variant(scale_by="f_mode"), False),
    "rel/f_p60": (make_variant(scale_by="f_p60"), False),
    "rel/f_median": (make_variant(scale_by="f_median"), False),
    "rel/f_bays": (make_variant(scale_by="f_bays"), False),
    # Normalise the IMAGE, leave the classifier untouched.
    "white-balanced (per-ch mode)": (make_variant(), "wb"),
    # Cast-only correction, luminance untouched: fixes chroma without being
    # able to mis-scale brightness.
    "grey-world": (make_variant(), "grey"),
    "grey-world + rel/f_bays": (make_variant(scale_by="f_bays"), "grey"),
    "grey-world + rel/f_mode": (make_variant(scale_by="f_mode"), "grey"),
}


def evaluate(rows, fn):
    """Same vote as score_occupancy.main(): strict majority over usable views."""
    tp = tn = fp = fn_ = uncovered = 0
    for _bid, gt, views in rows:
        if not views:
            uncovered += 1
            continue
        preds = [fn(v) for v in views]
        pred = sum(preds) * 2 > len(preds)
        if pred and gt:
            tp += 1
        elif not pred and not gt:
            tn += 1
        elif pred and not gt:
            fp += 1
        else:
            fn_ += 1
    n = tp + tn + fp + fn_
    return {"tp": tp, "tn": tn, "fp": fp, "fn": fn_, "n": n, "uncovered": uncovered,
            "acc": (100.0 * (tp + tn) / n) if n else float("nan")}


ALL_AREAS = ["fmi_block", "fmi_block_4st", "fmi_block_4st_lp",
             "fmi_block_4st_shd", "fmi_block_4st_sun", "fmi_block_4st_sh"]


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    refresh = "--refresh" in sys.argv
    areas = ALL_AREAS if ("--all" in sys.argv or not args) else args
    areas = [a for a in areas
             if os.path.exists(os.path.join(ROOT, "sim", "output", a, "poses.json"))]

    modes = sorted({wb for _fn, wb in VARIANTS.values()}, key=str)
    table = {}
    for area in areas:
        rows = {}
        for m in modes:
            print(f"extracting {area}{' (' + m + ')' if m else ''} ...", flush=True)
            rows[m] = load(area, m, refresh)
        table[area] = {name: evaluate(rows[wb], fn) for name, (fn, wb) in VARIANTS.items()}

    w = max(len(v) for v in VARIANTS)
    print("\naccuracy %, with FP/FN beneath each\n")
    print(" " * (w + 2) + "".join(f"{a.replace('fmi_block', 'fb'):>20s}" for a in areas))
    for name in VARIANTS:
        cells = ""
        for area in areas:
            r = table[area][name]
            cells += f"{r['acc']:>9.1f} {('%d/%d' % (r['fp'], r['fn'])):>10s}"
        print(f"{name:<{w}}  {cells}")
    print("\nFP = free bay called occupied, FN = a car missed.")
    print("Every world's `current` row must match its committed number.")


if __name__ == "__main__":
    main()
