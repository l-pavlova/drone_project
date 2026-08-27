#!/usr/bin/env python3
"""Attribute the per-frame projection offset to something in the pose.

`paint_align.py` measures HOW FAR the projected bay outlines sit from the paint.
This says WHY: it rotates each frame's offset into along-track / cross-track
metres and regresses it against the candidate causes recorded in poses.json.

Run it after any change to the camera model, the gimbal, or the capture logic.
A model that fits leaves R2 near zero against every covariate, because there is
nothing systematic left to explain; a large R2 names the culprit directly.

    python vision/diag/offset_report.py fmi_block

That is how the 2026-08-20 fix was found. Against the old nadir projection:

    cross ~ alt*roll             slope -1.021   R2 0.994
    cross ~ alt*residual_roll    slope +7.194   R2 0.754
    along ~ alt*pitch            slope -0.003   R2 0.000
    along ~ alt*residual_pitch   slope +1.097   R2 0.427

Read that as: the lateral error was the FULL body roll at unit slope -- the
gimbal's roll compensation was not reaching the image at all -- while the
fore/aft error followed the RESIDUAL pitch, i.e. that joint's compensation was
working and only its few-mrad servo lag showed up. See
score_occupancy.camera_axes for what those two facts imply about the gimbal.

Weak frames are excluded (`--min-ncc`, `--min-bays`): one bay at a poor
correlation peak is a measurement, not evidence, and averaging it in hides the
signal.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))


def _parse_args():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("area", nargs="?", default="fmi_block")
    ap.add_argument("--min-ncc", type=float, default=0.40,
                    help="drop frames whose correlation peak is below this")
    ap.add_argument("--min-bays", type=int, default=2,
                    help="drop frames showing fewer bays than this")
    ap.add_argument("--max-shift", type=int, default=25)
    ap.add_argument("--bg-radius", type=int, default=9)
    return ap.parse_args()


args = _parse_args()
sys.argv = [sys.argv[0], args.area]          # paint_align/score_occupancy read argv[1]
sys.path.insert(0, HERE)
import paint_align as PA                     # noqa: E402
from paint_align import S                    # noqa: E402


def fit(xs, ys):
    """Least squares y = a*x + b; returns (a, b, R2)."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return 0.0, my, 0.0
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    ss = sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys))
    st = sum((y - my) ** 2 for y in ys)
    return a, b, (1 - ss / st) if st else 0.0


def residual_pitch(p):
    return p.get("pitch", 0.0) + p.get("cam_pitch", math.pi / 2) - math.pi / 2


def residual_roll(p):
    return p.get("roll", 0.0) + p.get("cam_roll", -p.get("roll", 0.0))


COVARIATES = [
    ("cross ~ alt*roll",          lambda p: p["alt"] * p.get("roll", 0.0),   "cross_m"),
    ("cross ~ alt*residual_roll", lambda p: p["alt"] * residual_roll(p),     "cross_m"),
    ("cross ~ alt*cam_roll",      lambda p: p["alt"] * p.get("cam_roll", 0.0), "cross_m"),
    ("along ~ alt*pitch",         lambda p: p["alt"] * p.get("pitch", 0.0),  "along_m"),
    ("along ~ alt*residual_pitch", lambda p: p["alt"] * residual_pitch(p),   "along_m"),
    ("along ~ ground speed",      lambda p: p.get("_speed", 0.0),            "along_m"),
    ("cross ~ ground speed",      lambda p: p.get("_speed", 0.0),            "cross_m"),
]


def main():
    out, poses = PA.load_run(args.area)
    bays = S.load_bays()
    by_idx = {S.pose_idx(p): p for p in poses}

    # ground speed from the neighbouring captures: a temporal lag would show up
    # as an offset proportional to it and aligned with the direction of travel
    order = sorted(by_idx)
    for a, b in zip(order, order[1:]):
        pa, pb = by_idx[a], by_idx[b]
        by_idx[b]["_speed"] = math.hypot(pb["x"] - pa["x"], pb["y"] - pa["y"])

    rows = []
    for i in order:
        pose = by_idx[i]
        m = PA.measure_frame(pose, bays, PA.frame_array(out, i),
                             args.max_shift, args.bg_radius)
        if m and m["bays"] >= args.min_bays and m["peak"] >= args.min_ncc:
            rows.append((pose, m))
    if len(rows) < 3:
        raise SystemExit(f"only {len(rows)} usable frames -- nothing to fit")

    mags = sorted(m["mag_m"] for _, m in rows)
    print(f"{args.area}: {len(rows)} usable frames "
          f"(of {len(order)}; ncc >= {args.min_ncc}, bays >= {args.min_bays})")
    print(f"offset magnitude: median {mags[len(mags) // 2]:.3f} m, "
          f"mean {sum(mags) / len(mags):.3f} m, worst {mags[-1]:.3f} m\n")

    print(f"{'covariate':<28} {'slope':>8} {'intercept':>10} {'R2':>7}")
    for label, xf, key in COVARIATES:
        xs = [xf(p) for p, _ in rows]
        if max(xs) - min(xs) < 1e-9:
            continue
        a, b, r2 = fit(xs, [m[key] for _, m in rows])
        print(f"{label:<28} {a:>+8.3f} {b:>+10.3f} {r2:>7.3f}")
    print("\nA well-modelled camera leaves every R2 near zero. A slope near "
          "+/-1 against\nan alt*angle term means that angle is displacing the "
          "image and the projection\nis not accounting for it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
