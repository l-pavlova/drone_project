#!/usr/bin/env python3
"""Measure, per captured frame, how far the PROJECTED bay outlines sit from the
paint the camera actually saw.

Why this exists
---------------
The earlier diagnostic sampled a 1-D brightness profile across each bay's short
axis and located the painted line by `argmax`. That measurement had a known
systematic bias (measured half-width came out 0.7-0.9 px under predicted), and
being 1-D it could not tell a translation from a rotation, nor give the
*direction* of the offset -- which is the one number that discriminates between
the remaining causes (along-track => timing, world-fixed => pose, radial from
the image centre => optics).

This replaces it with a 2-D sub-pixel registration:

  observed  a RIDGE response -- grey minus a local box mean, kept only where
            the pixel is near-achromatic.  A thin bright line survives; a car
            roof (bigger than the box, often coloured) does not, so the
            estimate is not dragged around by parked cars.
  predicted the four painted line rectangles of every visible bay, rasterised
            from `score_occupancy.project()` verbatim.
  match     normalised cross-correlation over integer shifts, then a parabolic
            fit on the 3x3 neighbourhood of the peak for sub-pixel precision.

`--self-test` injects a known shift into the OBSERVED image (bilinear) and
checks the estimator recovers it.  A measurement that cannot pass a
known-answer test is worthless -- same principle as the mutation testing in
tools/check_consistency.py.

    python vision/diag/paint_align.py fmi_block                # every frame
    python vision/diag/paint_align.py fmi_block --frame 26     # one frame
    python vision/diag/paint_align.py fmi_block --self-test

Sign convention: the reported offset is PAINT MINUS PREDICTION, i.e. how far
the model has to move to land on the real paint.
"""
import argparse
import json
import math
import os
import re
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
VISION = os.path.dirname(HERE)
ROOT = os.path.dirname(VISION)


def _parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("area", nargs="?", default="fmi_block")
    ap.add_argument("--frame", type=int, default=None, help="only this frame index")
    ap.add_argument("--max-shift", type=int, default=25, help="search radius, px")
    ap.add_argument("--bg-radius", type=int, default=9,
                    help="box radius for the ridge background, px")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--json", metavar="PATH", default=None,
                    help="also write the per-frame results here")
    ap.add_argument("--overlay", metavar="DIR", default=None,
                    help="write a magnified check image per frame: the capture "
                         "with the projected outlines in RED and the same "
                         "outlines moved by the measured offset in GREEN")
    ap.add_argument("--zoom", type=int, default=4, help="overlay magnification")
    return ap.parse_args(argv)


args = _parse_args()

# score_occupancy resolves the survey area from sys.argv[1] AT IMPORT TIME, so
# hand it the area we were given before importing it.
sys.argv = [sys.argv[0], args.area]
sys.path.insert(0, VISION)
import score_occupancy as S            # noqa: E402


def line_width_m():
    """LINE_W from the generator, read rather than copied.

    Importing generate_world.py builds a world as a side effect, so this parses
    it as text -- the same compromise tools/check_consistency.py makes.
    """
    src = open(os.path.join(ROOT, "sim", "generate_world.py"), encoding="utf-8").read()
    m = re.search(r"^LINE_W\s*=\s*([0-9.]+)", src, re.M)
    if not m:
        raise SystemExit("could not find LINE_W in sim/generate_world.py")
    return float(m.group(1))


LINE_W = line_width_m()


# ---------------------------------------------------------------- observed


def box_mean(a, r):
    """Mean over a (2r+1) square, edge-replicated. Separable, via cumsums."""
    p = np.pad(a, r, mode="edge")
    c = np.cumsum(p, axis=0)
    c = np.vstack([np.zeros((1, c.shape[1]), dtype=c.dtype), c])
    a1 = (c[2 * r + 1:, :] - c[:-2 * r - 1, :]) / (2 * r + 1)
    c = np.cumsum(a1, axis=1)
    c = np.hstack([np.zeros((c.shape[0], 1), dtype=c.dtype), c])
    return (c[:, 2 * r + 1:] - c[:, :-2 * r - 1]) / (2 * r + 1)


def ridge(img_arr, bg_radius):
    """Thin-bright-achromatic-line response of a frame."""
    px = img_arr.astype(np.float32)
    grey = px.mean(axis=2)
    mx, mn = px.max(axis=2), px.min(axis=2)
    # achromatic weight, using the same 45-count chroma scale region_stats uses
    # to call a pixel "paint" rather than bodywork
    achroma = np.clip(1.0 - (mx - mn) / 45.0, 0.0, 1.0)
    return np.clip(grey - box_mean(grey, bg_radius), 0.0, None) * achroma


# --------------------------------------------------------------- predicted


def bay_line_quads(ring):
    """The four painted line rectangles of a bay, in ENU metres.

    generate_world.bay_lines() insets each line by LINE_W/2 so its OUTER edge
    lands on the bay rectangle; the geojson ring is therefore the outside of the
    paint, not its centreline. Reproduce that here or the prediction sits half a
    line width wide on all four sides.
    """
    (x0, y0), (x1, y1), (x2, y2), (x3, y3) = ring[:4]
    cx = (x0 + x1 + x2 + x3) / 4.0
    cy = (y0 + y1 + y2 + y3) / 4.0
    ex, ey = x1 - x0, y1 - y0                     # first edge
    fx, fy = x3 - x0, y3 - y0                     # adjacent edge
    le, lf = math.hypot(ex, ey), math.hypot(fx, fy)
    ux, uy = ex / le, ey / le
    vx, vy = fx / lf, fy / lf
    he, hf = le / 2.0, lf / 2.0                   # half extents along u, v
    h = LINE_W / 2.0

    def rect(cu, cv, hu, hv):
        return [(cx + ux * (cu + su * hu) + vx * (cv + sv * hv),
                 cy + uy * (cu + su * hu) + vy * (cv + sv * hv))
                for su, sv in ((-1, -1), (+1, -1), (+1, +1), (-1, +1))]

    return [rect(0.0, +(hf - h), he, h),          # the two long sides
            rect(0.0, -(hf - h), he, h),
            rect(+(he - h), 0.0, h, hf),          # and the two ends
            rect(-(he - h), 0.0, h, hf)]


SS = 4          # rasterisation supersampling, see predicted_mask


def predicted_mask(bays, pose):
    """Rasterise every visible bay's painted outline for this pose.

    ANTI-ALIASED, and that is not cosmetic. A painted line is ~2 px wide, so a
    hard binary rasterisation changes in whole-pixel jumps as the geometry moves
    a fraction of a pixel; the correlation surface then has a quantised peak and
    the parabolic fit on it is biased. Measured: whole-pixel injected shifts
    recovered to 0.001 px either way, but fractional ones were out by up to
    0.43 px until the mask was supersampled and averaged down.
    """
    img = Image.new("F", (S.IMG_W * SS, S.IMG_H * SS), 0.0)
    d = ImageDraw.Draw(img)
    drawn = 0
    for b in bays:
        ring_px = [S.project(x, y, pose) for x, y in b["ring"]]
        us = [p[0] for p in ring_px]
        vs = [p[1] for p in ring_px]
        if max(us) < 0 or min(us) >= S.IMG_W or max(vs) < 0 or min(vs) >= S.IMG_H:
            continue
        drawn += 1
        for quad in bay_line_quads(b["ring"]):
            d.polygon([(u * SS, v * SS) for u, v in
                       (S.project(x, y, pose) for x, y in quad)], fill=1.0)
    fine = np.array(img, dtype=np.float32)
    coarse = fine.reshape(S.IMG_H, SS, S.IMG_W, SS).mean(axis=(1, 3))
    return coarse, drawn


# ------------------------------------------------------------ registration


def shift_int(a, du, dv):
    """Translate an array by whole pixels (+du right, +dv down), zero filled."""
    out = np.zeros_like(a)
    if dv < 0:
        ys, yd = slice(-dv, None), slice(None, a.shape[0] + dv)
    else:
        ys, yd = slice(None, a.shape[0] - dv), slice(dv, None)
    if du < 0:
        xs, xd = slice(-du, None), slice(None, a.shape[1] + du)
    else:
        xs, xd = slice(None, a.shape[1] - du), slice(du, None)
    out[yd, xd] = a[ys, xs]
    return out


def bilinear_shift(a, du, dv):
    """Sub-pixel translate, for the self-test's known-answer injection."""
    fu, fv = int(math.floor(du)), int(math.floor(dv))
    su, sv = du - fu, dv - fv
    return ((1 - su) * (1 - sv) * shift_int(a, fu, fv)
            + su * (1 - sv) * shift_int(a, fu + 1, fv)
            + (1 - su) * sv * shift_int(a, fu, fv + 1)
            + su * sv * shift_int(a, fu + 1, fv + 1))


def register(pred, obs, max_shift):
    """Sub-pixel (du, dv) that moves `pred` onto `obs`, by normalised
    cross-correlation. Returns (du, dv, peak_ncc, clipped).

    Integer search first, then the sub-pixel part by SEARCHING actual shifted
    correlations rather than fitting a parabola to the three samples around the
    peak. The correlation of a thin-line pattern is far too sharply peaked for a
    parabola: fitting one recovered whole-pixel injected shifts perfectly (the
    bias cancels when the surface merely re-indexes) but was out by up to
    0.43 px on fractional ones, which is the regime the whole measurement lives
    in. Two refinement passes, coarse then fine, cost ~0.3 s per frame.
    """
    obs_e = math.sqrt(float((obs * obs).sum())) or 1.0

    def ncc(p):
        e = math.sqrt(float((p * p).sum()))
        return -np.inf if e == 0 else float((p * obs).sum()) / (e * obs_e)

    n = 2 * max_shift + 1
    corr = np.full((n, n), -np.inf)
    for i, dv in enumerate(range(-max_shift, max_shift + 1)):
        for j, du in enumerate(range(-max_shift, max_shift + 1)):
            corr[i, j] = ncc(shift_int(pred, du, dv))
    i, j = np.unravel_index(int(np.argmax(corr)), corr.shape)
    peak = float(corr[i, j])
    du, dv = float(j - max_shift), float(i - max_shift)
    if i in (0, n - 1) or j in (0, n - 1):        # peak clipped: no sub-pixel
        return du, dv, peak, True

    for step, span in ((0.2, 1.0), (0.02, 0.2)):
        best = (peak, du, dv)
        k = int(round(span / step))
        for a in range(-k, k + 1):
            for b in range(-k, k + 1):
                cu, cv = du + a * step, dv + b * step
                c = ncc(bilinear_shift(pred, cu, cv))
                if c > best[0]:
                    best = (c, cu, cv)
        peak, du, dv = best
    return du, dv, peak, False


# ------------------------------------------------------------------ driver


def px_per_m(pose):
    return S.IMG_W / (2.0 * pose["alt"] * math.tan(S.FOV / 2.0))


def to_drone_frame(du, dv, pose):
    """Pixel offset -> (along, cross) metres, inverting score_occupancy.project:
    u = W/2 - cross*k and v = H/2 - along*k."""
    k = px_per_m(pose)
    return -dv / k, -du / k


def measure_frame(pose, bays, img_arr, max_shift, bg_radius, inject=(0.0, 0.0)):
    pred, drawn = predicted_mask(bays, pose)
    if drawn == 0 or pred.sum() == 0:
        return None
    obs = ridge(img_arr, bg_radius)
    if inject != (0.0, 0.0):
        obs = bilinear_shift(obs, *inject)
    du, dv, peak, clipped = register(pred, obs, max_shift)
    along, cross = to_drone_frame(du, dv, pose)
    return {"frame": S.pose_idx(pose), "bays": drawn,
            "du_px": du, "dv_px": dv, "peak": peak, "clipped": clipped,
            "along_m": along, "cross_m": cross,
            "mag_m": math.hypot(along, cross),
            "mag_px": math.hypot(du, dv)}


def write_overlay(path, img_arr, bays, pose, du, dv, zoom):
    """Eyeball check: the capture with the projected paint outlines drawn where
    the model puts them (red) and where the measured offset says they are
    (green). If the green outlines sit on the paint and the red ones do not,
    the number in the table is real."""
    img = Image.fromarray(img_arr).resize(
        (S.IMG_W * zoom, S.IMG_H * zoom), Image.NEAREST)
    d = ImageDraw.Draw(img)
    for b in bays:
        ring_px = [S.project(x, y, pose) for x, y in b["ring"]]
        if all(u < 0 or u >= S.IMG_W for u, _ in ring_px) or \
           all(v < 0 or v >= S.IMG_H for _, v in ring_px):
            continue
        for quad in bay_line_quads(b["ring"]):
            q = [S.project(x, y, pose) for x, y in quad]
            d.polygon([(u * zoom, v * zoom) for u, v in q], outline="red")
            d.polygon([((u + du) * zoom, (v + dv) * zoom) for u, v in q],
                      outline="lime")
    img.save(path)


def load_run(area):
    out = os.path.join(ROOT, "sim", "output", area)
    poses = json.load(open(os.path.join(out, "poses.json"), encoding="utf-8"))
    return out, poses


def frame_array(out, i):
    return np.array(Image.open(os.path.join(out, f"frame_{i:03d}.png")).convert("RGB"))


def self_test(out, poses, bays, a):
    """Inject known shifts into the observed image; the estimate must follow."""
    if a.frame is not None:
        pose = next(p for p in poses if S.pose_idx(p) == a.frame)
    else:
        # The frame showing the most paint, not an arbitrary one: this is a
        # known-answer test of the estimator, and a frame with a single bay in
        # it has too little structure to locate to a tenth of a pixel. That is
        # a property of the frame, not of the method -- picking a weak frame
        # would fail the test for the wrong reason.
        pose = max(poses, key=lambda p: predicted_mask(bays, p)[1])
    img = frame_array(out, S.pose_idx(pose))
    base = measure_frame(pose, bays, img, a.max_shift, a.bg_radius)
    if base is None:
        raise SystemExit("self-test frame shows no bays")
    print(f"self-test on frame {base['frame']} ({base['bays']} bays), "
          f"base offset du={base['du_px']:+.3f} dv={base['dv_px']:+.3f} px "
          f"peak={base['peak']:.3f}")
    print(f"{'injected du,dv':>16}  {'recovered - base':>18}  {'error px':>9}")
    worst = 0.0
    for iu, iv in ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (-2.0, 1.5),
                   (2.5, -0.5), (-0.25, -0.25), (3.0, 3.0)):
        m = measure_frame(pose, bays, img, a.max_shift, a.bg_radius, inject=(iu, iv))
        ru, rv = m["du_px"] - base["du_px"], m["dv_px"] - base["dv_px"]
        err = math.hypot(ru - iu, rv - iv)
        worst = max(worst, err)
        print(f"{iu:+7.2f},{iv:+7.2f}      {ru:+8.3f},{rv:+8.3f}  {err:9.3f}")
    # 0.15 px is the honest floor of this test rather than of the estimator:
    # the injection bilinearly RESAMPLES the observed image (which slightly
    # blurs it) while the estimator shifts the predicted mask, so the two are
    # not exact inverses and a fraction of a pixel of disagreement is expected.
    # Whole-pixel injections, where they are exact inverses, recover to 0.001 px.
    # 0.15 px is 0.009 m at 30 m -- 70x smaller than the 0.63 m error this tool
    # was built to explain, and 4x smaller than the residual it now reports.
    ok = worst <= 0.15
    print(f"worst error {worst:.3f} px -> {'PASS' if ok else 'FAIL'} (tolerance 0.15 px)")
    return 0 if ok else 1


def main():
    a = args
    out, poses = load_run(a.area)
    bays = S.load_bays()
    if a.self_test:
        return self_test(out, poses, bays, a)

    rows = []
    print(f"{'frame':>5} {'bays':>4} {'du_px':>7} {'dv_px':>7} {'along_m':>8} "
          f"{'cross_m':>8} {'|d|_m':>7} {'ncc':>5}")
    for pose in poses:
        i = S.pose_idx(pose)
        if a.frame is not None and i != a.frame:
            continue
        img_arr = frame_array(out, i)
        m = measure_frame(pose, bays, img_arr, a.max_shift, a.bg_radius)
        if m is None:
            continue
        rows.append(m)
        if a.overlay:
            os.makedirs(a.overlay, exist_ok=True)
            write_overlay(os.path.join(a.overlay, f"align_{i:03d}.png"),
                          img_arr, bays, pose, m["du_px"], m["dv_px"], a.zoom)
        flag = " CLIPPED" if m["clipped"] else ""
        print(f"{m['frame']:>5} {m['bays']:>4} {m['du_px']:>+7.2f} {m['dv_px']:>+7.2f} "
              f"{m['along_m']:>+8.3f} {m['cross_m']:>+8.3f} {m['mag_m']:>7.3f} "
              f"{m['peak']:>5.3f}{flag}")
    if rows:
        mags = sorted(r["mag_m"] for r in rows)
        med = mags[len(mags) // 2]
        worst = max(rows, key=lambda r: r["mag_m"])
        print(f"\n{len(rows)} frames: median |offset| {med:.3f} m, "
              f"worst frame {worst['frame']} at {worst['mag_m']:.3f} m "
              f"(along {worst['along_m']:+.3f}, cross {worst['cross_m']:+.3f})")
    if a.json:
        json.dump(rows, open(a.json, "w"), indent=1)
        print(f"wrote {a.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
