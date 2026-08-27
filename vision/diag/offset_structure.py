"""Is Sofiaplan's bay error SYSTEMATIC (one transform fixes it) or per-street?

    python vision/diag/offset_structure.py vision/data/annot_0035/bay_outlines.json

Hand-drawn bays (`import_annotations.py`) give, for the first time, ground truth for
where specific mapped bays really are. Each one yields a correction vector. What the
vectors look like decides how much work the fix is:

  * **one tight cluster** -> a constant offset, most likely a datum/projection slip.
    One translation corrects all 31,705 spaces and nobody draws anything again.
  * **tight per `georef_201`** -> the dataset records the YEAR each feature was
    georeferenced (2009, 2017, ...), i.e. it is a digitisation over an orthophoto of
    that year. A per-year offset would mean one transform per batch.
  * **tight per street** -> per-street correction, the option already tabulated in
    `docs/bay_data_gap.md`.
  * **scattered** -> the geometry is individually wrong and only redrawing fixes it.

Fits a translation, then a similarity (translation + rotation + uniform scale), and
reports the residual each leaves. A similarity that beats translation by a lot is
the signature of a projection problem rather than sloppy digitising.
"""
import argparse
import json
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import score_occupancy as so                                     # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "data")


def centroid(ring):
    return (sum(p[0] for p in ring) / len(ring),
            sum(p[1] for p in ring) / len(ring))


def fit_translation(src, dst):
    dx = statistics.fmean(d[0] - s[0] for s, d in zip(src, dst))
    dy = statistics.fmean(d[1] - s[1] for s, d in zip(src, dst))
    res = [math.hypot(s[0] + dx - d[0], s[1] + dy - d[1]) for s, d in zip(src, dst)]
    return (dx, dy), res


def fit_similarity(src, dst):
    """Umeyama, 2-D: the least-squares rotation + uniform scale + translation."""
    mx = (statistics.fmean(p[0] for p in src), statistics.fmean(p[1] for p in src))
    my = (statistics.fmean(p[0] for p in dst), statistics.fmean(p[1] for p in dst))
    sxx = syy = sxy = syx = var = 0.0
    for s, d in zip(src, dst):
        ax, ay = s[0] - mx[0], s[1] - mx[1]
        bx, by = d[0] - my[0], d[1] - my[1]
        sxx += ax * bx
        syy += ay * by
        sxy += ax * by
        syx += ay * bx
        var += ax * ax + ay * ay
    theta = math.atan2(sxy - syx, sxx + syy)
    ct, st = math.cos(theta), math.sin(theta)
    scale = ((sxx + syy) * ct + (sxy - syx) * st) / var if var else 1.0
    res = []
    for s, d in zip(src, dst):
        ax, ay = s[0] - mx[0], s[1] - mx[1]
        px = scale * (ct * ax - st * ay) + my[0]
        py = scale * (st * ax + ct * ay) + my[1]
        res.append(math.hypot(px - d[0], py - d[1]))
    return (math.degrees(theta), scale), res


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(round(p / 100.0 * (len(v) - 1))))]


def report(tag, src, dst, indent="  "):
    if len(src) < 3:
        print(f"{indent}{tag}: only {len(src)} bay(s) -- not enough to fit")
        return
    raw = [math.hypot(s[0] - d[0], s[1] - d[1]) for s, d in zip(src, dst)]
    (dx, dy), rt = fit_translation(src, dst)
    (th, sc), rs = fit_similarity(src, dst)
    print(f"{indent}{tag}  n={len(src)}")
    print(f"{indent}  uncorrected error      median {q(raw, 50):5.2f} m   p90 {q(raw, 90):5.2f}")
    print(f"{indent}  best TRANSLATION       ({dx:+.2f}, {dy:+.2f}) m  ->  "
          f"residual median {q(rt, 50):5.2f} m   p90 {q(rt, 90):5.2f}")
    print(f"{indent}  best SIMILARITY        rot {th:+.2f} deg, scale {sc:.5f}  ->  "
          f"residual median {q(rs, 50):5.2f} m   p90 {q(rs, 90):5.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outlines", help="bay_outlines.json from import_annotations.py")
    a = ap.parse_args()

    drawn = json.load(open(a.outlines, encoding="utf-8"))["bays"]
    props = {str(f["properties"]["id"]): f["properties"]
             for f in json.load(open(os.path.join(DATA, "block_bays.geojson"),
                                     encoding="utf-8"))["features"]}
    orig = {str(b["id"]): (b["cx"], b["cy"]) for b in so.load_all_bays()}

    src, dst, meta = [], [], []
    for bid, v in drawn.items():
        if bid not in orig:
            continue
        src.append(orig[bid])
        dst.append(centroid(v["ring"]))
        meta.append(props.get(bid, {}))

    print(f"{len(src)} bays with hand-drawn ground truth\n")

    vec = [(d[0] - s[0], d[1] - s[1]) for s, d in zip(src, dst)]
    mag = [math.hypot(*v) for v in vec]
    ang = [math.degrees(math.atan2(v[1], v[0])) % 360 for v in vec]
    print("=== the correction vectors themselves ===")
    print(f"  magnitude  median {q(mag, 50):.2f} m   p10 {q(mag, 10):.2f}   p90 {q(mag, 90):.2f}")
    print(f"  direction  spread: p10 {q(ang, 10):.0f} deg, median {q(ang, 50):.0f}, "
          f"p90 {q(ang, 90):.0f}")
    # circular spread: |mean unit vector| is 1.0 for one direction, 0 for uniform
    cx = statistics.fmean(math.cos(math.radians(t)) for t in ang)
    cy = statistics.fmean(math.sin(math.radians(t)) for t in ang)
    R = math.hypot(cx, cy)
    print(f"  directional concentration R = {R:.3f}   "
          + ("(one direction -- looks systematic)" if R > 0.8 else
             "(spread out -- NOT one global shift)" if R < 0.5 else "(partly aligned)"))

    print("\n=== ALL bays together ===")
    report("global", src, dst)

    for key, label in (("georef_201", "georeferencing year"),
                       ("mestopoloz", "street"),
                       ("park_txt", "parking type")):
        groups = {}
        for s, d, m in zip(src, dst, meta):
            groups.setdefault(m.get(key, "?"), ([], []))
            groups[m.get(key, "?")][0].append(s)
            groups[m.get(key, "?")][1].append(d)
        print(f"\n=== grouped by {label} (`{key}`) ===")
        for k, (gs, gd) in sorted(groups.items(), key=lambda kv: -len(kv[1][0])):
            report(str(k), gs, gd)


if __name__ == "__main__":
    main()
