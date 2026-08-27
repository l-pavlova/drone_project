"""Do two independent hand-annotation passes agree, and on WHICH axis?

    python vision/diag/gate_polygons.py vision/data/annot_0035/gate20.json \
           --baseline vision/data/annot_0035/bay_outlines.json

The gate before any larger annotation run: redraw a handful of bays and ask whether
the drawings reproduce. The answer turns out to be "yes on one axis, no on the
other", and that split is the whole result -- so this file reports the two axes
separately and refuses to average them into a single distance.

**Measured on flight 0035, 23 polygons against 56 earlier drawings:**

  | axis        | median  | p90    |
  |-------------|---------|--------|
  | ACROSS row  | 0.18 m  | 0.52 m |
  | ALONG row   | 1.95 m  | 3.28 m |

Across the row -- which side of the street, how far from the kerb -- two annotators
agree to 18 cm. Along the row they differ by about half a bay, and the overlays say
why: on this street the parking surface is unmarked cobble, so there is no paint to
say where one bay ends and the next begins. A row of identical bays can slide along
itself and still look right. That is a property of the STREET, not of the annotator,
and no amount of care removes it.

Two consequences, and they point in opposite directions:

  * It is the argument FOR the curb-run representation. The axis humans cannot
    agree on is exactly the axis an interval along an arclength makes irrelevant,
    while the axis they agree on to 18 cm is the one that decides which run a car
    belongs to. The representation was chosen before this was measured; this is
    the confirmation.
  * It bounds what per-bay ground truth can ever mean here. "Is bay 17690
    occupied" is not answerable to better than half a bay when nobody can say
    where 17690 starts. Per-run counts are answerable.

**Do not match by bay id.** Both files resolve a drawing to a published bay by
nearest centroid, and with a ~4.5 m error against a ~5 m pitch that assignment is
close to arbitrary -- measured here, the nearest earlier drawing carries the same
bay id only 11 times in 23, and matching by id reported a spurious 10.97 m maximum
that was really a two-bay mis-assignment. Matching is by ground position.

**The bearing test that used to live here has been removed as invalid.** It asked
whether drawn bearings scatter away from the pose-yaw set, on the theory that an
axis-aligned rect export would cluster on it. But the drone flies ALONG the street
it surveys and the bays are parallel to that street, so a correctly drawn polygon
has a bearing near the pose yaw too. The test cannot separate the two cases and its
83%-vs-98% reading meant nothing.
"""
import argparse
import json
import math
import sys

sys.stdout.reconfigure(encoding="utf-8")


def load_rings(path):
    """bay_outlines-shaped file -> {bay_id: [(x, y), ...]} in ENU metres."""
    doc = json.load(open(path, encoding="utf-8"))
    bays = doc.get("bays") or doc
    return {str(k): [tuple(p) for p in v["ring"]] for k, v in bays.items()}


def centroid(ring):
    return (sum(p[0] for p in ring) / len(ring),
            sum(p[1] for p in ring) / len(ring))


def long_axis(ring):
    """Unit vector along the ring's longest edge."""
    best, bu = -1.0, (1.0, 0.0)
    for i in range(len(ring)):
        a, b = ring[i], ring[(i + 1) % len(ring)]
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d > best:
            best, bu = d, ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
    return bu


def long_short(ring):
    e = sorted(math.hypot(ring[(i + 1) % len(ring)][0] - ring[i][0],
                          ring[(i + 1) % len(ring)][1] - ring[i][1])
               for i in range(len(ring)))
    return e[-1], e[0]


def pct(vals, q):
    v = sorted(vals)
    return v[min(len(v) - 1, int(q / 100.0 * len(v)))] if v else float("nan")


def rms(vals):
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("polygons", help="bay_outlines.json from the POLYGON redraw")
    ap.add_argument("--baseline", default="vision/data/annot_0035/bay_outlines.json")
    ap.add_argument("--pass-across-m", type=float, default=0.75,
                    help="median ACROSS-row agreement at or below this = pass")
    ap.add_argument("--fail-across-m", type=float, default=1.5,
                    help="median ACROSS-row agreement above this = fail")
    ap.add_argument("--pair-max-m", type=float, default=6.0,
                    help="beyond this the two drawings are different bays, not a pair")
    a = ap.parse_args()

    new = load_rings(a.polygons)
    old = load_rings(a.baseline)
    print(f"polygon redraw: {len(new)} drawings   baseline: {len(old)} drawings")

    # ---- pair by POSITION, never by bay id ---------------------------------
    pairs = []
    same_id = 0
    for bid, ring in new.items():
        c = centroid(ring)
        best = min(old.items(),
                   key=lambda t: math.hypot(c[0] - centroid(t[1])[0],
                                            c[1] - centroid(t[1])[1]))
        co = centroid(best[1])
        d = math.hypot(c[0] - co[0], c[1] - co[1])
        if d <= a.pair_max_m:
            pairs.append((bid, ring, best[0], best[1], c, co))
            same_id += (best[0] == bid)
    if not pairs:
        sys.exit("no drawing pairs within "
                 f"{a.pair_max_m} m -- the two passes drew different bays")

    print(f"{len(pairs)} pair(s) within {a.pair_max_m} m; the nearest earlier "
          f"drawing carried the SAME bay id {same_id}/{len(pairs)} times")
    if same_id < len(pairs) * 0.8:
        print("  -> bay-id assignment is UNSTABLE at this error scale, which is why "
              "pairing is by position")

    # ---- decompose on the bay's own axis -----------------------------------
    along, across = [], []
    for _bid, ring, _oid, _oring, c, co in pairs:
        u = long_axis(ring)
        n = (-u[1], u[0])
        dv = (c[0] - co[0], c[1] - co[1])
        along.append(abs(dv[0] * u[0] + dv[1] * u[1]))
        across.append(abs(dv[0] * n[0] + dv[1] * n[1]))

    print("\nagreement between the two passes, on the bay's own axes:")
    for tag, v in (("ACROSS row", across), ("ALONG row", along)):
        print(f"  {tag:11s} median {pct(v, 50):.2f} m   rms {rms(v):.2f}   "
              f"p90 {pct(v, 90):.2f}   max {max(v):.2f}")

    # ---- size ---------------------------------------------------------------
    print()
    for tag, rings in (("baseline", [p[3] for p in pairs]),
                       ("redraw  ", [p[1] for p in pairs])):
        ls = [long_short(r) for r in rings]
        print(f"  {tag} size: median {pct([x[0] for x in ls], 50):.2f} x "
              f"{pct([x[1] for x in ls], 50):.2f} m")
    print("   (Sofiaplan's published rectangles are 5.40 x 2.20 m)")

    # ---- verdict ------------------------------------------------------------
    ac = pct(across, 50)
    al = pct(along, 50)
    print()
    if ac > a.fail_across_m:
        sys.exit(f"FAIL -- the two passes disagree by {ac:.2f} m ACROSS the row "
                 f"(> {a.fail_across_m}).\nThat is the axis that decides which row "
                 "a car belongs to, so hand annotation is not\nreproducible here "
                 "and nothing downstream can rest on it.")
    if ac <= a.pass_across_m:
        print(f"PASS -- the two passes agree to {ac:.2f} m ACROSS the row "
              f"(<= {a.pass_across_m}).")
        print(f"Hand annotation IS reproducible on the axis that matters.")
        if al > 1.0:
            print(f"\nALONG the row they differ by {al:.2f} m -- about half a bay. "
                  "Expect this, and do\nnot try to annotate it away: with no bay "
                  "paint on the surface there is nothing in\nthe image that fixes "
                  "the phase of the row. It is the measured argument for\ncounting "
                  "cars along a curb run rather than per addressable bay.")
        return
    print(f"INCONCLUSIVE -- {ac:.2f} m across the row, between the pass bound "
          f"({a.pass_across_m}) and the\nfail bound ({a.fail_across_m}). Draw more "
          "bays before deciding.")


if __name__ == "__main__":
    main()
