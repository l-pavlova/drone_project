"""Apply a hand-made bay geometry correction at LOAD time.

    from bay_corrections import load, apply_to
    bays = apply_to(so.load_all_bays(), load("vision/data/annot_0035/bay_outlines.json"))

A sidecar, never an edit to `data/block_bays.geojson`. Sofiaplan's geometry stays
exactly as published and every consumer opts in, so a correction can be compared
against no correction, revised, or thrown away without re-deriving anything. Same
posture as a street closure overriding the published answer at read time instead of
being written into `bay_state`.

Two producers, two entry shapes -- see `apply_to`:

  * `import_annotations.py` (primary) writes a replacement **outline**, from bays
    drawn where they really are and unprojected to the ground. Size and orientation
    come from the drawing, so this fixes rows that are the wrong SHAPE.
  * `align_bays.py` (fallback) writes a rigid **move** of the published rectangle,
    which keeps Sofiaplan's size and can only fix where it sits.

Bays with no entry are returned untouched, which is what makes a file safe to apply
to the full 1,698-bay set even though a flight only ever verifies a few dozen.
"""
import json
import math
import os


def load(path):
    """-> {bay_id: {dx, dy, rot_deg, cx, cy}}; {} for None/missing."""
    if not path:
        return {}
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    doc = json.load(open(path, encoding="utf-8"))
    return {str(k): v for k, v in (doc.get("bays") or doc).items()}


def _move(x, y, c):
    ex, ey = x - c["cx"], y - c["cy"]
    th = math.radians(c.get("rot_deg", 0.0))
    ct, st = math.cos(th), math.sin(th)
    return (ct * ex - st * ey + c["cx"] + c.get("dx", 0.0),
            st * ex + ct * ey + c["cy"] + c.get("dy", 0.0))


def apply_to(bays, corr):
    """score_occupancy.load_all_bays() shape in, same shape out.

    Two kinds of entry, because the two ways of correcting a bay produce different
    information and neither should be forced into the other's shape:

      * ``{dx, dy, rot_deg, cx, cy}`` -- a rigid move of the published rectangle,
        from `align_bays.py`. The bay keeps Sofiaplan's size and proportions.
      * ``{ring: [[x, y], ...]}`` -- an outright replacement outline in ENU metres,
        from `import_annotations.py`, i.e. someone drew the bay where it really is.
        Size and orientation come from the drawing, so a row of bays that is the
        wrong SHAPE (not merely in the wrong place) can be fixed this way.
    """
    if not corr:
        return bays
    out = []
    for b in bays:
        c = corr.get(str(b["id"]))
        if not c:
            out.append(b)
            continue
        if c.get("ring"):
            ring = [(float(x), float(y)) for x, y in c["ring"]]
            nb = dict(b)
            nb["ring"] = ring
            nb["cx"] = sum(p[0] for p in ring) / len(ring)
            nb["cy"] = sum(p[1] for p in ring) / len(ring)
            out.append(nb)
            continue
        ring = [_move(x, y, c) for x, y in b["ring"]]
        nb = dict(b)
        nb["ring"] = ring
        nb["cx"] = sum(p[0] for p in ring) / len(ring)
        nb["cy"] = sum(p[1] for p in ring) / len(ring)
        out.append(nb)
    return out


def summary(corr):
    # Keyed by (street, group), not by street: a row can carry several groups with
    # DIFFERENT offsets -- that is the whole point of the grouping -- and collapsing
    # them to one line per street would report one group's shift as the row's.
    rings = sum(1 for c in corr.values() if c.get("ring"))
    by = {}
    for c in corr.values():
        if c.get("ring"):
            continue
        k = (c.get("street", "?"), c.get("group", c.get("segment", 0)))
        by.setdefault(k, [c, 0])[1] += 1
    lines = []
    if rings:
        lines.append(f"  {rings:>4} bays  outline REPLACED from a drawn annotation")
    for (st, g), (c, n) in sorted(by.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        d = math.hypot(c.get("dx", 0.0), c.get("dy", 0.0))
        tag = f"{st} [{g + 1}]" if any(k[0] == st and k[1] != g for k in by) else st
        lines.append(f"  {n:>4} bays  shift {d:5.2f} m "
                     f"({c.get('dx', 0):+.2f}, {c.get('dy', 0):+.2f})  "
                     f"rot {c.get('rot_deg', 0):+.1f} deg   {tag}")
    return "\n".join(lines)


def _self_test():
    """Prove the rigid move: a known shift/rotation must come back out exactly.

    Run as `python vision/bay_corrections.py`. The point is the same one
    `paint_ground_control.py --self-test` makes -- a geometry correction that is
    wrong does not look wrong, it produces a confident, plausible, wrong map.
    """
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import score_occupancy as so

    bays = so.load_all_bays()[:200]
    ids = [str(b["id"]) for b in bays]
    cx = sum(b["cx"] for b in bays) / len(bays)
    cy = sum(b["cy"] for b in bays) / len(bays)
    worst_t = worst_r = worst_shape = 0.0

    for dx, dy, rot in [(3.0, 0.0, 0.0), (0.0, -2.5, 0.0), (-1.25, 4.0, 0.0),
                        (0.0, 0.0, 7.5), (2.0, -3.0, -12.0)]:
        corr = {i: {"dx": dx, "dy": dy, "rot_deg": rot, "cx": cx, "cy": cy}
                for i in ids}
        moved = apply_to(bays, corr)
        for a, b in zip(bays, moved):
            # a pure translation must move every centroid by exactly (dx, dy)
            if rot == 0.0:
                worst_t = max(worst_t, abs(b["cx"] - a["cx"] - dx),
                              abs(b["cy"] - a["cy"] - dy))
            # a rigid move preserves every edge length, rotation or not
            for i in range(len(a["ring"]) - 1):
                la = math.hypot(a["ring"][i + 1][0] - a["ring"][i][0],
                                a["ring"][i + 1][1] - a["ring"][i][1])
                lb = math.hypot(b["ring"][i + 1][0] - b["ring"][i][0],
                                b["ring"][i + 1][1] - b["ring"][i][1])
                worst_shape = max(worst_shape, abs(la - lb))
            # and distance from the rotation centre is preserved
            ra = math.hypot(a["cx"] - cx, a["cy"] - cy)
            rb = math.hypot(b["cx"] - dx - cx, b["cy"] - dy - cy)
            worst_r = max(worst_r, abs(ra - rb))

        # inverting the correction must return the original geometry
        inv = {i: {"dx": 0.0, "dy": 0.0, "rot_deg": -rot, "cx": cx + dx, "cy": cy + dy}
               for i in ids}
        back = apply_to(apply_to(bays, corr), inv)
        undo = {i: {"dx": -dx, "dy": -dy, "rot_deg": 0.0, "cx": 0.0, "cy": 0.0}
                for i in ids}
        back = apply_to(back, undo)
        worst_rt = max(max(abs(p[0] - q[0]), abs(p[1] - q[1]))
                       for a, b in zip(bays, back)
                       for p, q in zip(a["ring"], b["ring"]))
        print(f"  dx={dx:+5.2f} dy={dy:+5.2f} rot={rot:+6.1f}  "
              f"round-trip worst {worst_rt:.2e} m")
        assert worst_rt < 1e-9, "round-trip failed"

    print(f"  translation exact to        {worst_t:.2e} m")
    print(f"  edge lengths preserved to   {worst_shape:.2e} m")
    print(f"  radius from centre kept to  {worst_r:.2e} m")
    assert worst_t < 1e-9 and worst_shape < 1e-9 and worst_r < 1e-9
    print("bay_corrections self-test PASS")


if __name__ == "__main__":
    _self_test()
