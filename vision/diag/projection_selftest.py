"""Prove project() and unproject() are actually inverses, on real poses.

    python vision/diag/projection_selftest.py [area ...]      # default: both fixtures

`unproject()` is what turns a detector's box into a place on the ground, so
every future occupancy verdict from the detector path rests on it. This checks
it the only way worth checking: take every bay corner of a flown survey, send it
through `project()` to a pixel and straight back through `unproject()`, and
require the metre it started from.

Why a round trip and not a hand-worked example: the two functions share
`camera_axes()`, so an error in the axes cancels and would NOT show up here.
That is deliberate and the limitation is the point - `paint_align.py` is what
tests the axes against the world (it registers the projection against the paint
actually visible in a frame). This tests the algebra between them, which is the
part `paint_align.py` cannot separate out. The two together cover it; either
alone does not.

Exits 1 on failure, so it can be run from a checker or a hook.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import json

import score_occupancy as so

TOL_M = 1e-3            # 1 mm: far below the 0.043 m the camera model achieves
FIXTURES = ("fmi_block", "fmi_block_4st")


def bays_of(area):
    """Bay rings in ENU for one area, via the scorer's own loader.

    score_occupancy resolves paths from module-level constants set at import
    from argv, so point them at this area before loading rather than
    re-implementing the geojson read.
    """
    root = so.ROOT
    so.SURVEY_AREA = area
    so.OUT = os.path.join(root, "sim", "output", area)
    so.GT_FILE = os.path.join(root, "sim", "worlds",
                              "ground_truth.json" if area == "fmi_block"
                              else f"{area}.ground_truth.json")
    return so.load_bays()


def run(area):
    out = os.path.join(so.ROOT, "sim", "output", area)
    poses = json.load(open(os.path.join(out, "poses.json"), encoding="utf-8"))
    bays = bays_of(area)

    worst = 0.0
    worst_where = None
    tested = skipped = 0
    for pose in poses:
        for b in bays:
            for px, py in b["ring"]:
                u, v = so.project(px, py, pose)
                back = so.unproject(u, v, pose)
                if back is None:
                    # A ray that never reaches the ground. For a nadir-ish
                    # survey pose this should not happen at all; count it rather
                    # than passing over it silently.
                    skipped += 1
                    continue
                err = math.hypot(back[0] - px, back[1] - py)
                tested += 1
                if err > worst:
                    worst, worst_where = err, (pose.get("frame_idx"), b["id"])
    return worst, worst_where, tested, skipped


def main():
    areas = sys.argv[1:] or list(FIXTURES)
    failed = False
    for area in areas:
        try:
            worst, where, tested, skipped = run(area)
        except OSError as exc:
            print(f"SKIP  {area}: {exc}")
            continue
        status = "ok " if worst <= TOL_M and not skipped else "FAIL"
        print(f"  {status} {area}: {tested} corner round-trips, worst "
              f"{worst * 1000:.6f} mm" + (f" (frame {where[0]}, bay {where[1]})"
                                          if where else "")
              + (f", {skipped} ray(s) never met the ground" if skipped else ""))
        if worst > TOL_M or skipped:
            failed = True
    if failed:
        print("\nFAILED: unproject() is not the inverse of project()")
        sys.exit(1)
    print("\nproject/unproject round-trip holds")


if __name__ == "__main__":
    main()
