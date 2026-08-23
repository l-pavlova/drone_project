"""Recover the camera HEADING for a real DJI flight, which the telemetry omits.

    python tools/dji_yaw.py <stills_dir> [--write] [--scale 4] [--min-move 3.0]
    python tools/dji_yaw.py --self-test

`tools/dji_srt.py` reads `gb_yaw`/`gb_pitch`/`gb_roll` when the SRT carries them.
The 2026-08-21 flights do not: every block holds latitude, longitude and
rel_alt and nothing else. So `poses.json` has a position and no orientation, and
without orientation `project()`/`unproject()` cannot say where in the frame a
bay is -- the whole real-occupancy path is blocked, and `frame.yaw` is NOT NULL
in the web schema, so a real frame cannot even be ingested.

**The estimator.** Between two consecutive stills the GPS gives a world
displacement and image registration gives the ground content's pixel shift.
Those are the SAME vector written in two frames, so the angle between them is
the camera heading. Writing it out for a nadir camera (see `camera_axes`:
image-right r = -left, image-down b = -forward):

    (du, dv) = (f * L / alt) * (-sin(yaw - course), cos(yaw - course))

    =>  yaw = course + atan2(-du, dv)
        and  hypot(du, dv) = f * L / alt,  i.e. L / hypot = the GSD

Two things fall out of that for free and both are kept:

  * **the magnitude validates the optics.** `L / hypot(du, dv)` must equal
    `alt / f` = 11.7 mm/px. If it does not, either `rel_alt` or the 73.7 deg FOV
    is wrong, and finding that out here is far better than meeting it later as a
    mysterious bay offset.
  * **course over ground is an independent second estimate**, computable with no
    imagery at all. A DJI in forward flight with the gimbal following the nose
    should show yaw ~ course; where the two disagree the flow estimate is the
    right one (the aircraft can crab, and the operator can yaw the gimbal), and
    the disagreement is itself worth reporting rather than hiding.

**Why the self-test is not optional.** Every real bay verdict downstream rests
on this number, and a wrong yaw does not look wrong -- it produces a confident,
plausible, wrong map. So `--self-test` closes the loop the only way that proves
anything: choose a yaw, ask `project()` itself where a ground point lands in two
poses, synthesise the second image by shifting the first by exactly that, and
require the estimator to give the yaw back. That tests the algebra against the
projection code (not against a hand-derived sign) AND the registration against a
known shift, which are the two independent ways this can be wrong.

Robustness rules, each one a way this goes quietly wrong:

  * **skip pairs that barely moved.** Direction is ill-conditioned over a short
    baseline -- consumer GPS noise is 1-3 m, so under a few metres of travel the
    course is mostly noise and the yaw inherits it.
  * **screen non-nadir frames first** (`vision/check_nadir.py`). The formula
    assumes a nadir camera; an oblique frame yields a confident wrong answer.
    Flight 0035 tilts up over its last ~7 frames on the return.
  * **take a robust central estimate, not a per-pair one.** A single bad
    registration must not leave one frame rotated on its own, so each frame gets
    the circular median of a window of pair estimates.
"""
import argparse
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "vision"))

import cameras                                                  # noqa: E402
import numpy as np                                              # noqa: E402
import score_occupancy as so                                    # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

MIN_MOVE_M = 3.0        # below this the GPS direction is mostly noise
SCALE = 4               # downscale for registration: 3840 -> 960
SMOOTH = 5              # frames in the circular-median window
GSD_TOL = 0.25          # reject a pair whose implied GSD is off by more than this
PATCH = 0.40            # fraction of the frame used as the match template
MIN_SCORE = 0.30        # below this the correlation peak is not a match


# --------------------------------------------------------------- the estimator

def yaw_from_shift(dx, dy, du, dv):
    """(world move, image move of the ground) -> (yaw rad, implied metres/px).

    Both inputs are for the SAME pair, in the same order: `dx, dy` is where the
    aircraft went, `du, dv` is where the picture's content went. Returns None
    for a degenerate pair (no motion in one of the two frames).
    """
    length = math.hypot(dx, dy)
    pixels = math.hypot(du, dv)
    if length <= 0 or pixels <= 0:
        return None
    return ((math.atan2(dy, dx) + math.atan2(-du, dv)) % (2 * math.pi),
            length / pixels)


def wrap(a):
    """Angle to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def circ_median(angles):
    """Circular median: the angle minimising total absolute wrapped deviation.

    A plain median is wrong on a circle (0 deg and 359 deg average to 180), and
    the circular MEAN would let one bad registration drag the answer. This is
    O(n^2) over a handful of candidates, which at these sizes is free.
    """
    if not angles:
        return None
    best, best_cost = None, None
    for c in angles:
        cost = sum(abs(wrap(a - c)) for a in angles)
        if best_cost is None or cost < best_cost:
            best, best_cost = c, cost
    return best


# --------------------------------------------------------------- registration

def _prep(img, scale):
    """Grayscale float32, downscaled."""
    import cv2
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    if scale != 1:
        g = cv2.resize(g, (g.shape[1] // scale, g.shape[0] // scale),
                       interpolation=cv2.INTER_AREA)
    return np.float32(g)


def image_shift(img_a, img_b, scale=SCALE, patch=PATCH):
    """Pixel shift of the ground content from frame A to frame B, full-res px.

    Take the central `patch` of frame A and find where it went in frame B, by
    normalised cross-correlation. Returns (du, dv, score), score in [-1, 1].

    **Phase correlation was tried first and does not work on this footage.**
    It is the textbook answer for a pure translation and it failed outright:
    peak responses of 0.002-0.02 and shifts that disagreed with the GPS by a
    factor of 6.5, because a 30 m nadir frame over Lozenec is mostly summer
    canopy -- broadband, self-similar, and moving with a parallax of its own
    against the ground. There is no single dominant global translation for the
    whole frame to lock on to. Normalised cross-correlation of a central patch
    scores 0.66-0.84 on the same pairs and agrees with the GPS to within ~10%.

    Two consequences worth knowing. The search is bounded by the frame minus
    the patch, so at the default 40% patch and scale 4 it reaches +/-1300 px
    full-res -- comfortably past the ~460 px a 1 Hz step produces here, but not
    unlimited. And the match is translation-only, so a pair spanning a TURN
    scores low and is dropped rather than answered wrongly; that is the correct
    outcome and it is why the score is returned rather than swallowed.
    """
    import cv2
    a, b = _prep(img_a, scale), _prep(img_b, scale)
    h, w = a.shape
    pw, ph = int(w * patch), int(h * patch)
    x0, y0 = (w - pw) // 2, (h - ph) // 2
    res = cv2.matchTemplate(b, a[y0:y0 + ph, x0:x0 + pw], cv2.TM_CCOEFF_NORMED)
    _, score, _, loc = cv2.minMaxLoc(res)
    return (loc[0] - x0) * scale, (loc[1] - y0) * scale, score


# ------------------------------------------------------------------ self-test

def self_test():
    """Prove the algebra against project(), and the registration against a
    known shift. Both must hold or nothing downstream is believable."""
    import cv2
    ok = True
    cam = cameras.DJI_NADIR

    # (1) ALGEBRA. For a spread of headings and courses, ask project() itself
    # where a ground point lands in two poses, then require the estimator to
    # recover the heading from that displacement alone.
    worst, n = 0.0, 0
    for yaw_deg in range(0, 360, 17):
        for course_deg in range(0, 360, 23):
            yaw = math.radians(yaw_deg)
            course = math.radians(course_deg)
            length = 4.2                              # a 1 Hz step at ~4 m/s
            dx, dy = length * math.cos(course), length * math.sin(course)
            p0 = {"x": 0.0, "y": 0.0, "alt": 30.1, "yaw": yaw}
            p1 = {"x": dx, "y": dy, "alt": 30.1, "yaw": yaw}
            gx, gy = 3.0, -2.0                        # any fixed ground point
            u0, v0 = so.project(gx, gy, p0, cam=cam)
            u1, v1 = so.project(gx, gy, p1, cam=cam)
            got = yaw_from_shift(dx, dy, u1 - u0, v1 - v0)
            assert got, "degenerate pair in the self-test itself"
            err = abs(math.degrees(wrap(got[0] - yaw)))
            gsd_err = abs(got[1] - cam.gsd(30.1))
            worst = max(worst, err)
            n += 1
            if err > 1e-6 or gsd_err > 1e-9:
                ok = False
                print(f"  FAIL algebra: yaw {yaw_deg} course {course_deg}: "
                      f"off by {err:.6f} deg, gsd off {gsd_err:.3e} m/px")
    print(f"  {'ok ' if ok else 'FAIL'} algebra vs project(): {n} "
          f"(yaw, course) pairs, worst {worst:.2e} deg")

    # (2) REGISTRATION. Shift a real frame by a known amount and measure it
    # back. np.roll wraps around, which phase correlation is untroubled by (it
    # is a circular correlation to begin with) and which keeps the test free of
    # any padding choice that could flatter it.
    hits = sorted(glob.glob(os.path.join(
        so.ROOT, "pics", "dji", "stills", "*", "frame_0000.jpg")))
    if not hits:
        print("  SKIP registration: no real stills on disk")
        return ok
    src = cv2.imread(hits[0])
    worst_px = 0.0
    for shift in ((120, 0), (0, -180), (-260, 340), (75, 75)):
        moved = np.roll(np.roll(src, shift[0], axis=1), shift[1], axis=0)
        du, dv, resp = image_shift(src, moved)
        err = math.hypot(du - shift[0], dv - shift[1])
        worst_px = max(worst_px, err)
        if err > 2.0 * SCALE:
            ok = False
            print(f"  FAIL registration: wanted {shift}, got "
                  f"({du:.1f}, {dv:.1f}), response {resp:.3f}")
    print(f"  {'ok ' if ok else 'FAIL'} registration on a real frame: 4 known "
          f"shifts, worst {worst_px:.2f} px (scale {SCALE}, tol {2 * SCALE} px)")

    # (3) END TO END. A known yaw produces a predicted image shift; synthesise
    # the second frame with it and run the whole chain. This is the one that
    # would catch a sign error that (1) and (2) each cancel on their own.
    worst_deg = 0.0
    for yaw_deg in (0, 47, 132, 205, 291):
        yaw, course = math.radians(yaw_deg), math.radians(25.0)
        length = 4.2
        dx, dy = length * math.cos(course), length * math.sin(course)
        theta = yaw - course
        scale = cam.f * length / 30.1
        du, dv = -scale * math.sin(theta), scale * math.cos(theta)
        moved = np.roll(np.roll(src, int(round(du)), axis=1),
                        int(round(dv)), axis=0)
        mu, mv, _ = image_shift(src, moved)
        got = yaw_from_shift(dx, dy, mu, mv)
        err = abs(math.degrees(wrap(got[0] - yaw)))
        worst_deg = max(worst_deg, err)
        if err > 1.0:
            ok = False
            print(f"  FAIL end-to-end: yaw {yaw_deg} recovered as "
                  f"{math.degrees(got[0]):.2f} ({err:.2f} deg off)")
    print(f"  {'ok ' if ok else 'FAIL'} end to end (yaw -> image -> yaw): "
          f"5 headings, worst {worst_deg:.3f} deg")
    return ok


# ----------------------------------------------------------------------- main

def screen_oblique(stills, poses):
    """Frames whose top strip reads as sky, via check_nadir's own thresholds."""
    import cv2
    sys.path.insert(0, os.path.join(so.ROOT, "vision"))
    import check_nadir
    out = set()
    for pose in poses:
        img = cv2.imread(os.path.join(stills, pose["file"]))
        if img is None:
            continue
        top = img[:int(img.shape[0] * check_nadir.TOP_FRAC)]
        if (float(top.mean()) > check_nadir.T_BRIGHT
                and float(top[:, :, 0].mean() - top[:, :, 2].mean())
                > check_nadir.T_BLUE):
            out.add(pose["file"])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills", nargs="?", help="dir of frame_*.jpg + poses.json")
    ap.add_argument("--write", action="store_true",
                    help="write yaw back into poses.json (default: report only)")
    ap.add_argument("--scale", type=int, default=SCALE)
    ap.add_argument("--min-move", type=float, default=MIN_MOVE_M)
    ap.add_argument("--smooth", type=int, default=SMOOTH)
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()

    if a.self_test:
        sys.exit(0 if self_test() else 1)
    if not a.stills:
        ap.error("give a stills dir, or --self-test")

    import cv2
    path = os.path.join(a.stills, "poses.json")
    poses = json.load(open(path, encoding="utf-8"))
    cam = cameras.DJI_NADIR
    print(f"{len(poses)} poses in {path}")

    oblique = screen_oblique(a.stills, poses)
    if oblique:
        print(f"  {len(oblique)} frame(s) screened out as non-nadir: "
              f"{sorted(oblique)[0]} .. {sorted(oblique)[-1]}")

    # Per-pair estimates, keyed by the index of the FIRST frame of the pair.
    pair = {}
    n_short = n_skip = n_weak = 0
    prev_img = prev_i = None
    for i, pose in enumerate(poses):
        if pose["file"] in oblique or "x" not in pose:
            prev_img, prev_i = None, None
            n_skip += 1
            continue
        img = cv2.imread(os.path.join(a.stills, pose["file"]))
        if img is None:
            prev_img, prev_i = None, None
            n_skip += 1
            continue
        if prev_img is not None:
            p0 = poses[prev_i]
            dx, dy = pose["x"] - p0["x"], pose["y"] - p0["y"]
            if math.hypot(dx, dy) < a.min_move:
                n_short += 1
            else:
                du, dv, resp = image_shift(prev_img, img, a.scale)
                got = yaw_from_shift(dx, dy, du, dv) if resp >= MIN_SCORE else None
                if resp < MIN_SCORE:
                    n_weak += 1
                if got:
                    pair[prev_i] = {"yaw": got[0], "gsd": got[1], "resp": resp,
                                    "course": math.atan2(dy, dx),
                                    "move": math.hypot(dx, dy),
                                    "alt": p0.get("alt")}
        prev_img, prev_i = img, i

    if not pair:
        print("no usable pairs -- nothing written")
        sys.exit(1)

    # Does the geometry agree with the optics? The free cross-check.
    exp_gsd = cam.gsd(sum(v["alt"] for v in pair.values()) / len(pair))
    gsds = sorted(v["gsd"] for v in pair.values())
    med_gsd = gsds[len(gsds) // 2]
    ratio = med_gsd / exp_gsd
    print(f"  {len(pair)} usable pairs ({n_short} too short, {n_weak} weak "
          f"match, {n_skip} skipped)")
    print(f"  implied GSD {med_gsd * 1000:.2f} mm/px vs {exp_gsd * 1000:.2f} "
          f"expected from {cam!r} -- ratio {ratio:.3f}"
          + ("   <-- CHECK alt/FOV" if abs(ratio - 1) > GSD_TOL else "   ok"))

    # Drop pairs whose implied GSD is wildly off: a registration that locked on
    # to something other than the ground gets the scale wrong as well as the
    # angle, so the scale is a free validity test on the angle.
    good = {i: v for i, v in pair.items()
            if abs(v["gsd"] / exp_gsd - 1) <= GSD_TOL}
    print(f"  {len(good)} pairs pass the GSD check")
    if not good:
        print("no pairs survived the GSD check -- nothing written")
        sys.exit(1)

    idxs = sorted(good)
    all_courses = [good[j]["course"] for j in idxs]
    n_flow = 0
    for i, pose in enumerate(poses):
        near = [good[j]["yaw"] for j in idxs if abs(j - i) <= a.smooth]
        if near:
            pose["yaw"] = round(circ_median(near), 6)
            pose["yaw_src"] = "flow"
            n_flow += 1
        else:
            # No imagery-backed estimate within reach (an oblique tail, or a
            # hover). Course over ground is worse but it is a heading, and the
            # source is recorded so a consumer can tell the two apart.
            pose["yaw"] = round(circ_median(all_courses), 6)
            pose["yaw_src"] = "course"
        if i in good:
            pose["yaw_resid_deg"] = round(abs(math.degrees(
                wrap(good[i]["yaw"] - good[i]["course"]))), 2)

    # Flow vs course agreement: on a gimbal that follows the nose these track,
    # and where they do not the flow number is the real one.
    diffs = sorted(abs(math.degrees(wrap(v["yaw"] - v["course"])))
                   for v in good.values())
    print(f"  flow vs GPS course: median {diffs[len(diffs) // 2]:.1f} deg, "
          f"90th pct {diffs[int(len(diffs) * 0.9)]:.1f} deg, max {diffs[-1]:.1f} deg")
    print(f"  yaw set on {len(poses)} poses "
          f"({n_flow} from flow, {len(poses) - n_flow} from course)")

    if a.write:
        with open(path, "w", encoding="utf-8", newline="") as fh:
            json.dump(poses, fh, indent=1)
        print(f"  wrote {path}")
    else:
        print("  (dry run -- pass --write to update poses.json)")


if __name__ == "__main__":
    main()
