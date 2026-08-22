"""Read a DJI SRT telemetry sidecar into poses the projection code can use.

    python tools/dji_srt.py <flight.SRT> [--every N] [--json out.json]

DJI writes one subtitle block per video frame, carrying the shooting parameters
and — the part that matters here — latitude, longitude and altitude, and on most
recent models the gimbal angles too. That makes an SRT the real-world equivalent
of the sim's `poses.json`: with it, the SAME `project()` that puts bays onto a
Webots frame puts them onto real footage, so labelling a real flight collapses
from "draw a box around every car" to "is there a car in this bay, y/n".

**The format is not one format.** It has changed across firmware and models:
older files use `[latitude: ...]`, newer ones `[latitude : ...]`, some spell it
`GPS(lon,lat,alt)`, and the HTML-font-tag wrapper comes and goes. So this parses
by hunting known keys with regexes rather than by assuming a layout, reports
what it found, and says plainly what is missing instead of inventing it.

Two things it deliberately does NOT do:

  * **It does not trust the GPS.** Consumer GPS lands within 1-3 m; a parking
    bay is 2.2 m wide, so a pose straight from the SRT can put a bay a whole bay
    off. The pose here is the STARTING POINT for registration (template-matching
    the frame against satellite imagery, as in the supervisor's
    ground-vehicles-localization pipeline), not the final answer.
  * **It does not assume the camera is nadir.** That assumption cost this
    project 2.4 points of accuracy in the sim, where the gimbal was ALSO not
    doing what it was told (see CLAUDE.md, "Camera model"). Gimbal angles are
    carried through when present and left absent when not, so the caller
    chooses the fallback rather than inheriting a silent one.
"""
import json
import math
import os
import re
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# The local ENU frame, duplicated from sim/generate_world.py and
# vision/score_occupancy.py — the same standing duplication ORIGIN always was,
# and tools/check_consistency.py is what keeps the three in step.
ORIGIN = (42.6747105, 23.3298956)
MLAT = 111320.0
MLON = 111320.0 * math.cos(math.radians(ORIGIN[0]))

# Every spelling seen in the wild, in one place. First group wins.
PATTERNS = {
    "lat": r"\[?latitude\s*:\s*([-\d.]+)",
    "lon": r"\[?long?itude\s*:\s*([-\d.]+)",
    "alt": r"\[?(?:rel_alt|altitude)\s*:\s*([-\d.]+)",
    "abs_alt": r"\[?abs_alt\s*:\s*([-\d.]+)",
    "gb_yaw": r"\[?gb_yaw\s*:\s*([-\d.]+)",
    "gb_pitch": r"\[?gb_pitch\s*:\s*([-\d.]+)",
    "gb_roll": r"\[?gb_roll\s*:\s*([-\d.]+)",
    # Legacy files carry height as BAROMETER and put something much less
    # trustworthy in the GPS triple's third slot, so this wins where both exist.
    "baro": r"BAROMETER\s*:?\s*([-\d.]+)",
}
# The camera's own wall clock, on its own line inside each block:
#     2026-08-21 16:31:09.139
# LOCAL time as the aircraft had it, with no timezone marker -- DJI writes no
# offset, so this is naive by construction and must not be pretended otherwise.
WALLCLOCK = re.compile(r"(\d{4})-(\d{2})-(\d{2})[ T]"
                       r"(\d{2}):(\d{2}):(\d{2})(?:[.,](\d{1,6}))?")

# The older GPS(lon,lat,alt) triple, which some firmware writes instead.
GPS_TRIPLE = re.compile(r"GPS\s*\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)")
FRAME_CNT = re.compile(r"FrameCnt\s*:?\s*(\d+)")
TIMECODE = re.compile(r"(\d{2}:\d{2}:\d{2}[,.]\d{3})\s*-->")


def to_enu(lon, lat):
    """WGS84 -> local metres about ORIGIN. Mirrors score_occupancy.to_enu."""
    return ((lon - ORIGIN[1]) * MLON, (lat - ORIGIN[0]) * MLAT)


def parse(path):
    """SRT -> [{frame, t, lat, lon, alt, ...}], one record per subtitle block."""
    text = open(path, encoding="utf-8", errors="replace").read()
    # Blocks are separated by a blank line; a block without a timecode is not
    # one (some files start with a BOM or a stray newline).
    blocks = [b for b in re.split(r"\n\s*\n", text) if TIMECODE.search(b)]
    out = []
    for i, block in enumerate(blocks):
        flat = block.replace("\n", " ")
        rec = {"block": i}
        m = FRAME_CNT.search(flat)
        rec["frame"] = int(m.group(1)) if m else i
        m = TIMECODE.search(flat)
        rec["t"] = m.group(1) if m else None
        # NOTE `t` above is the SUBTITLE timecode (00:00:05,004) -- time since
        # the video started, not a date. The wall clock is a separate line.
        m = WALLCLOCK.search(flat)
        if m:
            y, mo, d, hh, mm, ss, frac = m.groups()
            us = int((frac or "0").ljust(6, "0")[:6])
            rec["captured_at"] = (f"{y}-{mo}-{d}T{hh}:{mm}:{ss}"
                                  + (f".{us:06d}"[:7] if us else ""))
        for key, pat in PATTERNS.items():
            m = re.search(pat, flat, re.I)
            if m:
                rec[key] = float(m.group(1))
        if "lat" not in rec or "lon" not in rec:
            m = GPS_TRIPLE.search(flat)
            if m:
                rec["lon"], rec["lat"] = float(m.group(1)), float(m.group(2))
                rec.setdefault("alt", float(m.group(3)))
        out.append(rec)
    return out


def to_pose(rec, frame_idx=None):
    """One SRT record -> a pose dict shaped like the sim's poses.json entries.

    Angle conventions, stated because getting them wrong is silent:
      * `yaw` is the CAMERA heading in the ENU frame used by project(): x east,
        y north, measured counter-clockwise from east. DJI's gimbal yaw is a
        compass bearing (clockwise from north), hence the 90 - yaw.
      * `cam_pitch` follows the sim's convention where +pi/2 is straight down,
        while DJI reports -90 deg for straight down, hence the negation and
        offset.
      * `cam_roll` is carried through as radians about the optical axis.
    Anything absent is left OUT of the dict rather than defaulted, so
    project()'s documented nadir fallback applies explicitly.
    """
    if "lat" not in rec or "lon" not in rec:
        return None
    x, y = to_enu(rec["lon"], rec["lat"])
    pose = {"frame_idx": rec["frame"] if frame_idx is None else frame_idx,
            "x": round(x, 3), "y": round(y, 3),
            "alt": rec.get("baro", rec.get("alt", rec.get("abs_alt"))),
            "lat": rec["lat"], "lon": rec["lon"]}
    if "captured_at" in rec:
        pose["captured_at"] = rec["captured_at"]
    if "gb_yaw" in rec:
        pose["yaw"] = math.radians(90.0 - rec["gb_yaw"])
    if "gb_pitch" in rec:
        pose["cam_pitch"] = math.radians(-rec["gb_pitch"])
    if "gb_roll" in rec:
        pose["cam_roll"] = math.radians(rec["gb_roll"])
    return pose


def rebase(poses, start=None):
    """Shift every `captured_at` so the first frame lands at `start` (default:
    now), keeping the intervals between frames exactly as flown.

    This is how footage from a past flight is replayed "as if taken now" --
    needed because occupancy votes over a freshness window (OCCUPANCY_WINDOW_S,
    2 h), so ingesting yesterday's flight with yesterday's timestamps produces a
    map of bays that are all already expired.

    It is deliberately NOT done when the file is written. `poses.json` records
    what actually happened; rebasing is a REPLAY decision and belongs to the
    consumer, which may legitimately want either. Overwriting the real capture
    time in the file would throw away the only copy of it.

    Returns new dicts -- the input is not modified.
    """
    from datetime import datetime
    stamped = [p for p in poses if p.get("captured_at")]
    if not stamped:
        return [dict(p) for p in poses]
    t0 = datetime.fromisoformat(stamped[0]["captured_at"])
    delta = (start or datetime.now()) - t0
    out = []
    for p in poses:
        q = dict(p)
        if q.get("captured_at"):
            q["captured_at"] = (datetime.fromisoformat(q["captured_at"])
                                + delta).isoformat()
        out.append(q)
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if not args:
        raise SystemExit("usage: python tools/dji_srt.py <flight.SRT> "
                         "[--every N] [--json out.json] [--as-now]")
    path = args[0]
    every = 1
    if "--every" in sys.argv:
        every = int(sys.argv[sys.argv.index("--every") + 1])

    recs = parse(path)
    if not recs:
        raise SystemExit(f"no subtitle blocks found in {path} — is it an SRT?")
    have = {k: sum(1 for r in recs if k in r) for k in
            ("lat", "lon", "alt", "baro", "captured_at",
             "gb_yaw", "gb_pitch", "gb_roll")}
    print(f"{os.path.basename(path)}: {len(recs)} blocks")
    for k, n in have.items():
        state = "ok " if n == len(recs) else ("MISSING" if n == 0 else "partial")
        print(f"  {state:8s} {k:9s} {n}/{len(recs)}")
    if not have["gb_pitch"]:
        print("  note: no gimbal angles in this file — every pose will project "
              "as nadir, which is exactly the assumption that cost this project "
              "2.4 points of accuracy in the sim. Verify against the imagery "
              "(vision/diag/paint_align.py) before trusting any projection.")

    poses = [p for p in (to_pose(r) for r in recs[::every]) if p]
    if poses:
        xs = [p["x"] for p in poses]
        ys = [p["y"] for p in poses]
        alts = [p["alt"] for p in poses if p["alt"] is not None]
        print(f"  {len(poses)} poses (every {every} block(s))")
        print(f"  local ENU extent: x {min(xs):.1f}..{max(xs):.1f} m, "
              f"y {min(ys):.1f}..{max(ys):.1f} m about ORIGIN {ORIGIN}")
        if alts:
            print(f"  altitude {min(alts):.1f}..{max(alts):.1f} m")
        far = max(math.hypot(p["x"], p["y"]) for p in poses)
        if far > 2000:
            print(f"  ** {far / 1000:.1f} km from ORIGIN ** — this footage is "
                  f"not over the cut block. Run tools/cut_block.py for its "
                  f"coordinates first; do NOT move ORIGIN.")
    if "--as-now" in sys.argv:
        poses = rebase(poses)
        print("  timestamps REBASED so the first frame is now "
              "(intervals preserved) -- replay mode, not the real capture time")

    if "--json" in sys.argv:
        dest = sys.argv[sys.argv.index("--json") + 1]
        json.dump(poses, open(dest, "w"), indent=1)
        print(f"  -> {dest}")


if __name__ == "__main__":
    main()
