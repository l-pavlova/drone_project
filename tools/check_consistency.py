#!/usr/bin/env python3
"""
PARKDRONE consistency check: constants and rules duplicated across files.

Several values in this project necessarily live in more than one place - the
Webots controller runs under Webots' own Python and cannot import the vision
code, the web tier is TypeScript, and the generator must rebuild geometry the
GIS tools produced. Every one of those duplications is a silent-failure waiting
to happen, and two have already fired:

  * `Косо` (angled) bays - tools/make_bays.py had three park_txt cases and
    sim/generate_world.py had two, so all 12 angled bays in the 1 km world were
    drawn 45 deg off their painted rectangle (2.23 m corner error). Invisible for
    weeks because fmi_block and fmi_block_4st contain ZERO angled bays.
  * The georeferencing ORIGIN, which is the spine of the whole project: if the
    generator and the scorer disagree, every bay is mis-projected and the
    occupancy answer is wrong rather than missing.

This script asserts they still agree. It parses the sources as TEXT rather than
importing them, because generate_world.py builds a world as a side effect of
import and parkdrone.py needs a live Webots.

    python tools/check_consistency.py          # exits 1 on any mismatch

Run it after touching any of: ORIGIN/projection, the DS_* sensor constants, bay
dimensions or orientation rules, or the camera intrinsics.

Verified by mutation: deleting the `Косо` branch, drifting ORIGIN by 1e-5 deg
(~1 m) and changing DS_RANGE in one file each make it exit 1 with a diagnostic
naming the two disagreeing sources.

KNOWN LIMITATION worth understanding before trusting a green run. Check 4
(per-bay geometry) *re-implements* the three orientation rules rather than
executing generate_world.py's copy of them, because importing that module builds
a world as a side effect. So check 4 validates the DATA against the intended
rules; it is check 3, which parses generate_world.py as text, that ties the
generator to them. If the orientation logic is ever restructured enough that
check 3's regexes stop matching, that is reported as a FAILURE rather than
silently passing - but the regexes will then need updating together with this
file's copy of the rules.
"""
import json, math, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
GEN = os.path.join(ROOT, "sim", "generate_world.py")
CTRL = os.path.join(ROOT, "sim", "controllers", "parkdrone", "parkdrone.py")
SCORE = os.path.join(ROOT, "vision", "score_occupancy.py")
MAKE = os.path.join(HERE, "make_bays.py")
GEO = os.path.join(ROOT, "web", "packages", "contracts", "src", "geo.ts")
BAYS = os.path.join(ROOT, "data", "block_bays.geojson")
WORLDS = os.path.join(ROOT, "sim", "worlds")
OUTPUT = os.path.join(ROOT, "sim", "output")

failures, checks, skips = [], 0, []


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def grab(path, pattern, label):
    """First regex group in `path`, or None (recorded as a failure)."""
    m = re.search(pattern, read(path), re.M)
    if not m:
        failures.append(f"{label}: pattern not found in {os.path.relpath(path, ROOT)} "
                        f"-- the constant was renamed or removed, so this check "
                        f"is no longer protecting anything")
        return None
    return m.groups() if len(m.groups()) > 1 else m.group(1)


def same(label, values, tol=0.0):
    """values = [(source_name, value)]; all must agree within tol."""
    global checks
    checks += 1
    vals = [v for _, v in values if v is not None]
    if len(vals) < 2:
        return
    ref = vals[0]
    for name, v in values:
        if v is None:
            continue
        ok = (abs(v - ref) <= tol) if isinstance(v, float) else (v == ref)
        if not ok:
            failures.append(f"{label}: {name} = {v!r} but {values[0][0]} = {ref!r}")
            return
    print(f"  ok  {label} = {ref}")


print("PARKDRONE consistency check\n")

# --- 1. the ENU projection: ORIGIN / MLAT / MLON -------------------------------
# The spine of the project. Python sim, Python vision and TypeScript web tier all
# project lon/lat into the same local metres; a drift here silently forks the
# georeference and every downstream number becomes meaningless.
print("projection (ORIGIN / MLAT / MLON)")
g_lat, g_lon = grab(GEN, r"^ORIGIN = \(([\d.]+), ([\d.]+)\)", "ORIGIN(generate_world)") or (None, None)
s_lat, s_lon = grab(SCORE, r"^ORIGIN = \(([\d.]+), ([\d.]+)\)", "ORIGIN(score_occupancy)") or (None, None)
t_lat = grab(GEO, r"ORIGIN = \{ lat: ([\d.]+)", "ORIGIN(geo.ts) lat")
t_lon = grab(GEO, r"lon: ([\d.]+) \}", "ORIGIN(geo.ts) lon")
same("ORIGIN.lat", [("generate_world.py", float(g_lat) if g_lat else None),
                    ("score_occupancy.py", float(s_lat) if s_lat else None),
                    ("geo.ts", float(t_lat) if t_lat else None)])
same("ORIGIN.lon", [("generate_world.py", float(g_lon) if g_lon else None),
                    ("score_occupancy.py", float(s_lon) if s_lon else None),
                    ("geo.ts", float(t_lon) if t_lon else None)])
same("MLAT", [("generate_world.py", float(grab(GEN, r"^mlat = ([\d.]+)", "mlat") or 0)),
              ("score_occupancy.py", float(grab(SCORE, r"^MLAT = ([\d.]+)", "MLAT") or 0)),
              ("geo.ts", float(grab(GEO, r"MLAT = ([\d.]+)", "MLAT(ts)") or 0))])
# MLON is derived (111320*cos(lat0)) in all three, so checking the formula's
# inputs is enough - but verify each really derives it rather than hardcoding.
for path, pat, name in ((GEN, r"mlon = 111320\.0 \* math\.cos", "generate_world.py"),
                        (SCORE, r"MLON = 111320\.0 \* math\.cos", "score_occupancy.py"),
                        (GEO, r"MLON = 111320\.0 \* Math\.cos", "geo.ts")):
    checks += 1
    if not re.search(pat, read(path)):
        failures.append(f"MLON: {name} no longer derives MLON as 111320*cos(lat0) "
                        f"- a hardcoded value will drift from the others")
    else:
        print(f"  ok  MLON derived from cos(ORIGIN.lat) in {name}")

# --- 2. obstacle sensor fan ----------------------------------------------------
# The generator MOUNTS the rays and the controller READS them by index; a
# mismatch means the controller reads a ray that points somewhere else, and the
# avoidance layer brakes for the wrong bearing.
print("\nobstacle sensor fan (DS_*)")
for const, cast in (("DS_N", int), ("DS_SPREAD_DEG", float), ("DS_RANGE", float)):
    a = grab(GEN, rf"^{const} = ([\d.]+)", f"{const}(generate_world)")
    b = grab(CTRL, rf"^{const} = ([\d.]+)", f"{const}(parkdrone)")
    same(const, [("generate_world.py", cast(a) if a else None),
                 ("parkdrone.py", cast(b) if b else None)])

# --- 3. bay dimensions and orientation rules -----------------------------------
# make_bays.py writes the geojson RING; generate_world.py rebuilds the rectangle
# from (centre, bearing_deg, L, W) and the vision scorer reads the ring verbatim.
# All three must describe the same rectangle. This is where the Косо bug lived.
print("\nbay dimensions (make_bays.py vs generate_world.py)")
mk = read(MAKE)
par = re.search(r"^PARALLEL = \(([\d.]+), ([\d.]+)\)", mk, re.M)
per = re.search(r"^PERP = \(([\d.]+), ([\d.]+)\)", mk, re.M)
ang = re.search(r"^ANGLE_DEG = ([\d.]+)", mk, re.M)
gen = read(GEN)
g_perp = re.search(r'"Напречн":[^\n]*\n\s*ang = brg \+ math\.pi / 2; L, W = ([\d.]+), ([\d.]+)', gen)
g_kos = re.search(r'"Косо":.*?\n\s*ang = brg \+ math\.radians\(([\d.]+)\); L, W = ([\d.]+), ([\d.]+)', gen)
g_par = re.search(r"else:\s*#[^\n]*parallel\s*\n\s*ang = brg; L, W = ([\d.]+), ([\d.]+)", gen)
if not (par and per and ang):
    failures.append("bay dims: could not parse PARALLEL/PERP/ANGLE_DEG from make_bays.py")
if not g_perp:
    failures.append("bay dims: could not parse the 'Напречн' branch from generate_world.py")
if not g_kos:
    failures.append("bay orientation: generate_world.py has NO 'Косо' branch. "
                    "make_bays.py rotates angled bays by ANGLE_DEG; without the "
                    "matching branch every angled bay is drawn at the street "
                    "bearing and is mis-georeferenced (this bug cost 12 bays in "
                    "the 1 km world, 2.23 m corner error each)")
if not g_par:
    failures.append("bay dims: could not parse the parallel branch from generate_world.py")
if par and g_par:
    same("bay PARALLEL length", [("make_bays.py", float(par.group(1))),
                                 ("generate_world.py", float(g_par.group(1)))])
    same("bay PARALLEL width", [("make_bays.py", float(par.group(2))),
                                ("generate_world.py", float(g_par.group(2)))])
if per and g_perp:
    same("bay PERP depth", [("make_bays.py", float(per.group(1))),
                            ("generate_world.py", float(g_perp.group(1)))])
    same("bay PERP width", [("make_bays.py", float(per.group(2))),
                            ("generate_world.py", float(g_perp.group(2)))])
if ang and g_kos:
    same("bay ANGLE_DEG", [("make_bays.py", float(ang.group(1))),
                           ("generate_world.py", float(g_kos.group(1)))])

# --- 4. every bay: drawn rectangle vs stored ring ------------------------------
# The end-to-end version of check 3, and the one that actually caught the Косо
# bug. Rebuilds each bay the way generate_world.py does and compares it with the
# geojson ring the scorer reads. Independent of how the rules are spelled.
print("\nper-bay geometry (drawn rectangle vs geojson ring)")
TOL = 0.05          # m; float printing alone costs ~2 mm


def enu(lon, lat, lat0, lon0):
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    return (lon - lon0) * mlon, (lat - lat0) * mlat


def rect_corners(cx, cy, a, L, W):
    c, s = math.cos(a), math.sin(a)
    return [(cx + c*dx - s*dy, cy + s*dx + c*dy)
            for dx, dy in ((L/2, W/2), (L/2, -W/2), (-L/2, -W/2), (-L/2, W/2))]


if os.path.exists(BAYS) and g_lat:
    lat0, lon0 = float(g_lat), float(g_lon)
    feats = json.load(open(BAYS, encoding="utf-8"))["features"]
    gts = sorted(f for f in os.listdir(WORLDS) if f.endswith("ground_truth.json"))
    if not gts:
        skips.append("per-bay geometry: no ground_truth.json in sim/worlds")
    for gt_name in gts:
        gt = json.load(open(os.path.join(WORLDS, gt_name), encoding="utf-8"))
        worst, worst_id, n = 0.0, None, 0
        for f in feats:
            pr = f["properties"]
            bid = str(pr.get("id"))
            if bid not in gt:
                continue
            ring = [enu(lon, lat, lat0, lon0)
                    for lon, lat in f["geometry"]["coordinates"][0][:-1]]
            if len(ring) != 4:
                continue
            cx = sum(p[0] for p in ring) / 4
            cy = sum(p[1] for p in ring) / 4
            brg = math.radians(pr.get("bearing_deg", 0.0))
            pk = pr.get("park_txt")
            if pk == "Напречн":
                a, L, W = brg + math.pi / 2, 4.8, 2.4
            elif pk == "Косо":
                a, L, W = brg + math.radians(45.0), 5.4, 2.2
            else:
                a, L, W = brg, 5.4, 2.2
            drawn = rect_corners(cx, cy, a, L, W)
            d = max(min(math.dist(p, q) for q in drawn) for p in ring)
            n += 1
            if d > worst:
                worst, worst_id = d, bid
        checks += 1
        if worst > TOL:
            failures.append(f"per-bay geometry [{gt_name}]: bay {worst_id} is "
                            f"{worst:.3f} m out (tolerance {TOL} m) - the world "
                            f"draws it somewhere the scorer does not look")
        else:
            print(f"  ok  {gt_name}: {n} bays, worst corner error {worst*1000:.1f} mm")
else:
    skips.append("per-bay geometry: block_bays.geojson or ORIGIN unavailable")

# --- 5. camera intrinsics vs the frames actually captured ----------------------
# score_occupancy.py hardcodes the Mavic2Pro camera resolution. Rather than
# compare it with the proto (which lives in a content-addressed cache), compare
# it with real captured frames: those are ground truth for what the camera did.
print("\ncamera intrinsics (score_occupancy.py vs captured frames)")
iw = re.search(r"^IMG_W, IMG_H = (\d+), (\d+)", read(SCORE), re.M)
if not iw:
    failures.append("camera: could not parse IMG_W/IMG_H from score_occupancy.py")
elif os.path.isdir(OUTPUT):
    want = (int(iw.group(1)), int(iw.group(2)))
    seen = 0
    try:
        from PIL import Image
        for area in sorted(os.listdir(OUTPUT)):
            d = os.path.join(OUTPUT, area)
            if not os.path.isdir(d) or "." in area:      # skip archived runs
                continue
            frames = [f for f in os.listdir(d) if f.startswith("frame_")]
            if not frames:
                continue
            got = Image.open(os.path.join(d, sorted(frames)[0])).size
            checks += 1
            seen += 1
            if got != want:
                failures.append(f"camera: {area} frames are {got[0]}x{got[1]} but "
                                f"score_occupancy.py projects with {want[0]}x{want[1]} "
                                f"- every bay is projected at the wrong scale")
            else:
                print(f"  ok  {area}: frames {got[0]}x{got[1]}")
        if not seen:
            skips.append("camera intrinsics: no captured frames to compare against")
    except ImportError:
        skips.append("camera intrinsics: Pillow not installed")
else:
    skips.append("camera intrinsics: sim/output missing")

# --- 6. camera FOV: score_occupancy.py vs the controller ----------------------
# The controller grew its own copy of the intrinsics on 2026-08-20, when the
# standoff capture started asking "is this waypoint in the picture" - a question
# that cannot be answered without the FOV and the aspect ratio. Check 5 compares
# the RESOLUTION against real frames; nothing compared the FOV against anything,
# and a drifted FOV here does not crash and does not look wrong - it silently
# shifts which skipped waypoints are judged recoverable, which is exactly the
# class of bug this script exists for.
print("\ncamera FOV (score_occupancy.py vs parkdrone.py)")
fov_s = re.search(r"^FOV = ([\d.]+)", read(SCORE), re.M)
fov_c = re.search(r"^CAM_FOV = ([\d.]+)", read(CTRL), re.M)
asp_c = re.search(r"^CAM_ASPECT = ([\d.]+) / ([\d.]+)", read(CTRL), re.M)
checks += 1
if not (fov_s and fov_c):
    failures.append("camera FOV: could not parse FOV from score_occupancy.py "
                    "or CAM_FOV from parkdrone.py")
elif abs(float(fov_s.group(1)) - float(fov_c.group(1))) > 1e-9:
    failures.append(f"camera FOV: score_occupancy.py has {fov_s.group(1)} but "
                    f"parkdrone.py has {fov_c.group(1)}")
else:
    print(f"  ok  FOV {fov_s.group(1)} rad in both")
checks += 1
if not (iw and asp_c):
    failures.append("camera FOV: could not parse CAM_ASPECT from parkdrone.py")
elif (asp_c.group(1), asp_c.group(2)) != (f"{iw.group(2)}.0", f"{iw.group(1)}.0"):
    # Spelled height/width so it reads as the image, not as a magic 0.6.
    failures.append(f"camera FOV: CAM_ASPECT is {asp_c.group(1)}/{asp_c.group(2)} "
                    f"but the image is {iw.group(1)}x{iw.group(2)} "
                    f"(expected {iw.group(2)}.0 / {iw.group(1)}.0)")
else:
    print(f"  ok  CAM_ASPECT {asp_c.group(1)}/{asp_c.group(2)} matches "
          f"{iw.group(1)}x{iw.group(2)}")

# --- report -------------------------------------------------------------------
print()
for s in skips:
    print(f"SKIP  {s}")
if failures:
    print(f"\nFAILED ({len(failures)} of {checks} checks):\n")
    for f in failures:
        print(f"  * {f}")
    sys.exit(1)
print(f"\nall {checks} checks passed"
      f"{f' ({len(skips)} skipped)' if skips else ''}")
