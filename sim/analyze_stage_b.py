#!/usr/bin/env python3
"""Report what the obstacle layer actually did on a --collide flight.

Reads only what the controller wrote to disk (poses.json + flight_log.csv) and
the world's hazard list, and answers the three questions a stage B run is judged
on:

  1. Did it get PAST anything, or just stop in front of it? (route progress)
  2. Did it stay outside the standoff while doing so? (closest approach)
  3. What did the detours cost? (time, speed, skipped captures)

Usage:  python analyze_stage_b.py [survey_area]        # default fmi_block_obst

Note the closest-approach figure is the closest GATED return, i.e. the same
number the controller was steering against; scenery off to the side is excluded
exactly as it is in flight. A run with no obstacle sensors (any survey world)
reports that and exits - the layer is absent by design there, not broken.
"""
import csv
import json
import math
import os
import sys

AREA = sys.argv[1] if len(sys.argv) > 1 else "fmi_block_obst"
# Archived runs live in `<area>.<tag>/` next to the live one, so the world files
# are looked up under the base name - otherwise an archive silently reports a
# 0-waypoint route and quietly drops every route-relative figure.
WORLD = AREA.split(".")[0]
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "output", AREA)
WORLDS = os.path.join(HERE, "worlds")


def load(path, default=None):
    try:
        with open(path, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return default


poses = load(os.path.join(OUT, "poses.json"), [])
route = load(os.path.join(WORLDS, f"{WORLD}.route.json"), [])
hazards = load(os.path.join(WORLDS, f"{WORLD}.hazards.json"), [])
try:
    with open(os.path.join(OUT, "flight_log.csv"), encoding="utf-8") as fh:
        log = list(csv.DictReader(fh))
except OSError:
    log = []

if not poses:
    sys.exit(f"no poses in {OUT} - has this area been flown?")

print(f"=== {AREA} ===")

# --- 1. route progress -----------------------------------------------------
idxs = [p["frame_idx"] for p in poses]
reached, total = max(idxs), len(route)
missing = [i for i in range(reached + 1) if i not in set(idxs)]
print(f"\nRoute      : reached wp{reached} of {total}"
      f"  ({len(poses)} frames captured)")
if missing:
    print(f"  skipped  : {len(missing)} waypoint(s) {missing} - inside an "
          f"obstacle, deliberately not captured")
print(f"  covered  : {len(poses)}/{reached + 1} of the waypoints flown, "
      f"{100.0 * len(poses) / (reached + 1):.0f}%")

# --- 2. clearance ----------------------------------------------------------
gated = [float(r["obst_m"]) for r in log if r.get("obst_m")]
if not gated:
    print("\nNo gated obstacle returns in the flight log: this world has no "
          "DistanceSensor fan (built without --collide), so the obstacle layer "
          "was switched off, as it is on every survey world.")
    sys.exit(0)
print(f"\nClearance  : closest gated return {min(gated):.2f} m"
      f"   (standoff target 12 m)")
if min(gated) < 11.0:
    print("  ** BELOW THE STANDOFF ** - check for creep while halted: a "
          "station-keep that is not latched drifts in at ~0.07 m/s, which is "
          "invisible per-step and 6 m over two minutes.")

# --- 2b. how far it wandered ----------------------------------------------
# The number that matches what you see out of the window. A detour is supposed
# to be a local excursion around one structure; if the drone is tens of metres
# off its own route it is not avoiding an obstacle, it is sightseeing.
def route_dist(px, py):
    best = float("inf")
    for a, b in zip(route, route[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / L2))
        best = min(best, math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy)))
    return best

if route and log:
    off = [route_dist(float(r["x"]), float(r["y"])) for r in log[::5]]
    steer_off_d = [route_dist(float(r["x"]), float(r["y"]))
                   for r in log[::5] if r.get("steering") == "1"]
    print(f"{chr(10)}Excursion  : max {max(off):.1f} m off the planned route"
          f"   (while steering: max "
          f"{max(steer_off_d) if steer_off_d else 0.0:.1f} m, mean "
          f"{sum(steer_off_d)/len(steer_off_d) if steer_off_d else 0.0:.1f} m)")

# --- 3. what the layer did -------------------------------------------------
def episodes(flag):
    out, cur = [], None
    for r in log:
        if r.get(flag) == "1":
            cur = cur or []
            cur.append(r)
        elif cur:
            out.append(cur)
            cur = None
    return out + ([cur] if cur else [])

steer = episodes("steering")
print(f"\nStage B    : {len(steer)} detour(s)")
for e in steer:
    t0, t1 = float(e[0]["t"]), float(e[-1]["t"])
    spd = sum(float(r["speed"]) for r in e) / len(e)
    dist = math.hypot(float(e[-1]["x"]) - float(e[0]["x"]),
                      float(e[-1]["y"]) - float(e[0]["y"]))
    side = "left" if e[0].get("side") == "1" else "right"
    # A detour that averages well under the 2 m/s cap did not fly an arc, it
    # crawled - which historically meant something else had pinned v_des to 0
    # (the stop-and-turn latch) or the drone was held on a station-keep point.
    note = "  <-- crawled, check v_des/hold in the log" if spd < 1.0 else ""
    print(f"  {t0:6.0f}->{t1:6.0f} s ({t1 - t0:4.0f} s) {side:5s} "
          f"net {dist:5.1f} m at {spd:.2f} m/s{note}")

avoid = episodes("avoiding")
print(f"Stage D    : {len(avoid)} braking episode(s), "
      f"{sum(1 for p in poses if p.get('avoiding'))} frame(s) flagged "
      f"`avoiding`, {sum(1 for p in poses if p.get('steering'))} flagged "
      f"`steering`")
print("             (both flags mean the frame was taken off the calibrated "
      "survey path - the scorer should exclude them, not score them)")

# --- 4. the structures it met ---------------------------------------------
synth = [h for h in hazards if h.get("synthetic")]
if synth:
    print(f"\nStructures in this world ({len(synth)} synthetic):")
    end = (float(log[-1]["x"]), float(log[-1]["y"])) if log else (0.0, 0.0)
    for h in synth:
        cx, cy = h["centroid"]
        near = min(range(len(route)),
                   key=lambda i: math.hypot(route[i][0] - cx, route[i][1] - cy))
        got = "passed" if reached > near else (
            "STOPPED HERE" if math.hypot(end[0] - cx, end[1] - cy) < 40 else
            "not reached")
        print(f"  {h['osm_id']:11s} {h['height_m']:5.1f} m at "
              f"({cx:7.1f},{cy:7.1f})  nearest wp{near:<3d}  {got}")

print(f"\nFlight ended at ({log[-1]['x']},{log[-1]['y']}) after "
      f"{float(log[-1]['t']):.0f} s of sim time." if log else "")
