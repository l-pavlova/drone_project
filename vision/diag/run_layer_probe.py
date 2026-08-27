"""Exercise the curb-run layer on a real flight, from cached detections.

    python vision/diag/run_layer_probe.py pics/dji/stills/DJI_..._0035_D \
           --dets pics/dji/demo_0035.mp4.dets.json

No detector pass: this reads a detections cache, so the run layer can be measured
and re-measured in seconds. It answers three things the per-bay pipeline cannot:

  * how many detections the run layer PLACES, against the 3 m per-bay rule which
    places only 122 of 386 hand-labelled cars (32%) and leaves four in five
    attributed to nothing;
  * what one robust lateral scalar per run is actually worth, reported as the
    shift and its MAD so a fit with no support is visible rather than silent;
  * free capacity per run, beside `observed_fraction`, because a gap nobody looked
    at is not a free space.

It deliberately reports, and never writes anything.
"""
import argparse
import collections
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import curb_runs as cr                                          # noqa: E402
import detect_occupancy as dof                                  # noqa: E402
import cameras                                                  # noqa: E402
import score_occupancy as so                                    # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q / 100.0 * len(v)))] if v else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--dets", required=True, help="detections cache json")
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--limit", type=int)
    ap.add_argument("--lateral-max", type=float, default=6.0)
    ap.add_argument("--no-correct", action="store_true",
                    help="skip the per-run lateral fit")
    a = ap.parse_args()

    cam = cameras.get(a.cam)
    poses = json.load(open(os.path.join(a.stills, "poses.json"), encoding="utf-8"))
    poses = [p for p in poses if "yaw" in p and "x" in p]
    if a.limit:
        poses = poses[:a.limit]
    cache = json.load(open(a.dets, encoding="utf-8"))
    frames = cache["frames"] if "frames" in cache else cache

    def dets_for(p):
        for key in (p["file"], os.path.splitext(p["file"])[0],
                    str(p.get("video_frame")), str(p.get("frame_idx"))):
            if key in frames:
                return [(d["box"], d["conf"]) for d in frames[key]]
        return None

    runs = cr.load()
    print(f"{len(runs)} verified runs, {len(poses)} poses, "
          f"{sum(len(v) for v in frames.values())} cached detections")

    # ---- pass 1: where do the cars fall relative to each run? ---------------
    pts_by_run = collections.defaultdict(list)
    all_lat = []
    placed = total = 0
    for p in poses:
        ds = dets_for(p)
        if ds is None:
            continue
        for box, conf in ds:
            total += 1
            g = so.unproject((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0,
                             p, cam=cam)
            if g is None:
                continue
            best = None
            for r in runs:
                got = cr.locate(r, g[0], g[1])
                if got is None:
                    continue
                s, lat, dist = got
                if dist > a.lateral_max:
                    continue
                if best is None or dist < best[0]:
                    best = (dist, r, lat)
            if best is not None:
                placed += 1
                pts_by_run[best[1]["run_id"]].append((g[0], g[1]))
                all_lat.append(best[2])
    print(f"\ndetections placed on a run within {a.lateral_max} m: "
          f"{placed}/{total} ({100.0 * placed / max(1, total):.0f}%)")
    if all_lat:
        print(f"  lateral offset: median {pct(all_lat, 50):+.2f} m  "
              f"p10 {pct(all_lat, 10):+.2f}  p90 {pct(all_lat, 90):+.2f}")

    # ---- pass 2: one robust scalar per run ---------------------------------
    fitted = {}
    if not a.no_correct:
        shifts = []
        for r in runs:
            pts = pts_by_run.get(r["run_id"], [])
            delta, n, mad = cr.estimate_lateral(r, pts)
            if mad is not None:
                fitted[r["run_id"]] = delta
                shifts.append((abs(delta), delta, n, mad, r))
        shifts.sort(reverse=True)
        print(f"\n{len(fitted)} of {len(runs)} runs had enough support to fit "
              f"a lateral shift")
        if shifts:
            ds = [s[1] for s in shifts]
            ms = [s[3] for s in shifts]
            print(f"  |shift|: median {pct([abs(d) for d in ds], 50):.2f} m  "
                  f"max {max(abs(d) for d in ds):.2f}")
            print(f"  MAD    : median {pct(ms, 50):.2f} m  "
                  f"(a tight MAD means the run really is displaced, not scattered)")
            print("  largest:")
            for _, d, n, mad, r in shifts[:5]:
                print(f"    {r['run_id']} {str(r['street'])[:26]:26s} "
                      f"n={n:3d}  shift {d:+.2f} m  MAD {mad:.2f}")

    runs = [cr.shift_lateral(r, fitted[r["run_id"]]) if r["run_id"] in fitted else r
            for r in runs]

    # ---- pass 3: intervals + observation, frame by frame --------------------
    per_run = collections.defaultdict(list)
    seen = collections.Counter()
    for p in poses:
        ds = dets_for(p)
        if ds is None:
            continue
        per = dof.run_intervals_from_dets(ds, runs, p, cam=cam,
                                          lateral_max_m=a.lateral_max)
        for rid, rec in per.items():
            if rec["observed"]:
                per_run[rid].append(rec)
            seen[rid] += rec["dets"]

    results = []
    for r in runs:
        frames_r = per_run.get(r["run_id"])
        if not frames_r:
            continue
        occ, obs, _views = cr.vote_cells(r, frames_r)
        res = cr.free_gaps(r, occ, obs)
        # detections that landed on this run but lost the majority vote. Same
        # class of signal as `unassigned` in the per-bay rule: a run reporting
        # "all free" while 18 cars were detected on it is a loud fact, and one
        # that is invisible unless it is counted.
        res["dets_seen"] = seen[r["run_id"]]
        res["outvoted"] = res["dets_seen"] > 0 and res["occupied_len_m"] == 0.0
        results.append(res)

    touched = [x for x in results if x["observed_fraction"] > 0.05]
    print(f"\n{len(touched)} runs observed by this flight "
          f"(>5% of their length)")
    print(f"{'run':10s} {'street':26s} {'cap':>4s} {'free':>5s} {'occ_m':>7s} "
          f"{'seen':>5s} {'obs':>5s}")
    for x in sorted(touched, key=lambda z: -z["observed_fraction"])[:18]:
        r = next(q for q in runs if q["run_id"] == x["run_id"])
        print(f"{x['run_id']:10s} {str(r['street'])[:26]:26s} "
              f"{x['capacity']:4d} {x['free']:5d} {x['occupied_len_m']:7.1f} "
              f"{seen[x['run_id']]:5d} {x['observed_fraction']:5.2f}")

    lost = [x for x in touched if x["outvoted"]]
    if lost:
        print()
        print(f"{len(lost)} run(s) report NO occupied curb despite detections "
              f"landing on them ({sum(x['dets_seen'] for x in lost)} in total):")
        for x in lost:
            print(f"    {x['run_id']}  {x['dets_seen']:3d} detections, "
                  f"observed {x['observed_fraction']:.2f} -- no cell reached a "
                  f"strict majority")

    cap = sum(x["capacity"] for x in touched)
    free = sum(x["free"] for x in touched)
    print(f"\ntotals over observed runs: capacity {cap}, free {free}, "
          f"occupied {cap - free} ({100.0 * (cap - free) / max(1, cap):.0f}%)")
    print(f"detections landing on an observed run: {sum(seen.values())}")


if __name__ == "__main__":
    main()
