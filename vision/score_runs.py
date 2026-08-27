"""Score a REAL flight against hand-counted CARS PER CURB RUN.

    python vision/score_runs.py pics/dji/stills/DJI_..._0035_D \
           --gt-runs vision/data/gt_runs_0035.json \
           --dets pics/dji/demo_0035.mp4.dets.json

The first accuracy number this project can defend on real footage. Everything
before it was either simulated (where `ground_truth.json` exists) or unmeasured --
`/api/v1/metrics` has always and correctly reported real-flight accuracy as `null`.

**Why the unit is a car count on a stretch of kerb, not a per-bay verdict.** Two
independent hand-annotation passes over this flight agreed to 0.18 m ACROSS the row
and differed by 1.95 m ALONG it, because the parking is unmarked cobble and nothing
in the image says where one bay ends and the next begins. "Is bay 17690 occupied" is
therefore not answerable to better than half a bay here; "how many cars are parked
between these two points" is answerable exactly.

**Three predictions are compared against the same human counts**, because the
interesting question is not whether the run layer is good but whether it is BETTER,
and against what:

  * `instances` -- detections clustered across frames into distinct cars, counted
    per segment. This is the most direct comparison: a car is a car.
  * `run layer` -- the voted occupied length divided by a car's length. This is the
    quantity the product actually derives free spaces from, so its error is the
    error a driver would feel.
  * `per-bay` -- the committed `bay_votes_from_dets` rule, counted as the number of
    bays it calls occupied whose centre falls in the segment. The incumbent.

**An unlabelled segment is not a zero.** Only segments the human actually counted
are scored; `skip` and never-labelled are both excluded, the same discipline
`score_real.py` gives a skipped bay and `score_occupancy.py` gives a closed street.
"""
import argparse
import collections
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cameras                                                   # noqa: E402
import curb_runs as cr                                           # noqa: E402
import detect_occupancy as dio                                   # noqa: E402
import score_occupancy as so                                     # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

CAR_M = cr.CAR_LEN_M     # median unprojected car length; owned by curb_runs


def load_dets(stills, path, cam, model, conf, imgsz):
    """-> [(pose, [(box, conf), ...]), ...]"""
    poses = json.load(open(os.path.join(stills, "poses.json"), encoding="utf-8"))
    poses = [p for p in poses if "yaw" in p and "x" in p]
    if path:
        doc = json.load(open(path, encoding="utf-8"))
        frames = doc.get("frames", doc)
        out = []
        for p in poses:
            for key in (p["file"], os.path.splitext(p["file"])[0],
                        str(p.get("video_frame")), str(p.get("frame_idx"))):
                if key in frames:
                    out.append((p, [(d["box"], d["conf"]) for d in frames[key]]))
                    break
        print(f"detections from cache: {path} ({len(out)} frames matched)")
        return out

    import contextlib
    import io as _io
    grab = []
    real = dio.bay_votes_from_dets

    def spy(dets, bays, pose, cam=None, **kw):
        grab.append((pose, dets))
        return real(dets, bays, pose, cam=cam, **kw)

    dio.bay_votes_from_dets = spy
    buf = _io.StringIO()
    with contextlib.redirect_stdout(buf):
        dio.run(stills, model, conf, cam, imgsz=imgsz)
    dio.bay_votes_from_dets = real
    return grab


def overlap(a0, a1, b0, b1):
    return max(0.0, min(a1, b1) - max(a0, b0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--gt-runs", required=True, help="gt_runs_*.json from label_runs.py")
    ap.add_argument("--dets", help="detections cache json (else the detector is run)")
    ap.add_argument("--runs", help="curb_runs.geojson")
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--model", default=os.path.join(os.path.dirname(
        os.path.abspath(__file__)), "runs", "merged1", "weights", "best.pt"))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=1024)
    ap.add_argument("--car-m", type=float, default=CAR_M)
    ap.add_argument("--no-correct", action="store_true",
                    help="skip the per-run lateral fit")
    a = ap.parse_args()

    cam = cameras.get(a.cam)
    gt = json.load(open(a.gt_runs, encoding="utf-8"))
    segs = {k: v for k, v in gt["segments"].items() if not v.get("skip")
            and v.get("count") is not None}
    skipped = len(gt["segments"]) - len(segs)

    runs = {r["run_id"]: r for r in cr.load(a.runs)}

    # ---- physical plausibility gate ----------------------------------------
    # A parallel-parked car occupies at least ~4.4 m of kerb, so a count implying
    # less than that is not a tight row, it is cars from somewhere else -- in
    # practice the SECOND row across the street, which is visible in the same tile
    # wherever the two rows are close. Silently averaging such a segment in would
    # inflate the truth and make every prediction look like an under-count.
    # Flagged rather than repaired: the fix is a human recounting five tiles, not
    # this file guessing which cars were meant.
    impl = []
    for k, v in sorted(segs.items()):
        r = runs.get(v["run_id"])
        if not r or not v["count"]:
            continue
        floor = 2.2 if r.get("park_txt") == "Напречн" else 4.0
        per = (v["s1"] - v["s0"]) / v["count"]
        if per < floor:
            impl.append((k, v["count"], v["s1"] - v["s0"], per, floor))
    if impl:
        print(f"IMPLAUSIBLE: {len(impl)} segment(s) counted denser than a car fits "
              f"-- EXCLUDED from the score")
        for k, c, L, per, floor in impl:
            print(f"    {k:14s} {c} cars in {L:.1f} m = {per:.2f} m/car "
                  f"(floor {floor:.1f}) -- likely the far row was counted too")
        segs = {k: v for k, v in segs.items() if k not in {x[0] for x in impl}}

    total_gt = sum(v["count"] for v in segs.values())
    print(f"ground truth: {len(segs)} segment(s) scored, {total_gt} cars"
          + (f", {skipped} skipped" if skipped else "")
          + (f", {len(impl)} implausible" if impl else ""))
    frames = load_dets(a.stills, a.dets, cam, a.model, a.conf, a.imgsz)
    if not frames:
        sys.exit("no frames matched -- wrong stills folder or detections cache?")

    # ---- per-run lateral fit (the one registration step) -------------------
    fitted = {}
    if not a.no_correct:
        pts_by_run = collections.defaultdict(list)
        for pose, dets in frames:
            for box, conf in dets:
                g = so.unproject((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0,
                                 pose, cam=cam)
                if not g:
                    continue
                best = None
                for r in runs.values():
                    got = cr.locate(r, g[0], g[1])
                    if got and got[2] <= 6.0 and (best is None or got[2] < best[0]):
                        best = (got[2], r["run_id"])
                if best:
                    pts_by_run[best[1]].append((g[0], g[1]))
        for rid, r in list(runs.items()):
            delta, n, mad = cr.estimate_lateral(r, pts_by_run.get(rid, []))
            if mad is not None:
                runs[rid] = cr.shift_lateral(r, delta)
                fitted[rid] = (delta, n, mad)
        print(f"lateral fit: {len(fitted)} run(s) had enough support "
              f"(>= {cr.estimate_lateral.__defaults__[1]} detections)")

    # ---- three predictions --------------------------------------------------
    rl = collections.defaultdict(list)
    for pose, dets in frames:
        per = dio.run_intervals_from_dets(dets, list(runs.values()), pose, cam=cam)
        for rid, rec in per.items():
            if rec["observed"]:
                rl[rid].append(rec)
    voted = {rid: cr.vote_cells(runs[rid], fr)[0] for rid, fr in rl.items()}

    cars = dio.cluster_instances(frames, cam)
    car_s = collections.defaultdict(list)
    for x, y, _n in cars:
        best = None
        for r in runs.values():
            got = cr.locate(r, x, y)
            if got and got[2] <= 6.0 and (best is None or got[2] < best[0]):
                best = (got[2], r["run_id"], got[1] if False else got[0])
        if best:
            car_s[best[1]].append(best[2])
    print(f"car instances: {len(cars)} distinct cars from "
          f"{sum(len(d) for _p, d in frames)} detections "
          f"({sum(len(v) for v in car_s.values())} placed on a run)")

    bays = so.load_all_bays()
    per_frame = [dio.bay_votes_from_dets(d, bays, p, cam=cam) for p, d in frames]
    bay_vote = dio.vote(per_frame)
    bay_pos = {}
    for b in bays:
        for r in runs.values():
            got = cr.locate(r, b["cx"], b["cy"])
            if got and got[2] <= 6.0:
                bay_pos.setdefault(b["id"], (r["run_id"], got[0]))
                break

    # ---- score --------------------------------------------------------------
    rows = []
    for sid, v in sorted(segs.items()):
        rid, s0, s1 = v["run_id"], v["s0"], v["s1"]
        if rid not in runs:
            continue
        occ = sum(overlap(x0, x1, s0, s1) for x0, x1 in voted.get(rid, []))
        rows.append({
            "seg": sid, "run": rid, "gt": v["count"],
            "inst": sum(1 for s in car_s.get(rid, []) if s0 <= s < s1),
            "run_layer": occ / a.car_m,
            "per_bay": sum(1 for bid, (br, bs) in bay_pos.items()
                           if br == rid and s0 <= bs < s1 and bay_vote.get(bid, (0,))[0]),
        })

    print()
    print(f"{'segment':14s} {'GT':>4s} {'inst':>5s} {'runL':>6s} {'bay':>5s}")
    for r in rows:
        print(f"{r['seg']:14s} {r['gt']:4d} {r['inst']:5d} "
              f"{r['run_layer']:6.1f} {r['per_bay']:5d}")

    print()
    print(f"{'prediction':12s} {'total':>7s} {'MAE':>7s} {'bias':>7s} "
          f"{'within 1':>9s}")
    g = [r["gt"] for r in rows]
    for tag, key in (("instances", "inst"), ("run layer", "run_layer"),
                     ("per-bay", "per_bay")):
        p = [r[key] for r in rows]
        err = [pi - gi for pi, gi in zip(p, g)]
        mae = sum(abs(e) for e in err) / len(err)
        bias = sum(err) / len(err)
        w1 = sum(1 for e in err if abs(e) <= 1.0)
        print(f"{tag:12s} {sum(p):7.0f} {mae:7.2f} {bias:+7.2f} "
              f"{w1:5d}/{len(err):<3d}")
    print(f"{'(truth)':12s} {sum(g):7d}")
    # ---- what SHIPS is what was scored -------------------------------------
    # The `inst` column above clusters and locates directly; `curb_runs.run_summary`
    # is what the server actually publishes, and it applies an observed-span gate
    # and a capacity clamp on top. If those silently drop cars, the bias reported
    # here is a number the product does not produce -- so assert they agree before
    # any of it is believed.
    spans_by_run = collections.defaultdict(list)
    for rid, fr in rl.items():
        for rec in fr:
            spans_by_run[rid].extend(rec["observed"])
    published = off_obs = 0
    gap_free = subtraction_free = 0
    for rid, r in runs.items():
        summ = cr.run_summary(r, car_s.get(rid, []), spans_by_run.get(rid, []))
        published += summ["cars"]
        off_obs += summ["cars_off_observed"]
        gap_free += summ["free"]
        subtraction_free += summ["free_by_subtraction"]
    placed = sum(len(v) for v in car_s.values())
    print()
    print(f"published vs scored: run_summary counts {published} car(s), "
          f"{placed} instance(s) placed on a run"
          f"{' -- MISMATCH' if published != placed else ' -- agree'}"
          f"{f'; {off_obs} on kerb no frame observed' if off_obs else ''}")
    print(f"free spaces over all runs: {gap_free} measured from gaps, "
          f"{subtraction_free} by capacity_observed - cars")

    print()
    print("MAE is cars per 18 m segment; bias>0 means over-counting. `total` is the "
          "flight-wide\ncar count, where over- and under-counts cancel -- so read MAE "
          "first.")


if __name__ == "__main__":
    main()
