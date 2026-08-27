"""Score a REAL flight against hand-labelled per-bay ground truth.

    python vision/score_real.py pics/dji/stills/DJI_..._0035_D --gt vision/data/gt_dji_0035.json
    python vision/score_real.py <stills> --gt <json> --sweep      # tune ASSIGN_MAX_M honestly

The sim has `ground_truth.json` and every accuracy figure on record comes from it.
Real footage had nothing, so `/api/v1/metrics` correctly reports accuracy as `null`
and the assignment radius could not be tuned -- widening it produces more occupied
bays and nothing said whether they were the right ones. `label_bays.py` produces
the missing file; this scores against it.

**`skip` labels are excluded, not guessed.** A bay a human could not judge is out of
the confusion matrix entirely, the same treatment `score_occupancy.py` gives a
closed street: counting it either way would put a number on record that nobody
actually verified.

**`--sweep` is the point of the whole exercise.** It reports the confusion matrix at
several radii under both assignment rules, so the constant is chosen on measured
accuracy instead of on how many bays light up. Note what it cannot fix: a car parked
where the dataset maps no bay is invisible to every radius, and it shows up here as
the accuracy ceiling rather than as an error.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import bay_corrections                                           # noqa: E402
import cameras                                                   # noqa: E402
import detect_occupancy as dio                                   # noqa: E402
import score_occupancy as so                                     # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")


def frames_with_dets(stills, model_name, conf, cam, imgsz, cache_path):
    """[(pose, [(box, conf), ...]), ...] -- cached, because inference dominates."""
    if cache_path and os.path.exists(cache_path):
        data = json.load(open(cache_path, encoding="utf-8"))
        print(f"detections from cache: {cache_path} ({len(data)} frames)")
        return [(f["pose"], [(b, c) for b, c in f["dets"]]) for f in data]

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
        dio.run(stills, model_name, conf, cam, imgsz=imgsz)
    dio.bay_votes_from_dets = real
    print(buf.getvalue().rstrip())
    if cache_path:
        json.dump([{"pose": p, "dets": [[list(b), c] for b, c in d]} for p, d in grab],
                  open(cache_path, "w", encoding="utf-8"))
        print(f"cached detections -> {cache_path}")
    return grab


def assign(frames, bays, cam, radius, nearest_wins):
    """Replay the detections->bays rule at a given radius. -> {bay_id: (occ, views)}"""
    w, h, _ = so._intrinsics(cam)
    per_frame = []
    for pose, dets in frames:
        hits = []
        for box, conf in dets:
            g = so.unproject((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0,
                             pose, cam=cam)
            if g:
                hits.append((g[0], g[1], conf))
        vis = []
        for b in bays:
            ring = [so.project(x, y, pose, cam=cam) for x, y in b["ring"]]
            us = [p[0] for p in ring]
            vs = [p[1] for p in ring]
            if min(us) < 0 or max(us) >= w or min(vs) < 0 or max(vs) >= h:
                continue
            vis.append(b)
        claimed = set()
        if nearest_wins:
            for hx, hy, _c in hits:
                cand = [(math.hypot(hx - b["cx"], hy - b["cy"]), i)
                        for i, b in enumerate(vis)]
                cand = [c for c in cand if c[0] <= radius]
                if cand:
                    claimed.add(min(cand)[1])
        else:
            for i, b in enumerate(vis):
                if any(math.hypot(hx - b["cx"], hy - b["cy"]) <= radius
                       for hx, hy, _c in hits):
                    claimed.add(i)
        per_frame.append([{"bay_id": b["id"], "occupied": i in claimed}
                          for i, b in enumerate(vis)])
    return dio.vote(per_frame)


def confusion(result, gt):
    tp = tn = fp = fn = 0
    missing = 0
    for bay, label in gt.items():
        if label == "skip":
            continue
        if bay not in result:
            missing += 1
            continue
        pred = result[bay][0]
        truth = (label == "occ")
        if pred and truth:
            tp += 1
        elif pred and not truth:
            fp += 1
        elif truth:
            fn += 1
        else:
            tn += 1
    return tp, tn, fp, fn, missing


def line(tag, result, gt):
    tp, tn, fp, fn, miss = confusion(result, gt)
    n = tp + tn + fp + fn
    acc = 100.0 * (tp + tn) / n if n else float("nan")
    rec = 100.0 * tp / (tp + fn) if (tp + fn) else float("nan")
    pre = 100.0 * tp / (tp + fp) if (tp + fp) else float("nan")
    print(f"{tag:>22}  n={n:>4}  acc {acc:5.1f}%  "
          f"recall {rec:5.1f}%  precision {pre:5.1f}%   "
          f"TP={tp:<4} TN={tn:<4} FP={fp:<4} FN={fn:<4}"
          + (f"  (+{miss} unseen)" if miss else ""))
    return acc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--gt", required=True, help="gt_*.json from label_bays.py")
    ap.add_argument("--model", default=os.path.join(os.path.dirname(
        os.path.abspath(__file__)), "runs", "merged1", "weights", "best.pt"))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=1024)
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--radii", default="2,3,4,5,6,7")
    ap.add_argument("--cache", help="detections cache json (default beside --gt)")
    ap.add_argument("--corrections", help="align_bays.py sidecar; bays are moved "
                                          "before scoring (data/ is never touched)")
    a = ap.parse_args()

    cam = cameras.get(a.cam)
    gt = json.load(open(a.gt, encoding="utf-8"))
    labels = gt["bays"] if isinstance(gt, dict) and "bays" in gt else gt
    labels = {str(k): v for k, v in labels.items()}
    n_occ = sum(1 for v in labels.values() if v == "occ")
    n_free = sum(1 for v in labels.values() if v == "free")
    n_skip = sum(1 for v in labels.values() if v == "skip")
    print(f"ground truth: {len(labels)} bays labelled -- "
          f"{n_occ} occupied, {n_free} free, {n_skip} skipped (excluded)")
    if not n_occ:
        print("  NOTE: no bay is labelled occupied, so recall is undefined and "
              "accuracy is the base rate. That is a finding about the bay data, "
              "not a model result.")

    cache = a.cache or os.path.splitext(a.gt)[0] + ".dets.json"
    bays = so.load_all_bays()
    corr = bay_corrections.load(a.corrections)
    if corr:
        print(f"corrections: {a.corrections}")
        print(bay_corrections.summary(corr))
        bays = bay_corrections.apply_to(bays, corr)
    frames = frames_with_dets(a.stills, a.model, a.conf, cam, a.imgsz, cache)

    print()
    if not a.sweep:
        line(f"radius {dio.ASSIGN_MAX_M:.0f} m (committed)",
             assign(frames, bays, cam, dio.ASSIGN_MAX_M, False), labels)
        return

    best = (None, -1)
    for radius in [float(r) for r in a.radii.split(",")]:
        for nw in (False, True):
            tag = f"{radius:.0f} m {'nearest-wins' if nw else 'as-is'}"
            acc = line(tag, assign(frames, bays, cam, radius, nw), labels)
            if acc == acc and acc > best[1]:
                best = (tag, acc)
    print(f"\nbest: {best[0]} at {best[1]:.1f}%")
    print("Before adopting it, check the confusion matrix and not just the accuracy -- "
          "on a mostly-free set the base rate is already high, so a radius that never "
          "fires can score well while finding nothing.")


if __name__ == "__main__":
    main()
