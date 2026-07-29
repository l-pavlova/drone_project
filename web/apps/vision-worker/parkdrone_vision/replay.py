"""Golden test: replay a sim world's frames through the STREAMING scorer and
assert the resulting bay_state matches the committed occupancy_results.json
(and, in eval mode, the accuracy vs. ground_truth.json).

    python -m parkdrone_vision.replay [world]     # default fmi_block

This proves the worker's per-frame extraction preserves the offline classifier:
the same frames + the same thresholds must yield the same per-bay predictions.
Runs against a live Postgres (it truncates observation/bay_state first).
"""
import json
import os
import sys

import numpy as np
from PIL import Image

from .config import SIM_OUTPUT_ROOT
from .db import vision_db as db
from .processing.pipeline import process_frame


def main():
    world = sys.argv[1] if len(sys.argv) > 1 else "fmi_block"
    out_dir = os.path.join(SIM_OUTPUT_ROOT, world)
    poses = json.load(open(os.path.join(out_dir, "poses.json"), encoding="utf-8"))
    expected = json.load(
        open(os.path.join(out_dir, "occupancy_results.json"), encoding="utf-8")
    )["results"]

    conn = db.connect()
    with conn.cursor() as cur:
        cur.execute("DELETE FROM observation WHERE world = %s", (world,))
        cur.execute("DELETE FROM bay_state")
    conn.commit()

    bays = db.load_bays_enu(conn)
    print(f"replay {world}: {len(poses)} frames, {len(bays)} bays")

    for pose in poses:
        i = pose["i"]
        path = os.path.join(out_dir, f"frame_{i:03d}.png")
        img = np.array(Image.open(path).convert("RGB"))
        res = process_frame(conn, world, i, img, pose, bays)

    # ---- compare final bay_state to the offline result (intersection = the
    #      bays the offline script scored; the worker also scores non-GT bays).
    state = {}
    with conn.cursor() as cur:
        cur.execute("SELECT bay_id, occupied FROM bay_state")
        for bay_id, occ in cur.fetchall():
            state[bay_id] = occ

    match = mismatch = missing = 0
    mismatches = []
    for bay_id, r in expected.items():
        pred = r["pred"]
        got = state.get(bay_id)
        if got is None:
            missing += 1
        elif got == pred:
            match += 1
        else:
            mismatch += 1
            mismatches.append((bay_id, pred, got))

    n = len(expected)
    print(f"\nvs occupancy_results.json ({n} scored bays):")
    print(f"  match={match} mismatch={mismatch} missing_in_state={missing}")
    if mismatches:
        print("  mismatched bays (bay_id: offline -> worker):")
        for bay_id, pred, got in mismatches[:20]:
            print(f"    {bay_id}: {pred} -> {got}")

    # ---- eval-mode accuracy vs ground truth (offline `gt` field)
    tp = tn = fp = fn = 0
    for bay_id, r in expected.items():
        got = state.get(bay_id)
        if got is None:
            continue
        gt = r["gt"]
        tp += got and gt
        tn += (not got) and (not gt)
        fp += got and (not gt)
        fn += (not got) and gt
    total = tp + tn + fp + fn
    if total:
        print(f"\naccuracy vs ground truth: {100*(tp+tn)/total:.1f}% "
              f"(TP={tp} TN={tn} FP={fp} FN={fn})")

    conn.close()
    # non-zero exit if the streaming worker diverged from the offline classifier
    sys.exit(1 if mismatch else 0)


if __name__ == "__main__":
    main()
