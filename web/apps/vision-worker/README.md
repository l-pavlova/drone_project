# PARKDRONE vision worker (Python)

Consumes frame jobs off Redis, classifies per-bay occupancy, writes state to
Postgres, and publishes occupancy deltas back to the API for WebSocket fan-out.

It **reuses the calibrated classifier verbatim** from the project's offline vision
stage (`vision/score_occupancy.py`): `project`, `bay_features`, `classify`, the
`IMG_W/IMG_H/FOV/MIN_VIS` intrinsics, and the ENU projection. Only the *driving*
loop is new — one frame at a time instead of a whole `poses.json` batch, and
decoupled from `ground_truth.json` (production has no ground truth).

## Streaming model vs. the batch script

- `vision_core.score_frame(img, bays, pose)` → single-view classification of every
  bay whose lengthwise core is ≥ `MIN_VIS` visible in this frame. Each becomes one
  **observation** row (append-only history).
- `bay_state` (current occupancy) is a **majority vote over that bay's accumulated
  observations** — identical semantics to the batch script's per-bay vote, so once
  all frames of a patrol are processed the state equals the offline result.
- A bay whose vote flips (or first becomes known) emits a `bay_delta`.

## Run

```bash
python -m venv .venv && . .venv/Scripts/activate     # or your env
pip install -r requirements.txt

# live worker (needs Redis + Postgres + object store)
python -m parkdrone_vision.worker

# golden test: replay a sim world through the streaming scorer and compare
# bay_state to the committed occupancy_results.json
python -m parkdrone_vision.replay fmi_block
```
