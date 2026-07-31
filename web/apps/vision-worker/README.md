# PARKDRONE vision worker (Python)

A single FastAPI process: the web edge (ingest, reads, WebSocket push, dev toggle) and the CV
classifier, in one process. Frame jobs move through an in-process `queue.Queue` drained by
dedicated classify threads — no Redis, no separate worker.

It **reuses the calibrated classifier verbatim** from the project's offline vision
stage (`vision/score_occupancy.py`): `project`, `bay_features`, `classify`, the
`IMG_W/IMG_H/FOV/MIN_VIS` intrinsics, and the ENU projection. Only the *driving*
loop is new — one frame at a time instead of a whole `poses.json` batch, and
decoupled from `ground_truth.json` (production has no ground truth).

## Layout

- `api/` — FastAPI routes (`app.py`), drone API-key auth (`auth.py`), the WebSocket fan-out hub (`hub.py`)
- `processing/` — the in-process job queue + classify thread pool (`jobs.py`), per-frame scoring pipeline (`pipeline.py`)
- `db/` — connection pool (`pool.py`), vision-side Postgres access: bay geometry/observations/bay_state (`vision_db.py`), web-edge SQL: reads/ingest/mission/auth (`web_db.py`)
- `vision/scoring.py` — bridge to the offline classifier
- root — `config.py`, `s3.py` (object store), and the CLI entry points below

## Streaming model vs. the batch script

- `vision.scoring.score_frame(img, bays, pose)` → single-view classification of every
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

# the server (needs Postgres + object store; see top-level CLAUDE.md for full dev setup)
python -m parkdrone_vision.server

# golden test: replay a sim survey area through the streaming scorer and compare
# bay_state to the committed occupancy_results.json
python -m parkdrone_vision.replay fmi_block
```
