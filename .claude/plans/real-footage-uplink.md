# Plan — real drone footage through the live web stack

**Goal.** Start a real DJI flight's footage and watch the parking map at :5173 fill
in live, exactly as `pnpm quickstart --fly` does for the simulator. A faithful
rehearsal of the PARKDRONE hardware path, using footage already on disk.

**Not the goal.** Real-time transport from an airborne drone (the DJI has no
user-programmable onboard compute — see CLAUDE.md §2b), and not an accuracy
result. The output of this work is a *demonstration* and a *production-shaped
scoring path*, never a number to quote.

Written 2026-08-22. Status: **not started.**

---

## Why this needs a new layer at all

The plumbing is nearly free — `POST /api/v1/ingest/frame` is already
drone-agnostic (multipart image + `{survey_area, pose, mission_id}` + per-drone
API key), and `sim_uplink.py` is 216 lines of which only three things are
sim-specific.

**The scoring is the layer.** Measured 2026-08-22 — the committed heuristic run
on one real frame with real intrinsics and a course-derived yaw:

```
frame 60:  8 bays called OCCUPIED, 0 called FREE     <- 100% false positives

core_chroma     median  33.4   threshold > 12.7      <- fires on everything
core_dark_frac  median  0.24   threshold > 0.02      <- fires on everything
core_brightness median  89.5   envelope 82-102       (this one is fine)
```

Real cobblestone is far more colourful and more textured than Webots asphalt.
`classify()`'s thresholds were hand-calibrated on `fmi_block_4st`
(`docs/occupancy_classifier.md`); a different *surface* breaks them outright, a
harsher version of the low-sun collapse already on record. **So a real-footage
demo driven by `classify()` would paint every bay red.** The fine-tuned detector
(`docs/training.md`) is not an enhancement here — it is the only thing that can
make the map mean anything.

---

## Piece 1 — pose adapter: yaw from course over ground

**Problem.** `project()` does `pose["yaw"]` with **no fallback** — verified, a real
DJI pose raises `KeyError: 'yaw'`. The nadir fallback covers `roll`, `pitch`,
`cam_pitch`, `cam_roll` but *not* yaw. The DJI SRT records no `gb_yaw`.

**Approach.** Derive heading from the track:

```python
yaw = math.atan2(y[i+1] - y[i-1], x[i+1] - x[i-1])     # ENU: CCW from east
```

Central difference over 1 Hz stills (~4.2 m apart), smoothed, and **undefined
while hovering** — carry the last good value and mark it.

**Every derived pose carries `yaw_estimated: true`.** Course over ground is not
heading: a drone crabs in wind and flies backwards on a return leg, and the
gimbal yaws independently. Expect a few degrees of error, which at 20 m off-nadir
displaces a bay by ~3.5 m — comparable to a bay's own width. Bays will light up
*approximately*, sometimes the neighbour.

**Where.** `tools/dji_srt.py` (a `course()` helper beside `rebase()`), surfaced by
`tools/dji_stills.py` behind a flag so `poses.json` keeps recording only what the
drone actually reported. The estimate is a consumer decision, same principle as
`--as-now`.

**Done when** `vision/diag/projection_selftest.py` round-trips a derived pose, and
projected bay outlines drawn on a real frame land on the visible kerb — checked by
eye, since there is no paint to register against.

---

## Piece 2 — camera intrinsics stop being module globals

**Problem.** `IMG_W`, `IMG_H`, `FOV` are module-level constants in
`score_occupancy.py`, baked to the Mavic2Pro proto (400x240, 45 deg). The real
drone is 3840x2160 at 73.7 deg H / 45.4 deg V. Testing the above required
monkeypatching them, which is not a thing production code may do.

**Approach.** Let intrinsics travel with the pose (`img_w`, `img_h`, `fov`), with
the current constants as defaults so every existing caller and every
`poses.json` on disk behaves identically. `project()`/`unproject()` read from the
pose when present.

**Note this is now the FOURTH copy of the camera model** (the `.wbt` proto,
`score_occupancy.py`, `parkdrone.py`'s `CAM_FOV`/`CAM_ASPECT`, and real-drone
values). Same standing duplication hazard as `ORIGIN` and `DS_*`;
`tools/check_consistency.py` does not cover it and should.

**Done when** both golden fixtures still score **100%** unchanged
(`fmi_block` TP=17 TN=25 FP=0 FN=0, `fmi_block_4st` TP=49 TN=62 FP=0 FN=0) and the
web tier's golden replay is still 42/42.

---

## Piece 3 — detector-backed scoring path *(the substance)*

**This is production code, not demo scaffolding**, and is the path the whole
learned-detector track is heading toward anyway.

**Approach.** An alternative to the per-bay `score_frame()`:

1. run the fine-tuned model on the frame (one inference, whole image);
2. `unproject()` each detection's box centre to the ground plane;
3. assign to the nearest bay within `ASSIGN_MAX_M` (3.0 m — about one bay width);
4. that bay is occupied; every *other* bay inside the camera footprint is free.

Step 4 is the subtle one: a detector reports cars, not vacancies, so "free" is an
inference from "visible and undetected". A bay under canopy is **not** free — it is
unobserved — and the footprint test alone cannot tell those apart. Start by
treating visible-and-undetected as free (what `detect_baseline.py` does), and log
the ambiguity rather than hiding it.

**Reuse, do not reimplement.** `vision/detect_baseline.py` already does exactly
this offline, including `ASSIGN_MAX_M` and the voting. Lift it into a shared
function both callers use, in the spirit of `vision/scoring.py` keeping
`classify()` in one place.

**Selection.** Per survey area or per drone — a stored `scoring_mode` of
`heuristic` | `detector`. The sim keeps `heuristic` so nothing on record moves.

**Cost.** ~0.9 s/frame for yolov8s at imgsz 1024 on this CPU, against the
heuristic's milliseconds. Still far above a drone's ~0.5 frames/s, so the existing
`queue.Queue` + classify-thread design holds without change. Worth measuring, not
assuming.

**Done when** the detector path, run offline over the 34 labelled val frames,
produces per-bay verdicts that match a hand check — and when switching a sim area
to `detector` mode does *not* change any committed sim number, because the sim
stays on `heuristic`.

---

## Piece 4 — `footage_uplink.py`

Sibling to `sim_uplink.py`, reading a stills directory instead of `sim/output/`.

- reads `pics/dji/stills/<flight>/poses.json` (record shape `{file, video_frame,
  t_s, x, y, alt, lat, lon, captured_at}`) and adapts it to the ingest pose;
- **paces playback** — real time from `t_s`, or `--speed N`; the sim version has no
  pacing because the flight itself provides it;
- `--as-now` so `observed_at`'s `DEFAULT now()` stays correct (decision of
  2026-08-22, CLAUDE.md web invariants);
- `frames_expected` = the number of stills, since there is no `route.json`;
- **store-and-forward with retry**, the pattern the real hardware needs: never post
  from a capture loop, always from a separate process reading disk. Mission-keyed
  idempotency (migration `0009`) makes retries safe by construction.

Then a `pnpm quickstart --footage <flight>` flag mirroring `--fly`.

**Done when** a flight replays end to end and the ops dashboard shows a real
mission with real progress.

---

## Order, and why

**3 → 2 → 1 → 4.**

Piece 3 first because it decides whether the demo shows anything, and it can be
built and tested **entirely offline** against the 494 labelled boxes before any
uplink plumbing exists. A failure there changes the plan; a failure in piece 4
just costs an afternoon.

2 before 1 because the yaw work needs correct intrinsics to be checkable at all.

4 last, and deliberately: it is the least interesting and the best understood.

---

## Risks

| risk | mitigation |
|---|---|
| Estimated yaw puts cars in the neighbouring bay | Flag every frame `yaw_estimated`; present as a demo, never a measurement. Real fix is registration against building corners — separate work. |
| "Free" is inferred from "undetected" | Canopy-occluded bays will read free. Log visible-but-undetected separately from detected-empty so the ambiguity is countable. |
| Touching `project()` breaks the sim | Both golden fixtures + the web golden replay re-run as the gate on piece 2. Defaults keep every existing pose behaving identically. |
| The detector was trained on this exact street | 130 frames, one flight. It will look better here than it deserves to. Do not let the demo become the evidence. |
| Detector inference is ~1000x the heuristic's cost | Measure. Headroom is large (~0.5 frames/s per drone) but the number should be real. |

---

## Acceptance

`pnpm quickstart --footage DJI_20260821163108_0035_D` brings up the stack, replays
the flight at real speed, and the map at :5173 fills in bay by bay as it goes,
with the ops dashboard showing a live mission. Every committed sim number is
unchanged. Nothing produced here is quoted as an accuracy figure.

---

## Open questions for the author

1. **Which survey area name** should real footage ingest under? It must not
   collide with a sim world, since `sim/worlds/<area>.ground_truth.json` is looked
   up by name (a miss is fine — `gt` becomes NULL and accuracy reports as unknown,
   which is the production case).
2. **Playback speed** — real time (136 s) is a good demo; faster is a better test.
3. Should the detector path eventually **replace** the heuristic for the sim too,
   or stay a parallel mode? TODO #5 implies replace; this plan assumes parallel.
