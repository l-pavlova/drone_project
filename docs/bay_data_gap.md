# The bay-data gap — why detected cars do not become occupied bays

*Investigation of 2026-08-25/26. Read this first if you are picking the vision track up cold.*
The mechanism and constants live in `CLAUDE.md` §2f–2g and `docs/training.md`; this file is the
argument: what was measured, what it proves, what it does **not** prove, and the decision waiting.

---

## The symptom

Two new DJI flights (0074, 0075 — 2026-08-25, midday, same block, ~30 m nadir) went through the
whole pipeline. The detector found **273 cars**. The occupancy scorer reported **zero occupied
bays** — 0 of 49 on 0074, 0 of 9 on 0075.

Not a detector failure. The pipeline has two independent stages and only the second one broke:

1. **find the cars in the frame** — worked. 4.3 detections/frame on 0075's parked-car street
   against 3.5 on the older flight 0035, at the same confidence distribution
   (p10 0.33 / p50 0.63 / p90 0.80 vs 0.33 / 0.68 / 0.83). Checked frame by frame in the
   overlays: every parked car carries a box. **The detector also generalises across the light** —
   these flights are midday where 0035 was 16:31, which was the open question about `merged1`.
2. **decide which mapped bay each car is in** — produced nothing. A detection counts for a bay only
   if it lands within `ASSIGN_MAX_M` (3 m) of that bay's centroid. On 0075 **not one of 91
   detections** was within 3 m of any bay; the nearest decile was already 5.9 m out.

---

## What is proven, and by what

**The vote is working.** Flight 0035 is the positive control: there an occupied bay reads
**11/13, 10/12, 7/7, 6/6** views — a car in a mapped bay is seen in nearly every look at it. On
0074 the best bay in the whole flight manages **6 of 21**, then 2/5, 2/7, 2/22; on 0075 all nine
bays have **zero** occupied views. That is the signature of cars parked *beside* the geometry, not
of a majority rule misfiring.

**The projection is sound.** Three independent checks:
- **Self-consistency.** The same car detected in ≥3 frames unprojects to the same ground point —
  median scatter **0.94 m** (0075) and **1.08 m** (0074), p90 ~2.5 m. A 9 m error cannot come from
  a 1 m pose.
- **An independent map.** OSM building footprints projected into the frames land on their real
  buildings (`vision/diag/ref_align.py`, flight 0074 frame 57 — the footprint line sits on the
  building's edge, allowing for the roof lean of a 5-storey block at 30 m off-nadir).
- **Yaw is not the cause.** Correcting yaw alone moves 0075's median detection-to-bay distance only
  9.01 → 8.19 m, and 0074's 4.81 → 4.68.

**Flat ground control separates the two flights.** `vision/diag/paint_ground_control.py` unprojects
painted road markings (paint lies **on** the ground plane `unproject()` assumes, so unlike a roof it
has no parallax), rasters the bay outlines on the same grid and correlates. `--self-test` recovers
injected shifts exactly. Both flights ran through the **identical** mask, so the extractor's own
imperfections cancel and the comparison carries the argument:

| flight | overlap at (0,0) | peak | headroom | peak/median | occupancy |
|---|---|---|---|---|---|
| **0035** (control) | 1204 | 1337 @ 4.03 m | **9.9%** | 1.55× — flat | 11 bays occupied |
| **0075** | 19 | 133 @ 4.51 m | **85.7%** | 3.98× — sharp | 0 bays occupied |

Flight 0035's bays **already sit on its paint**: the best available shift buys 9.9% over doing
nothing, on a peak barely above the search median. That is what "no offset" looks like, and it is
the control the earlier argument lacked — a systematic error in our pose chain would have shown up
here too. Flight 0075's bays do **not** sit on its paint.

---

## The conclusion, stated at the strength the evidence supports

**On 0075 it is both things at once.** There is a real ~4.5 m geometry disagreement on that street.
But 4.5 m does not explain the occupancy result, because its detections sit a **median 8.95 m** from
the nearest bay — correcting it would still leave most cars unassigned. So: a few metres of bad
geometry, *and* cars genuinely parked where the dataset maps no bay. Visible directly in the
frames — a row of painted, occupied bays along the kerb that Sofiaplan does not contain.

**And the gap is systemic, not a quirk of the new flights.** This only became visible once the
pipeline stopped discarding unplaced cars (below):

| flight | detections | matched no bay | median distance | within 6 m |
|---|---|---|---|---|
| 0075 | 100 | **100 (100%)** | 9.02 m | 12 |
| 0035 — the flight that *works* | 599 | **473 (79%)** | 5.21 m | 298 |

Even on the street whose bays sit on its paint, **four cars in five are attributed to nothing**.
0035 only looks healthy because enough cars land in mapped bays to yield 10 occupied.

### What is NOT established
- **Whose fault 0075's 4.5 m is.** A constant per-flight GPS bias and a mis-drawn street are
  indistinguishable from one flight on one street. 0035 being flat argues the *method* is fine, but
  does not exclude a per-flight offset on 0075 specifically.
- **The metre values from the paint tool.** The extractor finds strong markings (it picks the zebra
  crossing cleanly) but **misses worn bay lines** under dappled midday canopy. The 0035-vs-0075
  *contrast* is load-bearing; the absolute offsets are indicative.
- **Any real-world accuracy number.** There is still no per-bay ground truth for any real flight.

---

## Fixed along the way

**A colour-channel bug on the live path.** ultralytics reads a numpy array as **BGR**; three call
sites built theirs from PIL RGB, so the detector ran with red and blue swapped —
`detect_occupancy.py`, `render_demo.py`, and the server's `vision/scoring.py`. Cost **473 → 599
detections (+27%)** on flight 0035. Found only because two code paths disagreed on identical frames
(108 vs 173); they now agree to the exact count. Training labels and every reported
recall/precision figure were unaffected (`prelabel.py`/`detect_real.py` use cv2 and were always
right).

**The pipeline no longer throws away the cars it cannot place** (migration `0013`).
`bay_votes_from_dets` loops over *bays*, so a detection matching none vanished with no trace — the
system could report "9 bays, all free" while silently dropping 100 detections to get there. Zero
occupied bays is a quiet signal; 100 unplaced cars is a loud one, and the loud one was the one being
discarded. Now recorded as `frame_job.unassigned_dets` / `unassigned_near` and an `unassigned` block
on `/api/v1/metrics`, split at 2× the assignment radius because near and far call for **opposite**
fixes: near means the geometry is a few metres out, far means the street's parking is unmapped.

**Nadir screening inside the occupancy pass.** 22% of flight 0074 is oblique (the gimbal tilts to
the horizon). An oblique frame does not fail to project — it silently places its cars tens of metres
away, with full confidence.

---

## The decision waiting for you

Three fixes at very different sizes. **1 is done.** 2 is mechanical. **3 is a product decision and
it is yours**, because it changes what PARKDRONE claims to measure.

**2. Per-street geometry correction (medium).** `paint_ground_control.py` already measures the
offset; applying it per street is the honest form, with flight 0035 required to come out near zero
as the check that we are correcting Sofiaplan's geometry rather than injecting our own error.
**Blocked on the paint extractor** — it must get from "finds the zebra crossing" to "finds worn bay
lines" before its metre values can be trusted. Note this only buys ~4.5 m on 0075, so it is not the
main fix; it is the one that makes the numbers defensible.

**3. Get bays that match the street (large).** The real cause. Three ways:

| option | what it gives | cost | what you give up |
|---|---|---|---|
| **Extend the map from our own imagery** — detect painted bays, add them | the drone surveys parking **inventory**, not just occupancy; strongest thesis contribution | large; the paint work is half-built | scope creep late in the project |
| **Score against detected parking rows** — infer where parking is from where cars consistently sit | cheap, works everywhere | medium | the tie to official bay identity, which the whole web tier is built around |
| **Scope the product to streets Sofiaplan maps correctly** | completely honest, no new work | small | must say out loud that the block is only partially coverable |

### Suggested first moves tomorrow
1. **Hand-label per-bay ground truth on flight 0035's street** — the paint check is flat there, so a
   number measured on it stands up. This is the one missing piece for a reportable real-world
   result, and it no longer risks measuring the detector against unverified geometry.
2. Then pick among the three options in 3. The `unassigned` metric is now the evidence base for
   that choice: it says how much parking the dataset is missing, per street, per flight.

## Where things are

- **Datasets pre-labelled and waiting on hand correction:** `vision/data/dji_0074` (36 frames /
  60 boxes), `vision/data/dji_0075` (23 / 100). Both **train-only** — forced by geometry, since
  0074 hovers and doubles back so no split point reaches the 25 m footprint. Benchmark against
  `dji_merged`'s hand-drawn val set (checked at 68 m and 238 m clear of it). Labels are pixel boxes,
  so none of the geometry argument above affects them.
- **New diagnostics:** `vision/diag/ref_align.py` (OSM buildings/roads/greens as an independent
  control), `vision/diag/paint_ground_control.py` (flat ground control, with a self-test).
- **Gates all green** as of 2026-08-26: golden replay 42/42 at 100%, `detect_baseline fmi_block`
  unchanged (TP=0 FP=0 FN=30 / 58.8%), 40 consistency checks, projection self-test 0.000000 mm.
- **Nothing is committed.** Migration `0013` **is** applied to the live dev database. The server on
  :4000 is still running pre-change code from an earlier session — restart it to get the new
  metrics block.
