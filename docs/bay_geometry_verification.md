# Verifying the Sofiaplan bay coordinates against outside references

*Investigation of 2026-08-26. Companion to `docs/bay_data_gap.md`, which asked "why do detected
cars not become occupied bays?" and answered it with drone-side evidence only. This file answers
the prior question — **are the bay coordinates where they should be?** — using two references that
are not our pose chain: satellite imagery and OSM.*

New tools: `vision/diag/sat_align.py` (satellite overlay), `vision/diag/bay_audit.py` (geometric
audit against OSM). Both are diagnostics; neither is on the runtime path.

---

## First, what is actually Sofiaplan's

This matters because the two halves have different owners and different failure modes.

| | source | what it is |
|---|---|---|
| **the point** | Sofiaplan `spaces_25.geojson` (31,705 points; 1,698 in the block) | *their* datum — the thing being verified |
| **the rectangle** | `tools/make_bays.py` | **ours** — a 5.4x2.2 m box drawn on their point, oriented from the nearest OSM road centerline |

So "the bay is in the wrong place" splits into *their point is misplaced* and *our box is drawn
wrong*, and the evidence below separates them. One finding lands on each side.

---

## 1. Satellite (`vision/diag/sat_align.py`)

Basemap is **Esri World Imagery** XYZ tiles. Sofia is served to **z19 = 0.22 m/px**; z20 returns a
"map data not yet available" placeholder, so a 2.2 m bay is ~10 px across. That makes this a
**placement** check ("tarmac or garden?"), not a sub-metre one — sub-metre is what
`paint_ground_control.py` is for. Google's tiles are not used: their terms do not permit scraping
outside the Maps API, and Esri answers the same question.

**The basemap's own georeference is checked, not assumed.** OSM building footprints are drawn on
every render, and they sit on their roofs across the whole block (allowing for the lean of a
5-storey block in off-nadir satellite imagery). If those were displaced, nothing else in this file
would be admissible.

Tiles are cached under `pics/sat_tiles/` (gitignored); the whole 1 km block is 380 tiles.

```bash
python vision/diag/sat_align.py --out vision/diag/out/sat_block.png       # whole block
python vision/diag/sat_align.py --near "-291,-323" --half 55 \
       --bays-alt <other>/block_bays.geojson                              # compare two geometries
python vision/diag/sat_align.py --near "120,60" --half 60 \
       --dets vision/runs/dji_0035_occupancy.json                         # + real detections
```

## 2. OSM geometry (`vision/diag/bay_audit.py`)

Uses no photograph at all — it asks whether each bay is somewhere a parking bay can physically be,
against OSM road centerlines and building footprints. All 1,698 bays:

```
distance to nearest road centerline: median 4.1 m  p10 2.1  p90 6.7  max 42.2
   0-5 m  1076   5-10m  538   10-15m  20   15-20m  6   20-25m  6   25-30m  9   30-35m  23   35+m  20
centroid inside an OSM building :    20  (1.2%)
further than 15 m from any road :    64  (3.8%)
bearing off its street >25 deg  :   192  (11.3%)
```

**The positions are good.** 95% of bays sit in a tight kerbside band 2–7 m from a street
centerline, which is exactly (carriageway half-width + bay depth). This is the strongest single
statement available about the Sofiaplan points, and it is a positive one.

**The two flags that are not errors.** The 64 far-from-road bays are almost entirely two rows whose
own names say they are off-street — `междублоково при ул. Златовръх` (40 bays, "between the blocks
at…") and `паркинг при ул. Якубица` (12, "car park at…"). The audit is right that they are not
kerbside and wrong to call them suspect; they are correctly-mapped courtyard parking. The 20 bays
under buildings are the ones already on record in `CLAUDE.md` from the 1 km survey.

---

## Finding 1 — `data/block_bays.geojson` is STALE, and 261 bays (15.4%) are rotated wrong

This is **ours**, not Sofiaplan's.

Regenerating the bays from the committed inputs (`block_spaces.geojson` + `block_roads.geojson`)
with today's `tools/make_bays.py` reproduces **every centre to the last digit** and changes the
**orientation of 261 bays by more than 5 degrees**:

```
bays whose CENTRE moved:                   0
bays whose orientation CHANGED >5 deg:   261 (15.4%)
    35/111  ул. Кричим          28/41   ул. Вишнева        13/13  ул. Проф. Хараламби Тачев
    33/195  бул. Джеймс Баучер  24/43   ул. Плачковица     11/71  ул. Бигла
    32/83   ул. Йосиф Петров    19/83   ул. Св. Теодосий Търновски
```

Whole streets are affected, not scattered bays. On ул. Плачковица the committed rectangles carry
bearing **-1.6°** while the nearest centerline — a segment named Плачковица, 3.8 m away — runs at
**89.5°**: the boxes were oriented from a road that is 22 m away on a different street. Visible
directly in `vision/diag/out/sat_plachkovitsa.png`, where the committed boxes lie *across* the
carriageway and the regenerated ones lie *along* it, on identical centres.

Cause: `block_spaces.geojson`, `block_roads.geojson` and `block_bays.geojson` all carry the same
2026-07-04 12:13 mtime, so the bays were built against a roads file that was overwritten in the
same minute — most likely `get_roads.py` and `make_bays.py` run in the wrong order once. It has
been baked in ever since.

**What it does and does not affect** — measured, not argued:

* **Real-flight occupancy: nothing.** `bay_votes_from_dets` assigns a detection to a bay by
  distance to its **centroid**, and no centroid moves. Re-running the assignment for all three DJI
  flights against committed and regenerated geometry gives byte-identical counts.
* **The Webots worlds and every sim accuracy number: yes.** `generate_world.py` paints the bay
  *rectangle*, and `classify()` crops that rectangle. A 90°-rotated pad is a different piece of
  ground. Regenerating the bays therefore invalidates `ground_truth.json` and every score on
  record, so it is **not** a change to make casually — see the TODO note below.
* **The map: yes, cosmetically.** `web-user` draws these polygons.

## Finding 2 — the assignment radius is doing a lot of the rejecting

`ASSIGN_MAX_M` is 3.0 m from a bay **centroid**, while bays are 5.4 m long, so adjacent centroids
are ~5.4 m apart and a car parked on the join is already 2.7 m from both. Of the cars the rule
rejected:

| flight | rejected | inside a bay polygon | within 3 m | within 5 m |
|---|---|---|---|---|
| 0035 | 473 | 22 (4.7%) | 49 (10.4%) | **223 (47.1%)** |
| 0074 | 153 | 12 (7.8%) | 14 (9.2%) | **68 (44.4%)** |
| 0075 | 100 | 0 (0.0%) | 0 (0.0%) | 2 (2.0%) |

Widening to 5 m would place roughly **45%** of the rejected cars on the two flights over mapped
streets. It is not a free win — 5 m exceeds half a bay length, so one car becomes eligible for
several bays and the rule needs a nearest-wins tie-break it does not currently have.

Swept end to end (detector run once per flight, detections cached, the assignment rule replayed —
the 3 m/as-is row reproduces the numbers on record, which is the harness's self-test):

| flight | bays | 3 m (committed) | 4 m | 5 m | 6 m |
|---|---|---|---|---|---|
| 0035 | 88 | **10** | 19 | **24** | 33 |
| 0074 | 49 | **0** | 2 | **4** | 4 |
| 0075 | 9 | **0** | 0 | **0** | 1 |

(nearest-wins; the naive widening reads 21/30/40, 2/5/8, 0/0/1 — higher because one car claims
several bays, which is why it is an upper bound and not a result.)

**More occupied bays is not the same as more correct ones**, and that is the whole difficulty:
widening the radius trades false negatives for false positives, and with **no per-bay ground truth
on any real flight** there is nothing to say which it bought. So this is a measured lever, not a
recommendation — it cannot be tuned until step 2 below exists.

And it changes almost nothing on **0075**, which is still 0 at 5 m and 1 at 6 m. That street's cars
are not near-misses.

## Finding 2a — CORRECTION: flight 0035's bays do NOT sit on its paint

*This overturns a claim made earlier in this file and in `bay_data_gap.md`. It was found by
looking at the crops in the labelling page, which is exactly what those crops are for.*

The paint control reported 0035 as the positive control: overlap 1204 at (0,0), best shift 4.03 m
buying only **9.9%**, peak/median **1.55x** — "flat". That was read as *the bays already sit on the
paint*. **A flat correlation surface does not mean that.** It means the paint mask carried too
little signal to localise anything, so the search had nothing to find: uninformative, not
confirmatory. The doc's own caveat said the extractor "misses worn bay lines under dappled midday
canopy" and that only the 0035-vs-0075 *contrast* was load-bearing — the confirmatory reading went
further than that caveat allows.

Direct inspection settles it. On бул. Джеймс Баучер (61 of the flight's 88 bays), the rows land on
the **tram tracks** and on the pavement strip beside the building, while every parked car sits on
the cobbles between them — bays 17697, 17687, 17688 in `vision/diag/out/ra0035/align_frame_0030.jpg`
are unambiguous. The offset is roughly a car's width.

**What survives, and what does not:**

* **Survives:** the pose chain is sound (Finding 2b — buildings land on their roofs), the Sofiaplan
  points are kerbside as a population (§2), the cars are kerbside too (Finding 3), and the
  0035-vs-0075 contrast in the paint table.
* **Does not:** any statement that a particular street's bays are *verified* correct. Nothing in
  this project had verified that until the frames were looked at bay by bay.
* **Consequence:** labelling occupancy against the uncorrected rectangles would have measured
  almost nothing — most bays would read FREE because the rectangle is on a tram rail. The geometry
  has to be corrected first, which is what `vision/align_bays.py` is for (§Workflow).

The general lesson is the one already in the session record: **reach for the control before the
conclusion, and make sure the control can actually fail.** A correlation peak that is flat because
there is no signal looks identical to one that is flat because the answer is zero.

## Finding 2b — it is NOT the drone's position (tested 2026-08-26)

The natural reading of a 10 m gap is that the drone is 10 m out and the cars really are along the
street where the bays are drawn. It is the right hypothesis and it is wrong, by three independent
tests:

* **The building control fires in the same frame.** `vision/diag/ref_align.py` on flight 0075
  (`out/ref0075/ref_frame_0016.jpg`) draws the OSM building footprint onto the roof it belongs to —
  it lands. In that same frame the bays sit on the garden strip behind the fence while the three
  parked cars are on the asphalt. A 10 m position error cannot displace the bays and leave the
  building alone; both are projected by the same pose through the same `project()`.
* **Two flights three minutes apart want different corrections.** The median car→nearest-bay vector
  is (−1.18, +1.81) m on 0035, (+0.45, −2.68) m on 0074 and (−5.35, −4.80) m on 0075. 0074 and 0075
  were flown minutes apart with the same aircraft and the same GNSS state; a position bias would be
  essentially identical between them, and these differ in both size and direction.
* **~~The paint control already said so.~~** RETRACTED 2026-08-29 — this bullet cited the 9.9%
  headroom that **Finding 2a above overturns**, and it should not have survived that correction. A
  flat correlation surface was uninformative, not confirmatory. Two replacements, both measured:
* **The road centerlines land on the roads, in flight 0035's own frames.** `ref_align.py` on this
  flight (`out/ref0035/`, run 2026-08-29) draws the OSM centerline down the street in
  `ref_frame_0000.jpg` and `ref_frame_0025.jpg` while the bays sit on the tram rails and in the
  bushes and the cars are on the cobbles between them. A centerline is on the ground plane, comes
  from a source unrelated to Sofiaplan, and rides the same pose — a 6 m position error would carry
  it along. **Buildings are the weaker control at the frame edge** (footprint on the ground, roof at
  10–15 m, so at 30 m altitude it is displaced outward by parallax); the centerline has no such
  term, which makes it the one to reach for. In `ref_frame_0100.jpg` two bays contain exactly one
  car each — **the offset is not constant across the flight**, which no single pose error produces.
* **The best global shift buys ~9 points and sits at an implausible 4 m.** Re-measured 2026-08-29
  over **all 599 detections** (grid +/-8 m at 0.5 m): no shift **median 4.47 m, 175/599 (29%)
  within 3 m**; best **(-4.0, +0.5) m** -> **3.60 m, 230/599 (38%)**. That reproduces the original
  115-detection result on 5x the sample. ⚠ **Search over every detection, never over the unassigned
  alone** — that set is *defined* as the detections that already missed a bay, so its baseline is
  artificially low and the optimum is dragged outward: the same search over the 473 unassigned
  reports a spurious **10% -> 37% at 6.5 m**, which reads like a real bias and is an artefact of the
  sample.

What *is* true in the hypothesis: the cars are indeed parked sensibly along the street. Both
populations sit a normal kerbside distance from a centerline (Finding 3). They are simply not in the
same places as the mapped rectangles.

## Finding 3 — the DJI cars are kerbside; they just are not in mapped bays

Running the *same* distance-to-centerline test on the detected cars, so bays and cars are measured
against one reference:

| population | median distance to nearest road centerline |
|---|---|
| all 1,698 bays | **4.1 m** (p90 6.7) |
| unplaced cars, flight 0035 | 5.6 m (p10 0.8, p90 12.1) |
| unplaced cars, flight 0074 | 3.1 m (p10 1.3, p90 10.2) |
| unplaced cars, flight 0075 | 4.3 m (p10 3.4, p90 7.6) |

The cars sit in the same band as the bays. They are not being flung into gardens by a broken pose,
and they are not a metre or two off a bay they should have matched — on 0075 they are in a
**different row**. This is the same conclusion `bay_data_gap.md` reached from the paint control,
now reproduced from a reference that does not touch our pose chain at all.

The satellite renders show it plainly: `sat_0035.png` has bays on both kerbs with detections
following them plus a **third row** of parked cars the dataset does not map; `sat_0075.png` has the
detections in a continuous line offset ~10 m from the mapped bays.

---

## What this changes about the conclusion in `bay_data_gap.md`

It **strengthens** it and narrows what is left open.

* Still standing: the projection is sound, the vote works, and the dominant cause of unplaced cars
  is parking that Sofiaplan does not map. Two independent references now agree.
* **Newly settled:** the Sofiaplan *points* are, as a population, correctly placed. 95% kerbside at
  a plausible offset is not what a systematically wrong dataset looks like. The open question in
  `bay_data_gap.md` — "whose fault is 0075's 4.5 m" — should not be read as doubt about the dataset
  as a whole.
* **Newly found:** a defect on our side of the line, in `block_bays.geojson`'s orientations. It
  does not explain the occupancy gap (Finding 1 measures that directly), but it means every sim
  accuracy number was computed against 15% wrongly-oriented pads.
* Still open, unchanged: no per-bay ground truth exists for any real flight, so there is still no
  real-world accuracy number.

## First real hand-drawn geometry — flight 0035, 2026-08-27

22 frames annotated in makesense.ai (YOLO export, rect tool), imported with
`vision/import_annotations.py`:

```
157 boxes across 22 frames -> 126 distinct bays after merging within 2 m
 56 matched a mapped Sofiaplan bay   -- moved median 4.03 m, max 5.94 m
 70 matched NO mapped bay            -- nearest mapped bay 6.1 to 33.6 m away
```

**Two results, and the second is the bigger one.**

* Where Sofiaplan does map a bay on this street, its geometry is out by a **median
  4.03 m** — a full car's width, and consistent with the frames showing the rows on
  the tram tracks and the pavement rather than the parking strips.
* **56% of the parking bays actually visible here are not in the dataset at all**
  (70 of 126). This is the coverage gap measured directly for the first time,
  rather than inferred from unattributed detections, and it agrees with the
  detection-side figure (four cars in five attributed to nothing on this flight).
  They are written to `bay_outlines_new.geojson`.

**Caveats, stated because they bound what the next number means:**

* **Rect, not polygon.** The YOLO export is axis-aligned boxes. Measured on this
  flight the centre is unaffected and the area inflates a median 1.09x (max 1.58x,
  short side up to +1.08 m). Position is trustworthy; the outlines are not tight.
* **Drawn bays are car-sized** — median **3.96 x 2.02 m** against Sofiaplan's
  5.4 x 2.2. 61% have a detected car within 2 m and **39% do not**, and the frame
  overlays show empty bays drawn on bare cobbles, so this is not simply tracing
  cars. But the size suggests the boxes hug vehicles where vehicles are present,
  which biases a bay slightly toward whatever is in it. Redrawing as polygons on
  the lane markings would remove the doubt.
* The 70 unmapped bays are **not scoreable yet**: they have no id in
  `block_bays.geojson`, so `label_bays.py`/`score_real.py` see only the 56.

## Can Sofiaplan's data just be CORRECTED? Measured 2026-08-27 — no.

The obvious hope is that a ~4 m offset is a datum or projection slip, in which case one
transform fixes all 31,705 spaces and nobody draws anything again. The 56 hand-verified bays
are enough to test it (`vision/diag/offset_structure.py`):

```
correction vectors: magnitude median 4.03 m (p10 2.11, p90 5.37)
                    directional concentration R = 0.484   <- NOT one direction

                        uncorrected   best TRANSLATION   best SIMILARITY
all 56 bays               4.03 m         3.07 m            2.87 m  (rot -0.10 deg, scale 0.991)
бул. Джеймс Баучер (39)   4.13 m         2.57 m            2.84 m
ул. Димитър Димов (9)     3.69 m         2.63 m            2.23 m
ул. Света гора (8)        3.63 m         3.21 m            1.67 m
```

**Three ways of reading it, all pointing the same way:**

* **The vectors do not share a direction.** R = 0.484, where 1.0 is one direction and 0 is
  uniform. A datum shift would give R near 1.0. Annotation noise cannot push a genuine global
  shift from ~1.0 down to 0.48.
* **A global translation buys 24%** (4.03 -> 3.07 m) and a full similarity buys 29%, with
  rotation -0.10 deg and scale 0.991 — i.e. essentially no rotation or scale signature. If this
  were a projection problem the similarity would fit far better than the translation. It does not.
* **Even PER STREET the residual is ~2.5 m**, which is larger than a bay is wide (2.2 m). A
  corrected bay would still not sit on the right piece of ground.

`georef_201` could not be tested: all 56 bays in this flight's footprint carry the same year
(2017), so a per-batch offset remains untested rather than excluded. A flight over bays with a
different georeferencing year would test it.

**Caveat on the ground truth itself:** it is axis-aligned boxes drawn around bays, carrying
perhaps a metre of its own noise, so the *residuals* above are upper bounds on the data error.
The *directional spread* is the robust part, and it is what carries the conclusion.

**So there is no transform to apply.** The geometry is individually wrong, bay by bay, and the
only fix is the one already under way: draw it. That also settles the choice in
`docs/bay_data_gap.md` — "per-street geometry correction" is not worth building, because it
leaves more error than a bay is wide.

## Suggested order of work

0a. **Correct the geometry by DRAWING it** (preferred) —
   `python vision/pick_frames.py <stills> --out vision/data/annot_0035` picks a near-minimal covering
   set (22 frames for all 88 bays, greedy set cover), you outline the real bays as polygons in
   makesense.ai, and `python vision/import_annotations.py vision/data/annot_0035 --annot <export>
   --new-bays` unprojects every vertex to ENU and writes replacement outlines. This beats nudging
   published rectangles because a drawn bay carries its own **size and orientation**, so it fixes rows
   whose *shape* is wrong, not just their position — and the drawings that match no mapped bay are
   kept as their own GeoJSON, which on this block is the most valuable output of the whole exercise.
   Rules and gates in `vision/data/annot_0035/README.md`: draw the lane not the cars, one frame per
   bay, and `--self-test` round-trips 2,696 vertices at **0.000000 mm** before anything is believed.

0a-alt. **Or nudge the published rectangles** — `python vision/align_bays.py <stills> --out vision/data/align_dji_0035.html`.
   The unit is a **ROW — one side of one street** (four of them cover all 88 bays), split by which
   side of the OSM centerline a bay falls on, with the road matched **by name** so a junction's cross
   street cannot put the two kerbs of a N–S street on the "N" and "S" sides. Per *street* does not
   work and the frames say why: on бул. Джеймс Баучер the two rows are drawn too far **apart**, the
   near row on the building frontage and the far row on the tram tracks with the carriageway between
   them — one translation moves both the same way, so fixing one breaks the other. And a row's error
   is not constant along the street (one stretch of the `- S` row sits on its cars while another is a
   lane out), so a row is split by **clicking rectangles into groups**, each with its own rigid move.
   Cutting a row into N equal-count segments along its axis was tried first and does not work: a
   frame shows only a handful of *consecutive* bays out of 33, so they all land in one segment and
   the control appears to do nothing — and it cannot express "these two before the crossing, those
   three after". Groups of one are possible and to be resisted: an offset fitted bay by bay can be
   fitted to the *cars*, and then the occupancy question answers itself. Every visible row is drawn
   while you drag — active group bright, rest of the row in its group colour, other rows grey. **Align to the lane, never to the cars**: kerb, carriageway edge, tram rail,
   surviving paint. Several frames per street are provided so the offset can be checked where the row
   is empty. Output is a sidecar keyed by bay id, scoped to the bays this flight actually saw;
   `data/block_bays.geojson` is never touched and every consumer opts in with `--corrections`.
   Gated by `python vision/bay_corrections.py` (rigid-move self-test: round-trip 2e-13 m, edge
   lengths and radii preserved) and by the check that the page's linearised drawing equals the exact
   projection (688 vertices, **0.000 px** — a nadir projection is affine, so the page draws truth).

0b. **Label flight 0035's 88 bays** — `python vision/label_bays.py <stills> --out vision/data/gt_dji_0035.html`,
   label in the browser, Download JSON, then
   `python vision/score_real.py <stills> --gt vision/data/gt_dji_0035.json --sweep`.
   One crop per bay from its best view (the frame where it sits nearest the image centre), with the
   bay outline drawn, so the judgement asked is exactly the one the scorer makes: *is a car inside
   this rectangle*. `skip` is a real answer and those bays leave the confusion matrix. The scorer's
   self-test: a synthetic ground truth built from the pipeline's own 3 m output scores **100% at
   3 m**, confirming the replay matches the live rule before any real number is read from it.
   Pass `--corrections` so the crops show the CORRECTED rectangles.
   This produces the first real-world accuracy figure **and** settles the radius question.
1. **Decide on regenerating `block_bays.geojson`** (Finding 1). It is one command, but it moves the
   sim's painted pads, so it invalidates `ground_truth.json` and every accuracy figure in
   `CLAUDE.md`. The honest sequence is: regenerate, re-fly both survey worlds, re-score, and
   restate the numbers — and it wants a consistency check in `tools/check_consistency.py` asserting
   that `block_bays.geojson` reproduces from its inputs, so this cannot silently recur.
2. **Hand-label per-bay ground truth on flight 0035's street** — unchanged from `bay_data_gap.md`,
   and now better justified: both the paint control and the OSM audit say that street's geometry is
   sound, so a number measured there stands up.
3. **Then** the product decision in `bay_data_gap.md` §"The decision waiting for you". Nothing here
   changes the options; Finding 3 sharpens the evidence for the "extend the map" one.
