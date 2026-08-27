# PARKDRONE — open work

Last revised 2026-08-26. Five tracks. Detail that only matters while a task is
being worked lives in the task itself; the *why* lives here so a task can be
picked up cold.

Order agreed 2026-08-12: **#2 → #3 → obstacle track (#4)**. #2 and #3 are done;
the vision track (#5, #6, #7) is handled separately.

**Closed:** #1, #2, #3, #6, #5b, #10, #11, #13. **Open:** **#14 (the bay data is the bottleneck —
`docs/bay_data_gap.md`, the live question)**, #5 (learned detector — the model itself now works),
#4 residue, #7, #8 (P7), #9, **#12 (powertrain mismatch — do not fly as built)**, plus the two data
defects #1 uncovered.

---

## Obstacle avoidance

### 4. Stage B — steer around the obstacle — **WORKING, patrol COMPLETES**
Stage D (detect and stop) is done and verified. Stage B flies **past** every
structure: it commits a detour, rounds the obstacle on a tangent to a remembered
growing disc, skips waypoints buried inside it, and resumes the route on the far
side. Full write-up and the constants live in CLAUDE.md; `sim/analyze_stage_b.py`
reports a run from disk.

Design decisions, now implemented:
- **Steer, not climb.** Climbing would break the classifier's 30 m calibration.
- **A yaw bias into the navigator's existing channel** — never a lateral position
  command. A lateral position command saturates while off-heading and tumbles the
  drone; that is a standing controller invariant.
- The threat is remembered as a **growing bounding disc** in world metres, and the
  detour is released on **Bug2's leave condition** (closer to the goal than at
  commit, and a clear straight run to it). Both were arrived at the hard way; the
  geometric alternatives do not terminate.
- Clearance is one tight constant (`CLEAR_R` 5 m) that everything derives from,
  with per-obstacle escalation (double it, retry) instead of giving up.

**2026-08-19 — the patrol first completed:** wp96 of 97, 76 frames, all four
structures passed including `trap`.

**2026-08-20 — detours stopped being circles.** The completing run was still
flying full laps around structures (four detours sweeping 188–338°, two of them
around `mast`, a 0.9 m pole). Cause: commit and release asked "is it in my way?"
over *different* windows. Both now use `STEER_LOOK` (12 m past the current
waypoint), and `STEER_ORBIT` abandons any detour that has swept 270°, cooling
off that disc for 45 s. **270° and not less — a 200° cap ended in a CRASH.**
Result: still 97/97, in **6 detours instead of 18, none a circle**, 79 frames,
closest approach 6.55 m.

**Still open on this track:**
- **Skipped waypoints — MOSTLY RECOVERED 2026-08-20 (standoff capture).** The
  floor is stage D's 12 m standoff: a waypoint closer than that to a structure
  cannot be flown to, so it can never be *arrived at*. But its ground can still
  be photographed, so the controller now keeps each skipped waypoint as a
  standing target and shoots it at closest approach, flagged `standoff` +
  `standoff_m`. It never steers for a frame and the real waypoint always wins
  the camera. Measured on `fmi_block_obst`: of 18 unreachable waypoints, **9
  recovered**, bays classified **100 → 104 of 111**, uncovered **11 → 7**,
  accuracy **100%** throughout.
  The gate turned out to be the whole task: the footprint is 400×240 px with up
  = heading, so it reaches 12.4 m across the nose and 7.5 m along it. Testing
  the inscribed circle instead (7.4 m — the right answer only if yaw is unknown,
  which it is not at capture time) recovered **4**, less than half, because a
  detour passes its obstacle *abeam*.
  **The remaining 9 are a DECLARED GAP, decided 2026-08-20 — not a task.** Five
  of them are consecutive (49–53): a structure the route runs straight *through*
  rather than past, so a frame there would need a deliberate detour flown for
  coverage, which is a far more invasive thing than an opportunistic shot and
  buys ground the survey can honestly say it did not see. So the drone says it:
  the controller writes `sim/output/<area>/coverage.json` (waypoints, captured,
  unreachable, standoff, uncovered), rewritten on every change like poses.json
  so an interrupted flight still leaves an honest account, and
  `score_occupancy.py` prints the declaration next to the accuracy number. That
  matters because an uncovered bay has two very different causes — a bad pass
  (#7, fixable) or an obstacle (legitimate) — and only the flight knows which.
- The tower the route re-enters three times still costs one 238° swing and one
  `STEER_ORBIT` bail-out; `trap` is passed only via the escalation ladder (two
  150 s timeouts).
- Detour timeouts still cost 150 s each before escalating. A cheaper failure
  test (no progress for N seconds) would make a run much shorter.
- **Closest approach is 6.55 m**, inside the 12 m standoff by design (the
  committed obstacle gets the clearance-derived standoff). Worth a deliberate
  answer on what the real minimum should be for the hardware, given the 9-ray
  fan's 0.9 m gaps at that range.
- `--lowpoly`/1 km worlds have never been flown with the obstacle layer on.

Test world `fmi_block_obst` carries four deliberate failure modes: `tower` (wide,
head-on), `slab` (offset 7 m, clips the corridor without blocking it), `mast`
(0.9 m wide — the sparse-fan blind spot, and the argument for a Lidar) and `trap`
(concave U — expected to defeat a purely reactive controller, and the argument
for keeping some route memory).

---

## Vision — accuracy *(handled separately)*

### 2. Root-cause the ~0.6 m per-frame projection error — **DONE 2026-08-20**
`fmi_block` now scores **100%** (was 95.2%, then 97.6% after the `CORE_W` split).
`fmi_block_4st` unchanged at 100%; `fmi_block_4st_lp` 98.2% → 99.1%.

**Cause: `project()` assumed a nadir camera.** The Mavic2Pro gimbal's roll joint
sits *below* its pitch joint, so at the +pi/2 pitch a nadir survey flies, the
roll axis has been rotated onto the optical axis — it can no longer level the
camera, it only spins the picture. So the FULL body roll displaced every frame
laterally (slope **-1.02, R² 0.994**) while the compensated pitch contributed
only its few-mrad servo lag. `score_occupancy.camera_axes()` now models all
three effects; median per-frame misalignment **0.269 m → 0.043 m**, worst frame
**0.97 m → 0.060 m**, and every covariate R² is now ≤ 0.04.

What is left is understood and deliberately not chased: a constant **3.7 cm**
along-track bias, which is the size hypothesis 7 computed for the camera's
mounting offset ahead of the GPS (~3.8 cm). At 0.6 px it is below what the
classifier can feel.

Bay 17685 was therefore a *projection* error the whole time, exactly as the
2026-08-11 note suspected — but the mechanism was neither eccentricity nor lens
centre, which is why the affine test (hypothesis 1) could not see it.

**What made it findable, having failed with the 1-D profile:** measure in **2-D**
and prove the measurement first. `vision/diag/paint_align.py` registers the
projected outlines against the paint sub-pixel and `--self-test` recovers
injected shifts to 0.10 px; `vision/diag/offset_report.py` regresses the result
against the pose. The direction of the offset — almost pure cross-track — is
what ruled out timing and named roll in one step, and a 1-D profile across the
bay's short axis could not produce it. Both are **committed** this time; the
previous set lived in a session scratchpad and was lost.

Also fixed on the way, because the change made them matter: the web `frame`
table now stores `cam_pitch`/`cam_roll` (migration `0008`) so a job recovered
after a restart projects identically to the live path; `project()` treats a
null angle as absent; and `footprint_reach()` allows for the tilt so its cheap
rejection test cannot drop a bay that is genuinely in shot (verified equivalent
to the unindexed path over all 76 bay-views of `fmi_block`).

**Verified end-to-end 2026-08-20**, once Docker was up: migration `0008`
applied; `replay fmi_block` **42/42, 100%**; full-stack `replay_ingest` **42/42,
100%** with all 34 frame rows carrying `cam_pitch`/`cam_roll`; and the
restart-recovery path (stage frames with `CLASSIFY_THREADS=0`, restart,
`recovered_on_start: 34`) reproduces the **same 42/42 at 100%** — which is the
whole point of `0008`. That check can fail: stripping the two columns from a
pose moves bay 17685's projected outline by **0.35–1.55 m** on the frames that
see it, against a 0.21 m core-crop clearance.

### 5. Learned detector — TRAINED, AND NOW DRIVING THE LIVE MAP FROM REAL FOOTAGE
**Decided with the author (2026-08-21):** a learned model is a thesis deliverable in its own right
(master plan stage 9), it runs **server-side** in `vision-worker` (no Coral constraints), and it is a
**whole-frame detector** rather than a per-bay crop classifier. Real Sofia drone video (DJI, SRT
telemetry, over a Sofiaplan-mapped block) is coming from the author; sim and real results are to be
reported **separately**, never averaged. Framework is open — any modern detector — which points at
`transformers` RT-DETR/D-FINE (Apache-2.0) over Ultralytics (AGPL-3.0), though the supervisor's own
MIT repo uses Ultralytics, so precedent exists either way. Full plan:
`.claude/plans/tranquil-shimmying-candle.md`.

**Built:** `unproject()` + a 49,188-corner round-trip self-test, `<name>.cars.json` from the
generator (worlds byte-identical), `vision/dataset.py` (exact boxes, free), `detect_baseline.py`
(detection *and* occupancy scoring), `tools/dji_srt.py`. See CLAUDE.md §2b.

**Measured, and it settles the first question:** COCO-pretrained **YOLOv8m detects 0 of 52 cars**
across both survey worlds. Not a threshold problem (nothing at conf 0.01), not resolution (2-4x
upscaling changes nothing) — at floor confidence it calls a nadir car a *tie* and a van a *traffic
light*. Occupancy lands exactly on the base rate. **So fine-tuning is mandatory rather than an
improvement**, and the sim's free exact labels are what makes it cheap.

Worth noting the tension this creates with the prior art: the supervisor's pipeline runs this same
model zero-shot *successfully* on real footage. Either real texture carries it, or that footage is
more oblique than our strict nadir. Running `detect_baseline.py` against the real video when it
arrives answers it directly, and the answer shapes how much sim data is worth generating.

**2026-08-22 — real footage arrived, and it ANSWERS the tension above.** Three DJI clips over the
FMI block at a constant 30.1 m (the sim's calibration altitude), true nadir, 4K.
`tools/dji_stills.py` cuts them to stills paired with SRT poses. Zero-shot COCO YOLOv8m on real
frames scores **28.7% recall / 56.4% precision** whole-frame, **30.6% / 50.8%** tiled — against
**0 of 52** on Webots frames. So the supervisor's success and our 0% are both explicable: **the
renderer was a large part of the sim's 0%**, and what remains is viewpoint, colour and canopy.
Tiling to native scale does not rescue it (resolution is not the constraint — a car is ~385 px),
and the dominant failure is **224 `cell phone` detections at median confidence 0.78**. Failures
skew hard by **body colour and tree canopy**, neither of which the sim can generate — which is a
concrete limit on how much sim data is worth making.

**First hand-labelled data exists:** `vision/data/dji_0035/`, **31 frames / 108 boxes**, drawn by
the author, format-identical to `vision/dataset.py --yolo` so sim and real mix in one run. Its
`LABELING.md` holds the drawing rules and `README.md` the record. `vision/check_labels.py`
validates and draws them back. **The train/val split is SPATIAL and must stay so** — this flight
doubles back over its own street (frames 55-85 within 3-6 m of frames 5-30), so a random split
would score memorisation.

**In flight:** a fine-tune **probe** (`vision/train_detector.py`, yolov8s, imgsz 1024, 80 epochs,
CPU). It is a probe and not a result: 82 train boxes and **26 val boxes**, so one car is ±3.8% of
recall and anything under ~10% of difference is noise. What it can settle is whether labelling
several hundred more boxes is worth the hours. At epoch 26 it was already past the zero-shot
baseline on both axes (**R 0.50, P 0.476, mAP50 0.407** vs 0.306/0.508), which is the direction that
justifies more labelling. The compounding payoff is that a working fine-tune becomes the
**pre-labelling tool** for the remaining 106 frames of this flight and for video 0034 — correcting
boxes is 3-5x faster than drawing them, and it sidesteps the HF/OWLv2 download blocker entirely.

**2026-08-23 -- the detector now DRIVES THE LIVE MAP.** Three fine-tune runs finished
(`vision/runs/probe1|probe2|merged1`); `merged1` (yolov8s, 130 real frames / 366 boxes across
`dji_0035` + `dji_0035_r2`, imgsz 1024) reaches **mAP50 0.873, P 0.945, R 0.766** -- against COCO
zero-shot's 28.7% recall / 56.4% precision on the same footage. That model is now wired end to end:
**137 real DJI frames ingested through the live stack, 0 failed, 88 bays carrying a `detector`
verdict on the map.** Full write-up in CLAUDE.md 2c. The three things it needed:

- **Yaw, which the SRT does not record.** `tools/dji_yaw.py` recovers it from GPS displacement vs
  image registration. Phase correlation fails outright on canopy-heavy nadir footage (responses
  0.002-0.02, 6.5x off); normalised cross-correlation scores 0.66-0.84. Verified two ways: implied
  GSD 12.50 vs 11.74 mm/px expected, and flow-vs-course agreeing to a **median 1.9 deg**.
- **Camera intrinsics as a value** (`vision/cameras.py`), since real frames are 3840x2160 at
  73.7 deg against the proto's 400x240 at 45. Defaults keep every sim number byte-identical.
- **A second occupancy backend** (`OCCUPANCY_BACKEND`, migration `0010` for provenance), with the
  heuristic still the default so `replay fmi_block` stays 42/42 at 100%.

**Two findings worth carrying.** There is **no GPS bias worth correcting** -- the best global ENU
shift moves the median detection-to-bay distance only 4.35 -> 3.47 m, at an implausible 4.5 m. The
spread is instead a DATA fact: **most cars on this street are not in a Sofiaplan-mapped bay**,
visible directly in the `real_align.py` overlays where a column of bays sits over a pavement strip
while the cars are on the cobbles. And the map's freshness TTL was **wrong by 12x** (client 10 min
vs server 2 h) -- fixed, the server now publishes the window.

**Still open, and it is the obvious next task:** there is **no per-bay ground truth for flight
0035**, so this is a pipeline and a demo, not a real accuracy number (`/api/v1/metrics` correctly
reports `null`). ~86 bays fall under the flight's footprint; hand-labelling them occupied/free is
what turns this into a reportable real-world result comparable-but-never-averaged with the sim's
98.4%.

**2026-08-25 -- two more flights, and the bottleneck is now MEASURED, not inferred.** Flights 0074
and 0075 (same block, ~30 m nadir, **midday** rather than 16:31) were extracted, yaw-recovered,
nadir-screened and scored. `merged1` generalises across the light (confidence p10/p50/p90 0.33/0.63/
0.80 vs 0.33/0.68/0.83 on 0035; 4.3 detections/frame on 0075 vs 3.5) -- but **neither flight yields
a single occupied bay**, 0 of 49 and 0 of 9 from 273 detections. That is the bay data, and 0035 is
the positive control that proves it is not the vote: there an occupied bay reads 11/13, 10/12, 7/7,
6/6 views, while 0074's best bay is 6 of 21 and 0075's nine bays have zero. Not a yaw error, and no
shared bias to correct (0075 wants (-7,-8), 0074 (+3,-9), three minutes apart). The overlays show
painted, occupied bays in the photograph where the Sofiaplan rectangles sit on a garden.
**So "hand-label the ~86 bays under flight 0035" is no longer obviously the next task** -- the more
informative measurement is how many parked cars on this block are in a mapped bay at all, which is
now three flights' worth of evidence saying "few".

**CAVEAT ADDED 2026-08-26 — this conclusion is INDICATED, NOT SETTLED, and was first written too
firmly.** What is solid: the pose is self-consistent to ~1 m (the same car detected in >=3 frames
unprojects to the same ground point: median scatter 0.94 m on 0075, 1.08 m on 0074, p90 ~2.5 m), OSM
building footprints project onto their real buildings (`vision/diag/ref_align.py`, flight 0074
frame 57), and the vote is demonstrably working. What was over-claimed: "there is no shared bias to
correct". Self-consistency is **blind to a constant per-flight offset**, and the grid search says one
would take 0075 from 0 to 51 of 100 detections on-bay. The argument that the two flights want
different shifts does NOT separate a per-flight GPS bias from a per-street data offset, because the
two flights are on different streets — those two explanations are indistinguishable in that test.
Settling it needs an **absolute** check against flat ground control (painted bay markings, which
unlike a roof have no parallax); until that is done, no real-world accuracy number should be quoted
against these bays.

**One live-path bug fell out and is fixed:** ultralytics reads a numpy array as BGR, and three call
sites -- including the server's detector backend -- built theirs from PIL RGB, so the live map ran on
colour-swapped frames (473 -> 599 detections on 0035, +27%). Training labels and every reported
recall/precision figure are unaffected (`prelabel.py`/`detect_real.py` use cv2). Details in
CLAUDE.md 2f and `docs/training.md`.

**Ready for the author:** `vision/data/dji_0074` (36 frames / 60 boxes) and `dji_0075` (23 / 100),
pre-labelled with `merged1` and validated, waiting on hand correction. Both train-only -- 0074 has
no split point reaching the 25 m footprint because it hovers and doubles back over its own ground.

**Next:** finish the probe and report it honestly against the 26-box val caveat; then either label
more (if it moved) or diagnose structurally (if it did not). Dataset volume is bounded by flight
length (~0.9 cars/frame in sim, ~3.5 boxes/frame on real stills), so more data means more flights;
fine-tune from aerial-pretrained weights if the HF download blocker is cleared (CLAUDE.md §2b),
else from the Ultralytics path that already works.

### 5b. The old framing — learned *classifier* v2 — **CLOSED, folded into #5**
`classify()` is five hand-tuned thresholds calibrated on `fmi_block_4st`. The
case for replacing it is now **weaker on the numbers and unchanged in
principle**: the chroma-12.99 false positive that used to be the headline
evidence turned out to be #2's projection bug, not a classifier limit, and both
survey worlds are at 100%. What still stands: the `CORE_W` sweep is
non-monotonic because a bay flips across a threshold, `fmi_block_4st_lp`'s
remaining miss is a dark car a colour heuristic cannot separate from asphalt,
and the light-pole parallax case is a physically real occlusion.

`classify()` is deliberately the only swap point — projection, view selection,
voting and scoring stay unchanged. Labelled crops are already on disk.

**Closed 2026-08-21 by #6's ablation.** The old note here said to sequence this *with* scene
hardening because "on the current uniform-lighting scene a learned model has nothing to beat".
Scene hardening has now happened, and it produced something to beat: under a low sun the heuristic
scores **44.1%**, and no normalisation recovers it without breaking the calibrated worlds, because
`chroma` — the only load-bearing test — is exactly what a colour correction destroys. So the case
for a learned model is no longer "weaker on the numbers"; it is the robustness argument, made
quantitatively. There is nothing separate left to plan here: the work is #5, and the lighting worlds
are its benchmark.

### 6. Shadows — FLOWN AND MEASURED 2026-08-21. **CLOSED: the heuristic has no normalisable form.**
`castShadows FALSE` was never an aesthetic choice: shadow mapping over a 1.2 km ground plane paints
streak artifacts across the nadir frames, so turning it on as-is would have hardened the classifier
against a *rendering defect*. The quality fix rides in the same flag — with `--shadows` the ground
plane shrinks from `2*WINDOW+200` to `2*WINDOW+40`, because a directional light spreads one shadow
map over the scene extent and the plane *is* the extent. **Verified by looking before measuring:**
the frames show car shadows, building shadows and a light-pole shadow, cleanly, with **no streaks**.

**The experiment had to be run as a 2x2, because `fmi_block_4st_sh` moves two variables at once**
(`--shadows` *and* `--sun 135,25`). Two more worlds were generated to separate them —
`fmi_block_4st_sun` (low sun, no shadows) and `fmi_block_4st_shd` (shadows, default sun) — both with
`ground_truth.json` and `route.json` **byte-identical to `fmi_block_4st`**, so only the light differs
in any cell. All four flew 97/97:

| | shadows OFF | shadows ON |
|---|---|---|
| **default sun (dir `0.4 0.5 -1`, ~60 deg elevation)** | **100.0%** `fmi_block_4st` | **89.2%** `fmi_block_4st_shd` (FP=12) |
| **low sun (`--sun 135,25`)** | **44.1%** `fmi_block_4st_sun` (FP=62) | **44.1%** `fmi_block_4st_sh` (FP=62) |

**Recall is 100% in every cell — not one car is ever missed.** Every point lost is a false positive
on a *free* bay, so the failure mode is "the drone thinks the street is full", never "the drone
misses a car".

**The prediction on record was right about the mechanism and wrong about the cause.** It expected
shaded free bays to fall below `T_BRIGHT_LO` (82). They do — the shadows-only cell loses 12 bays,
every one of them shaded (`bright` 53-83, `dark_frac` up to 1.00). But the *dominant* effect is not
shading at all: **the low sun alone, with shadows off, produces the entire collapse to 44.1%** with
the identical confusion matrix. A 25 deg sun on a horizontal plane cuts Lambert irradiance to
cos(25)/cos(60) ~ 0.48 of the default, and every free bay drops from `bright` 85 to 72 — *uniformly,
including bays in full sun*. Below 82, so all 62 read as occupied. The shadow cell then adds nothing
further because everything is already outside the envelope; the two lower cells are the same number
for the same reason.

**What the free-bay distribution says is the whole point:**

| world | free-bay `core_brightness` | best single-threshold separability |
|---|---|---|
| `fmi_block_4st` | median 85, range 85-87 | 87.4% |
| `fmi_block_4st_sun` | median 72, range 72-75 | — |
| `fmi_block_4st_sh` | median 72, range **53**-75 | 86.5% |

The distribution **translated down 13 grey levels and kept its tight spread**; separability barely
moved. So the scene is not harder and the signal is not destroyed — `classify()` fails purely
because `T_BRIGHT_LO/HI` is an **absolute** tone envelope. Shadows' only distinct contribution is
widening the *lower tail* (53 vs 72) for the genuinely shaded bays, which is the real, physical,
harder case; the global shift is the trivially-fixable one.

**ATTEMPTED AND MEASURED 2026-08-21 — "give the heuristic its fair form" does not have an answer.**
`vision/diag/classifier_ablation.py` (committed; extracts every per-view feature once per world,
caches it, then evaluates a classifier variant in milliseconds against the exact vote
`score_occupancy.main()` uses; its `current` row reproduces every committed accuracy as a self-test).

**First, the mechanism above was WRONG, and the harness is what caught it.** The collapse is not the
brightness envelope. On all 62 false positives in `fmi_block_4st_sun`, **`chroma` and `bright` fire
together**:

| free-bay core | `fmi_block_4st` | `fmi_block_4st_sun` | threshold |
|---|---|---|---|
| `core_chroma` | 11.00 | **14.00** | > 12.7 |
| `core_brightness` | 84.7 | **72.3** | outside 82–102 |

So removing *either* test alone changes nothing — `drop bright` still scores 44.1% with the same 62
FP. And note chroma **rises** as the light dims: a shallower sun means proportionally more of each
surface's light comes from the tinted ambient sky, so the grey asphalt takes on a colour cast. That
is a change in the light's **spectrum**, not its level, and no intensity normalisation can undo it —
`rel/f_mode` scales `T_CHROMA` the *wrong way* and makes things worse.

**Second, and this is the finding: `chroma` is the only load-bearing test, so every colour
normalisation attacks the signal itself.** Dropping `chroma` costs `fmi_block_4st` 100% → 89.2%
(12 cars missed); dropping `dark`, `paint` or `std` costs **nothing at all** on either golden world.
Grey-world cast correction equalises the channel means — which also suppresses the very colour a red
car is detected by, and the golden world falls to 93.7% with 7 missed cars. Measured:

| variant | fb | fb_4st | fb_4st_lp | fb_4st_shd | fb_4st_sun | fb_4st_sh |
|---|---|---|---|---|---|---|
| current (absolute) | **100** | **100** | 99.1 | 89.2 | 44.1 | 44.1 |
| rel/f_mode | 100 | 99.1 | 98.2 | 88.3 | 44.1 | 44.1 |
| white-balanced (per-channel mode) | 95.2 | 93.7 | 98.2 | 84.7 | **89.2** | 82.0 |
| grey-world | 95.2 | 93.7 | 99.1 | 85.6 | 42.3 | 43.2 |
| grey-world + rel/f_mode | 95.2 | 92.8 | 98.2 | 84.7 | **93.7** | 78.4 |

**Nothing recovers the lighting worlds without breaking the calibrated ones.** The best sun result
(93.7%) costs `fmi_block_4st` 100 → 92.8.

Two structural reasons, both measured, both worth keeping:
- **The margin is 3.5%.** Free cores sit at brightness 85 against a floor of 82. Any absolute
  intensity test needs the scene illumination estimated to ~1–2%, which will not happen on real
  footage.
- **There is no stable local reference in this scene.** It is three flat tones — ground 121 (56% of
  pixels), road 50, bay pad 85 — so a ring around a bay reads either 121 or the *neighbouring pad's*
  85 and is bimodal by construction (CV 0.115 on the calibration world). Frame-wide estimators are
  composition-dependent instead: the per-channel mode jumps when road or grass becomes the plurality,
  which is exactly why white-balancing costs the golden worlds.
- Worth noting `bright` **is** load-bearing for the box cars: `drop bright` costs `fmi_block_4st_lp`
  99.1% → 91.9% (9 missed). A uniform dark box has no chroma to find.

**So the sequencing question in 5b is answered: there is no fair form of this heuristic that survives
a lighting change.** Robustness is not a tuning problem here, and hardening `classify()` further is
not worth more time. This is now positive evidence for the learned detector (#5) rather than a
prerequisite for it — and it says what the model must be trained on: **varied illumination**, which
`--sun`/`--shadows` can now generate cheaply and with exact labels.

### 7. Reduce capture scatter (~3 uncovered bays)
Timeout arrivals capture up to ~15 m off the waypoint, so ~1% of bays fall
outside every footprint: 3 of 45 on `fmi_block`, 0 of 111 on `fmi_block_4st`.

More interesting than the count: bay 17685 (#2) was decided by a **single** view
while correctly-classified free bays got 2–3. The multi-view majority vote is the
designed defence against one bad look, and it is exactly the thinly-covered bays
that failed — so coverage work also hardens the vote. (#2 is fixed, so 17685 is
no longer wrong; the structural point stands.)

Untried: fly higher for a larger footprint (costs resolution, 16 → ~10 px/m at
50 m, and needs the classifier re-verified at that scale); re-aim the capture
yaw; tighten the orbit-timeout bail-out. **Do not tighten `WP_REACH` below 6 m** —
at cruise speed the turn radius is ~4 m and the drone settles into a stable orbit
inside a tighter basin, hanging the patrol forever.

---

## Infrastructure

### 3. A consistency test for constants duplicated across files — **DONE**
`tools/check_consistency.py`, exits 1 on mismatch. The check count is
data-driven (one per world, one per captured-frame set), so it moves as worlds
and flight outputs come and go — currently 25. Covers the ENU
projection (`ORIGIN`/`MLAT`/`MLON` across the two Python files *and* `geo.ts`,
including that all three still *derive* `MLON` rather than hardcoding it), the
`DS_*` sensor fan, the bay dimension and orientation rules, a per-bay
drawn-vs-stored geometry check over every world (50 mm tolerance; float printing
alone costs ~2 mm), and the camera resolution — checked against the **actual
captured PNGs** rather than the proto, since the frames are ground truth for what
the camera did.

**Verified by mutation**, because a check that cannot fail is worthless:
deleting the `Косо` branch, drifting `ORIGIN` by 1e-5° (~1 m) and changing
`DS_RANGE` in one file each produce exit 1 with a diagnostic naming the two
disagreeing sources.

Known limitation, documented in the file: check 4 re-implements the orientation
rules rather than executing the generator's copy (importing `generate_world.py`
builds a world as a side effect), so it validates the *data*; check 3, which
parses the generator as text, is what ties the generator to the rules.

*Original rationale:* two bugs found on 2026-08-11/12 were the *same* failure —
a rule living in two files with nothing checking they agree.

- **Bay orientation** — `tools/make_bays.py` had three `park_txt` cases,
  `sim/generate_world.py` had two, so all 12 `Косо` bays in the 1 km world were
  drawn 45° off their painted rectangle (2.23 m corner error). Invisible because
  `fmi_block` and `fmi_block_4st` contain **zero** angled bays.
- **The golden fixture** — July frames re-scored in August, describing a world
  that no longer existed.

Other live duplications with identical risk:

| Constant | Duplicated in |
|---|---|
| `ORIGIN` / `MLAT` / `MLON` | `generate_world.py`, `score_occupancy.py`, `web/packages/contracts/src/geo.ts` |
| `DS_N` / `DS_SPREAD_DEG` / `DS_RANGE` | `generate_world.py`, `parkdrone.py` |
| Bay `L`/`W` dimensions | `make_bays.py`, `generate_world.py` |
| Camera intrinsics (`IMG_W`/`IMG_H`/`FOV`) | the Mavic proto, `score_occupancy.py` |

Proposed: one script asserting all of these agree, plus the per-bay check that
caught the `Косо` bug (drawn rectangle vs geojson ring, 2 mm tolerance).
~1 hour, retires a whole category of silent georeferencing bugs.

---

## Data / scale

### 1. Score the 1 km survey — **DONE 2026-08-21. 98.4%, and every error is explained.**
The relaunched flight **completed: 1976 of 1976 waypoints, 1976 frames**, landed at (507.3,-230.9).
It had resumed from frame 87 on disk after being stopped by hand on 2026-08-17, which is the
documented resume behaviour working as intended over a ~19 km, 5 h patrol.

**The project's first neighbourhood-scale accuracy number:**

    1593 bays, 1516 classified, 77 uncovered
    TP=692  TN=799  FP=25  FN=0
    accuracy 98.4%   precision 96.5%   recall 100.0%

**FN=0 — not one of the 692 parked cars was missed.** All 25 errors are false positives on free
bays, and all 25 have a cause. None is a classifier-threshold failure:

| cause | bays | error rate |
|---|---|---|
| bay centroid lies **inside** an OSM building footprint (the camera photographs the roof: `bright` 165-198, `std` 4-12, a flat bright surface) | 14 of the 20 such bays | **70%** |
| bay within 3 m of a building — at 30 m the wall leans into the crop by parallax, and these bays are all off-centre (`off` 33-141 px) | 7 | — |
| bay rectangle **geometrically overlaps** a neighbour that holds a car, so the car is physically inside the free bay's core | 4 | — |
| ordinary street bays | **11 of 1463** | **0.75%** |

Excluding the 20 building-covered bays entirely: **99.7%** (1496 classified, 4 errors).

**The bays-on-grass prediction did not happen, and that is a real answer.** All **33** bays whose
centroid falls on a green polygon classified **correctly**. The reason is already in the design: the
bay pad is painted at z=0.04-0.06 and greens at z=0.005, so the pad covers the grass and the bay
looks like tarmac from above. Green areas are not a hazard to the classifier; **buildings are**.

**Two findings worth carrying, both about the DATA and not the vision:**
- **A bay inside a building footprint is unscoreable by construction.** 20 of them exist in the 1 km
  cut. Either the Sofiaplan point sits wrong, or the OSM footprint covers a courtyard/passage the
  bay legitimately occupies. Whichever it is, the generator currently paints a bay pad *and* raises a
  `SimpleBuilding` on the same ground, and the drone can only ever see the roof. Candidate fixes: drop
  such bays from the world and from `ground_truth.json`, or flag them so the scorer excludes them —
  the second is more honest, since a real deployment would still fly over them.
- **222 overlapping bay pairs among 1698 bays.** Where two bay rectangles overlap, a car in one is
  inside the other's core and the ground truth itself is ambiguous — no classifier can be right. The
  same abbreviation/orientation data quality thread as #9.

Also of note: the 1 km world's scene reference tone (median `core_brightness`, 84.7) is identical to
`fmi_block_4st`'s, so the 98.4% is not lighting-shifted relative to the small worlds and the numbers
are directly comparable.

`sim/output/fmi_block_1km/` now holds the world's first clean fixture (frames, `poses.json`,
`occupancy_results.json`), in the same canonical place as the two small worlds — so
`tools/check_consistency.py` picks it up (32 checks pass, up from 25 with the three new light worlds).
**Note for #9:** geometry-based street matching would change this route, so re-cut and re-fly only
against this now-scored baseline.

### 9. Street-name matching — DIAGNOSED 2026-08-21, half fixed
4 of 49 streets in the 1 km cut have no OSM name match and fall back to a
straight-line PCA bearing: жк Лозенец, Арх. Йордан Миланов, Кръстю Сарафов,
Св. Седмочисленици. The fallback is only correct for *straight* streets.

**Measured, and the answer is "it depends which street":**

| street | bays | why it failed | does it curve? |
|---|---|---|---|
| ул. Арх. Йордан Миланов | 38 | abbreviation — OSM has `Архитект Йордан Миланов` | rows bend 3.5 / 5.7 m |
| ул. Св. Седмочисленици | 10 | abbreviation — OSM has `Свети Седмочисленици` | no, 0.03 m |
| ул. Кръстю Сарафов | 29 | **no OSM way carries this name at all** | **yes — 8 m per row** |
| жк Лозенец | 10 | it is a housing estate, not a street | no, 0.01 m |

The first measurement was misleading and worth recording: the raw deviation of a
street's bays from a straight line reached 18 m, which looks like heavy
curvature and is actually **bays on both sides of the road** (the two rows sit
7.8-14.3 m apart — that is the street width). Splitting each street into its two
rows first is what makes the curvature question answerable. And two-sided rows
are not even a problem for the fallback: a PCA line through both rows is roughly
the centreline, which is what a route wants.

**Fixed:** `norm_street()` in `generate_world.py` strips the class prefix
(ул./бул./жк) and expands the known abbreviations before matching, which
recovers two of the four. It is deliberately conservative — no transliteration,
no fuzzy distance — because a loose match pairs a bay row with the WRONG road,
and flying a real street that is not the one the bays are on is worse than the
straight-line fallback.

**Still open:** `ул. Кръстю Сарафов` has 29 bays, no OSM name to match, and rows
that genuinely deviate ~8 m from straight — the one street where the fallback is
really wrong. The fix is to match by GEOMETRY (nearest road polyline to the bay
row) rather than by name, which would subsume the name matcher entirely.
`жк Лозенец` should stay on the fallback: straight, and an estate rather than a
street.

**Blast radius, checked before changing anything:** all four streets lie outside
the 75 m and 130 m windows, so both golden worlds regenerate **byte-identical**
(verified). Only `fmi_block_1km` is affected — and it is mid-flight on the
current route, so regenerate and re-fly it only after the running survey has
been scored.

---

## Web

### 8. Web tier: P6 is BUILT and the stack is VERIFIED; P7 hardening remains
The documentation contradiction is resolved: **`apps/web-admin` is fully built**
(App + 5 components + `useMetrics` + `lib/format.ts`) and the whole stack came up
and passed end-to-end on 2026-08-20 — see #2's verification note. The stale line
was the status summary, not the body.

**Committed but never verified** (it went in with `2964746` "Fix steering", a
catch-all commit — the 2026-08-20 note calling it uncommitted was wrong): a
WebSocket **reconnect-cursor** feature across
`api/hub.py` (an asyncio lock serialising connect+broadcast, plus a cursor on
every delta), `useOccupancySocket.ts` (`?since=`, process-restart detection that
drops a stale overlay), `App.tsx` (REST reconcile on `syncVersion`) and
`contracts/src/index.ts`. It looks complete and it survived the E2E run, but it
was never deliberately reviewed or tested against an actual reconnect. **Do that
before it gets committed** — a mid-flight drop with a stale cursor is exactly the
case it exists for and exactly the case nothing has exercised.

**2026-08-21 — two of P7's items are done.** `/api/v1/metrics` and `/metrics` now require
`x-admin-key` whenever `ADMIN_API_KEY` is set (401 without, 403 wrong, 200 right — verified),
open with a loud startup warning when it is not, and the ops dashboard's vite proxy injects the
header so the secret never enters the browser bundle. And the **reconnect cursor is finally
exercised**: a client that drops at cursor N and reconnects with `?since=N` gets exactly the
deltas it missed, in order, with nothing it had already applied and no backlog for a fresh
client; across a server restart the cursor resets below the client's, which is the signal
`useOccupancySocket.ts` uses to discard a stale overlay and reconcile from REST.

**2026-08-24 — horizontal scaling is no longer a P7 blocker.** This section used to end by
warning that single-replica was a correctness requirement. It is not any more: migration `0011`
added job claiming, a shared delta channel and a global cursor, verified with two replicas
(#11). Nothing in P7 now depends on it.

Still open in P7 (the list below was lost in an earlier edit of this file and is restored here
from `docs/web_infra_plan.md` "Security follow-ups" + the Production section):

- **A real admin login** — sessions, users, an audit trail. `ADMIN_API_KEY` is a shared secret,
  which is the smallest thing that closes the door on `/metrics`, not an answer.
- **The dev toggle still has no auth**, only a default-off flag. `POST /api/v1/dev/occupy|free`
  must at minimum require the drone API key before anything beyond localhost can reach `:4000`.
- **Production secrets** — the one that actually matters. No value that has ever been in this
  repo gets reused: Postgres, the object store and the drone keys are all generated at deploy
  time into a real secret store. Until that exists, the whole stack is dev-only.
- **Deployment**: containers, a process supervisor, PgBouncer in front of Postgres, CI. Now that
  several replicas are safe, a rolling deploy is worth having — and it needs no sticky WebSocket
  routing, because the cursor and the replay buffer live in the database.
- **Observability**: the exposition side exists (`GET /metrics`); a scrape config, Grafana
  dashboards and alerting on `oldest_queued_age_s` / `expired_leases` / `job_failure_rate` /
  `frames_per_minute` do not.

### 10. A re-flight should ingest itself — **DONE 2026-08-21**
**Today it does not, and it fails silently.** Fly a survey area the stack has
already seen and every frame comes back `200` duplicate: ingest is idempotent on
`(drone_id, survey_area, frame_idx)`, so there is no classify job, no
`bay_state` change and no WebSocket delta. The drone flies a full patrol and the
map does not move. Nothing errors — the uplink warns once and goes quiet — which
is exactly why this cost an evening on 2026-08-20. `pnpm clear <area>`
(`scripts/clear.sh` → `parkdrone_vision/clear_area.py`, added the same day) is
the workaround, not the answer: **wiping the history to record new state is
backwards**, and no real deployment can do it — the whole point of the
`observation` table is that it is the analytics record.

**What it should do instead:** keep everything, and let a new flight simply add
to it. Re-flying an area 2 h later (past `OCCUPANCY_WINDOW_S`) should re-record
every bay from scratch, because by then the old votes are stale by definition
and nothing is being contradicted.

Three layers have to agree, and only the first is really a design decision:

- **The idempotency key is wrong.** `frame_idx` restarts at 0 on every flight, so
  `(drone, survey_area, frame_idx)` says "frame 7 of this area" when it means
  "frame 7 of *this flight*". A `mission_id` already exists and already scopes a
  flight — keying on `(drone_id, mission_id, frame_idx)` makes a re-flight ingest
  naturally while still absorbing the retry-a-frame case the constraint was
  written for. `frame` rows would then accumulate per flight; retention
  (`FRAME_RETENTION_S`, 4 h) already bounds that.
- **Then decide what the vote means across flights.** The occupancy vote is
  currently "every observation inside the window", so two flights inside 2 h
  would mix an old look with a new one for the same bay. Past the window that is
  moot (the point of this task), but inside it the honest rule is probably
  *latest mission wins per bay*, with earlier ones kept as history. That is a
  product decision about what "current" means, not a refactor.
- **The two disk-side resume behaviours are separate and stay:** the controller
  resumes from `poses.json` (so a genuine re-fly still needs that folder cleared
  — the flight's own semantics, nothing to do with the DB), and `sim_uplink`
  re-posts from frame 0 when it sees `poses.json` restart. Neither should be
  changed by this task; the point is that the *database* stops being the thing
  that blocks a re-flight.

Worth noting what is NOT a bug: a delta only fires on a *change*, so re-recording
an unchanged bay pushes nothing and the map correctly does not repaint. If a
demo needs to see liveness regardless, that is a separate "still fresh"
heartbeat (refresh `updated_at`, push a no-op), not a change to the vote.

**Done, migration `0009`.** The key is now
`(drone_id, COALESCE(mission_id, 'area:'||survey_area), frame_idx)`; the COALESCE sentinel keeps
the old behaviour for mission-less clients instead of silently dropping their deduplication. The
vote resolves the newest mission per bay and counts only that flight, so two flights inside one
window cannot average a stale look with a fresh one. The stored image key carries the mission too —
otherwise a re-flight overwrites the earlier flight's image while its `frame` row still points at
it.

Verified: five missions of `fmi_block` coexist (34 frames each, 87 observations each, no duplicate
views); a re-flight ingests with nothing cleared and `/bays` matches `occupancy_results.json`
**42/42**; restart recovery re-enqueued the 34 staged frames and reproduced the same 42/42; and the
vote rule was tested with teeth — 20 contrary views from an older mission change nothing, the same
20 re-labelled to the newest mission flip the bay.

`pnpm clear` stays, for what its name says: wiping an area deliberately.

### 11. Multi-replica: job claiming + a shared delta channel — **DONE 2026-08-24 (migration `0011`)**

Raised 2026-08-23 while reviewing the thesis draft, which claimed "any replica with a DB connection
can pick up a job". That was the opposite of the truth (review finding §1.1 in
`docs/final_doc_review.md`). Rather than weaken the sentence, the code was changed to match it.

**What was actually wrong** — three things per-process or unowned, and the first was a defect even
at one replica:
- `jobs.recover()` re-enqueued every `status='queued'` row with **no ownership filter**, so two
  replicas both drained the whole backlog: one frame scored twice, two `observation` rows for one
  look, and a vote that double-counts. And even alone, the pipeline is at-least-once by
  construction — `process_frame` commits the observations, then `mark_frame_processed` commits in a
  *separate* transaction, so a crash between them re-scores the frame on the next boot.
- The WebSocket hub was per-process: a delta from replica A never reached a browser on B.
- `Hub._cursor` was a per-process counter starting at 0 every boot, so `?since=` meant something
  different on each replica *and* after every restart.
- (Found on the way.) A job that failed for any reason other than a missing image stayed `queued`
  **forever** and was re-run on every restart — no attempt counter, no dead-letter.

**What was built.** All of it in Postgres — no Redis, no broker; the DB stays the only shared state.
- **Claiming.** `web_db.claim_frames` takes rows with `FOR UPDATE SKIP LOCKED` and stamps a
  `claimed_by`/`lease_expires_at` lease; `frame_job` also gained `attempts` and `last_error` and a
  `running` status. One dispatcher thread per replica claims into the existing local `queue.Queue`
  and the classify threads are untouched. **`recover()` is gone, and that is an upgrade**: the claim
  query's second arm takes any lease that stopped being renewed, so a dead replica's work is picked
  up by a *live* one within `LEASE_S` instead of waiting for the dead process to come back.
  Recovery stopped being a startup step and became something continuous.
- **`observation` is idempotent per `(frame_id, bay_id)`** (partial unique index, partial so
  pre-`0011` NULLs do not collide). Claiming cannot fix the sequential re-score above; only a
  unique key can.
- **Deltas over `LISTEN`/`NOTIFY`, ordered by a `bay_delta` table.** `process_frame` publishes in
  the **same transaction** as the state it announces — NOTIFY fires on COMMIT, so the wire is never
  ahead of the database. Each replica's listener thread delivers to its own clients, and a replica
  hears **its own** deltas back the same way: one delivery path, so every client sees one order.
  `bay_delta.id` is the cursor, the table is the replay buffer, and both are shared.
- Replay reads a little **behind** the client's cursor (`DELTA_REPLAY_SLACK`), because sequence ids
  are assigned before commit and can become visible out of order. A delta is an idempotent "set bay
  X to this state", so over-replaying is free and missing one is not.

Two things kept deliberately shallow, both of which would otherwise reintroduce the problem under a
new name: **local prefetch** (`CLAIM_PREFETCH`) — a replica that claims the whole backlog holds
leases on work it will not start for minutes — and the **poll**, which is a plain 1 s tick rather
than a second NOTIFY channel, because at ~0.5 frames/s per drone a second of latency on
cross-replica pickup is nothing.

**Verified with two replicas** (A :4000, B :4001, one Postgres, one MinIO):

| check | result |
|---|---|
| staged 34-frame backlog, both replicas started | **17 / 17** split, `claimed_by` shows both |
| duplicate `(frame_id, bay_id)` observations | **0**, and re-running all 34 frames left the count at **87, not 174** |
| `/api/v1/bays` vs the golden fixture, *both* replicas | **42/42, 100%** (TP=17 TN=25 FP=0 FN=0) |
| deltas published by A, client attached to **B** | **49 / 49**, identical cursors |
| resume with A's cursor against B | replayed correctly; both report the same `snapshot_cursor` |
| 12 jobs left `running` by a vanished replica | reclaimed **8 then 4** (the prefetch gate), finished clean |
| missing image / corrupt image | `failed` after **1** attempt / after exactly **3**, `last_error` recorded, neither loops |
| cleanup past retention with a job `running` | left in place, warned, exit 1 |

**The single-replica path is unchanged, and that was proved by A/B rather than argued.** The same
end-to-end run against the stashed pre-change code returns the identical `/api/v1/metrics` model
block (`views_scored 524, views_correct 508, view_accuracy 0.9695, bays_scored 49, bays_correct 42,
state_accuracy 0.8571`) and the same **49** WebSocket deltas. Offline `replay fmi_block` is 42/42 at
100%, `check_consistency.py` still 35/35.

*Correction to the record while measuring this:* the "**52** WS deltas" in `CLAUDE.md` is stale — it
predates the 2026-08-20 camera-model fix. **49** is right, and it is exactly the number of
`bay_state` transitions the observation history contains (checked independently in SQL).

**Still open, and deliberately not in scope here:** running >1 replica in production also wants a
deploy story and a process supervisor. It does *not* want sticky WebSocket routing — any replica
serves any client now, because the cursor and the replay buffer live in the database.

---

### 13. A CLOSED STREET must be represented end to end — **BUILT & VERIFIED 2026-08-25**

A street can be closed — roadworks, a market, an accident, a parade. Today nothing in the chain
knows the concept exists, so a closed street is silently surveyed and its bays are reported as
ordinary free/occupied parking. The point of this task is that "closed" is not one feature in one
place: it has to be *indicated everywhere along the chain*, and each link means something different.

**The chain, link by link — what "closed" means at each:**
- **Data (`data/`, `tools/`).** Where does the closure come from? There is no closure field in the
  Sofiaplan datasets and OSM's `access=no` / `highway=construction` on a way is at best a
  long-lived closure, not a temporary one. So the first decision is the *source*: a hand-authored
  `data/closures.geojson` (street geometry or a polygon + a validity interval) is the honest
  starting point, with an OSM/municipal feed as a later upgrade. Whatever it is, it must be able to
  say **which** street and **for how long**, because a closure that never expires is a different
  product from one that does.
- **Route planning (`sim/generate_world.py`).** The postman walk covers every street that has bays.
  A closed street should either be dropped from the coverage set (do not fly it) or kept and flown
  but marked — that is a real design choice and needs deciding, not assuming. Dropping it changes
  the walk, so `route.json` and `ground_truth.json` stop being byte-identical: gate it behind an
  opt-in flag exactly like `--shadows`/`--obstacles`, so the three survey worlds regenerate
  unchanged and every accuracy number on record stays valid.
- **The controller (`sim/controllers/parkdrone/parkdrone.py`).** If a closed street is skipped, its
  waypoints join the existing "declared, not hidden" machinery — the same discipline as an
  unreachable waypoint behind an obstacle: `coverage.json` must say the ground was not covered and
  **why** (`closed`, not just `uncovered`). Never let a closure look like a bad pass.
- **Scoring (`vision/score_occupancy.py`).** Bays on a closed street must not be counted as
  free — an empty bay on a closed street is not available parking. They are a third state, and
  folding them into either existing one corrupts the accuracy number in the direction that
  flatters us.
- **Schema + server (`web/`).** This is the bulk of the work. `bay.occupied` is a boolean today;
  the read paths already have an `occupied: null` = *unknown* (stale) case, and **closed is not
  unknown** — it is known and negative. Expect a new nullable `closure` concept (a `closure` table
  keyed by street or geometry + validity window, joined at read time the same way freshness is
  derived at read time rather than swept), a field on `/api/v1/bays` and `/summary`, and a delta
  when a closure opens or ends so the map moves live. Decide explicitly whether an observation on
  a closed street is still *recorded* (it should be — `observation` is history) while being
  excluded from the vote's product answer.
- **Both UIs (`web-user`, `web-admin`).** The driver map needs a visibly distinct third state —
  not red, not green — plus the street itself drawn as closed, and the popup saying so. Watch the
  Leaflet `preferCanvas` click-order trap documented in CLAUDE.md if the closure is drawn as a
  layer over the bays: draw it **before** the bays or it swallows every bay click inside it, the
  same way the airspace layer would have. The ops dashboard should report closed streets and
  exclude their bays from coverage, or coverage permanently reads short with no explanation.
- **The no-fly map is NOT this.** A closed *street* is a ground-traffic fact; a closed *airspace*
  is `nofly.py`. Do not conflate them — but note the closure layer is the second piece of
  reference data with a validity window, so it is worth looking at how `nofly.py` does it before
  inventing something new.

**Why it is worth doing:** it is the first thing in the product that is *not* derived from the
camera. Everything so far flows one way (fly → detect → vote → map); a closure flows the other way
(an external fact that overrides what the camera saw), and getting it right proves the chain can
carry an authoritative override at all. It also exposes, in one concrete case, the gap between
"this bay is empty" and "you can park here", which is the actual product claim.

**BUILT 2026-08-25.** The whole chain carries it, and the design choice that made it cheap was
**matching by GEOMETRY, not by street name**: bays carry only `mestopoloz` and roads carry OSM
`name`, reconciled by `norm_street()`'s substring test which already fails on 2 of 49 streets
(TODO #9) and exists only in Python. A polygon needs no reconciliation and is the same test in the
generator and in SQL. One source file, `data/closures.geojson`, feeds both consumers.

What each link does — and the two links that deliberately do NOTHING:
- **`generate_world.py --closures [path]`** drops closed streets from the coverage set and writes
  `worlds/<name>.closures.json`. The street's road runs stay in the `full` graph: it is closed to
  *ground traffic*, not to a drone at 30 m, so it must remain available as a transit or the MST
  fallback starts making straight hops. Opt-in, so the three survey worlds regenerate byte-identical.
- **The controller is untouched**, on purpose. A skipped street's waypoints never enter
  `route.json`, so there is nothing to skip; and `coverage.json` means *flight-time* unreachability
  (a waypoint inside an obstacle disc), so folding a plan-time exclusion into `unreachable` would
  destroy the very distinction the scorer prints it to preserve.
- **`score_occupancy.py`** excludes closed bays from the metrics entirely — neither TP/TN/FP/FN nor
  `uncovered` — and says so on its own line. Counting a closed-and-empty bay as a correct FREE call
  would inflate accuracy with bays the product deliberately hides.
- **Migration `0012` + `web_db._CLOSED`** apply the override at READ time, as a second fragment
  beside `_FRESH`. **`bay_state` is not touched and `process_frame` / the vote are not touched**:
  frames over a closed street are still classified and still write `observation` rows. The closure
  overrides the published *answer*, not the *record* — which is what keeps it falsifiable and lets
  a closure be lifted without re-flying. *Verified with teeth:* 143 observation rows before, 143
  after.
- **`python -m parkdrone_vision.closures load|list|end|rm`** loads the same geojson into Postgres
  and publishes a `bay_delta` for every affected bay, so the map repaints live on every replica
  through machinery that already existed.
- **Both UIs** gained a fourth `BayStatus`. `COLOR` is a `Record<BayStatus, string>`, so the
  compiler refused to build until every branch handled it.

**Known limit, stated rather than hidden:** a closure that expires on its *own clock* pushes no
delta, because nothing runs at that instant to notice; such a bay corrects itself on the client's
next fetch or reconnect. That is the same already-accepted limitation as a `bay_state` row going
stale under `OCCUPANCY_WINDOW_S`.

**Verified 2026-08-25:** three survey worlds byte-identical without the flag; `check_consistency.py`
40 checks (3 new, mutation-tested); route 34 -> 20 waypoints with one street closed; scorer 42 -> 38
classified with 4 excluded and the golden `fmi_block` still **42/42 at 100%**; golden replay 42/42;
full-stack ingest **49 deltas, 42/42**; summary 25/24/0 -> 24/20/5 -> back, reversible; 5 live
WebSocket deltas on lifting.

**Not done, deliberately:** an OSM or municipal closure feed (the hand-authored file is the source
until the shape is proven), and a real per-closure authoring UI — closures are created by CLI today.

---

### 14. The BAY DATA is the bottleneck, not the detector — **OPEN, full write-up in `docs/bay_data_gap.md`**

Found 2026-08-25/26. Two new flights detected **273 cars** and produced **zero occupied bays**. The
detector is fine (it generalises across the light, which was the open question about `merged1`); the
mapped bay geometry is what the cars cannot be matched to.

**Read `docs/bay_data_gap.md` first** — it holds the evidence chain, what it proves, what it
deliberately does not, and the decision waiting. The short version:

- **The vote works** — 0035 is the positive control, where an occupied bay reads 11/13 or 7/7 views.
- **The projection is sound** — 1 m pose self-consistency, OSM buildings landing on real buildings,
  and yaw correction changing almost nothing.
- **Flat ground control separates the flights** (`vision/diag/paint_ground_control.py`, self-tested):
  0035's bays already sit on its paint (9.9% headroom, flat peak), 0075's do not (85.7%, sharp).
- **The gap is systemic.** Once the pipeline stopped discarding unplaced cars (migration `0013`),
  even flight 0035 — the one that *works* — turns out to attribute **473 of 599 detections (79%) to
  nothing**.

**Fix 1 is DONE** (migration `0013`): unattributed detections are recorded rather than silently
dropped, split by whether they are within 2x the assignment radius, since near and far call for
opposite fixes. **Fix 2** (per-street geometry correction) is blocked on the paint extractor, which
misses worn lines under dappled midday canopy. **Fix 3 is a product decision and belongs to the
author** — extend the bay map from our own imagery, score against detected parking rows, or scope
the product to streets Sofiaplan maps correctly. The trade-offs are tabulated in the doc.

**Next concrete step:** hand-label per-bay ground truth on flight 0035's street, where the paint
check is flat, for the first defensible real-world accuracy number.

**Verified against outside references 2026-08-26 — `docs/bay_geometry_verification.md`.** Satellite
imagery (`vision/diag/sat_align.py`, Esri World Imagery at z19 = 0.22 m/px) and an OSM geometric
audit (`vision/diag/bay_audit.py`) both agree with the drone-side evidence, and settle one thing the
paint control could not: **the Sofiaplan POINTS are, as a population, correctly placed** — 95% of
1,698 bays sit 2-7 m from a street centerline, exactly a kerbside offset. The unplaced cars sit in
that same band (median 3.1-5.6 m by flight against the bays' 4.1 m), so they are neither mis-projected
nor near-misses; on 0075 they are in a different row. That removes doubt about the dataset as a whole
and leaves #15 and fix 3 as the real work.

### 15. `data/block_bays.geojson` is STALE — 261 bays (15.4%) are rotated wrong — **OPEN**

Found 2026-08-26, ours not Sofiaplan's. The bay rectangles are **not** source data: `tools/make_bays.py`
draws a 5.4x2.2 m box on each Sofiaplan point, oriented from the nearest OSM centerline. Regenerating
from the committed inputs reproduces **every centre exactly** and rotates **261 bays by >5 deg**, whole
streets at a time (35/111 Кричим, 33/195 Джеймс Баучер, 28/41 Вишнева, 13/13 Хараламби Тачев). On
Плачковица the committed boxes carry bearing -1.6 deg while the centerline 3.8 m away runs at 89.5 —
they were oriented from a road 22 m off on another street, and they lie *across* the carriageway.
All three data files share a 2026-07-04 12:13 mtime, so `get_roads.py` and `make_bays.py` almost
certainly ran in the wrong order once.

**Real-flight occupancy is unaffected** — assignment is by centroid distance and no centroid moves
(measured: identical counts on all three DJI flights under both geometries). **The sim is affected**:
`generate_world.py` paints the rectangle and `classify()` crops it, so 15% of pads are the wrong
piece of ground and every sim accuracy figure was computed against them.

Fixing it is one command and then a lot of re-measurement: regenerate, re-fly both survey worlds,
re-score, restate the numbers in `CLAUDE.md`. Do it deliberately, and add a `tools/check_consistency.py`
check asserting `block_bays.geojson` reproduces from its inputs so it cannot recur silently.

## Hardware

### 12. Powertrain is mismatched — 2300KV motors on 7" props at 4S — **OPEN, do not fly as built**

Found 2026-08-23 while reviewing the thesis hardware chapter. The parts list pairs **RS2205 2300KV**
motors (their own spec line says `Propeller Suggested: HQ 5045`, i.e. 5 inch) with a **Mark4 7 inch**
frame, **7 inch propellers**, **Emax BLHeli 30A** ESCs and a **1500 mAh 4S** pack. The user confirmed
this is the build as it physically exists, not a typo in the list. The frame is fine; the motor/prop/
cell-count combination is not.

**Why it is wrong, in numbers:**
- `2300 KV × 14.8 V ≈ 34 000 rpm` unloaded. Loaded it still wants 20 000+. A 7" prop at 25 000 rpm
  has a tip speed of `0.089 m × 2617 rad/s ≈ 233 m/s ≈ 0.68 Mach` — far outside what such a prop is
  designed for; efficiency collapses and the blade loading gets risky.
- Prop power scales as `P ∝ n³D⁵`, so 5" → 7" at equal rpm is `(7/5)^5 ≈ 5.4×`. The motor's KV is
  "geared" for 4S, so instead of spinning up it sits in a high-torque, high-current regime.
- **The ESCs burn first.** RS2205 on 5045 at 4S already peaks near 25–30 A; on 7" props that is
  40–50 A per motor against a 30 A ESC.
- Then the motors — a 2205 stator is small and cannot shed that heat.
- **Endurance is the other half.** 1500 mAh with 4 motors pulling ~30 A each is ~80C continuous and
  gives roughly **1–2 minutes** of flight, against a 1 km sim route that is **>60 min**.

**A measurement to redo:** the user measured ~17 cm between adjacent motors and ~29 cm across the
diagonal. 17 cm is geometrically impossible with 177.8 mm (7") props — the discs would overlap; a
symmetric X with a 29 cm diagonal gives 20.5 cm adjacent. Measure shaft-centre to shaft-centre.
Mark4 is a stretched X, so the lateral and longitudinal pairs differ and 17 cm may be the lateral
pair.

**Options, in order of cost:**
| option | change | outcome |
|---|---|---|
| zero-cost, immediate | 5" props on the existing motors | Safe, on-spec. Frame is oversized, payload margin stays thin. |
| right for the task | ~1500KV motors (e.g. 2806.5) + 45–50 A ESCs | 7" on 4S behaves; more thrust at lower rpm. |
| for real endurance | low KV + 6S Li-ion 3000–5000 mAh | The standard "7 inch long range" configuration — the only one where surveying a block is realistic. |

A survey platform wants **big props at low rpm**, because that is the efficient regime in hover —
the opposite of the racing logic 2205 2300KV was designed for.

**Thesis consequence (also logged in `docs/final_doc_review.md` §1.17):** a flight time of minutes
against a route of over an hour is a real gap between the simulation and the hardware that the draft
does not acknowledge anywhere. One honest sentence in the hardware chapter or in future work is
stronger than omitting it — a reviewer will do the same arithmetic. Also fix the contradiction at
draft l. 100, which calls the frame "3д принтирана" while the parts list names a commercial Mark4.

---

## Not tasks

- **Angled-bay direction** (left/right of the street) is not in the Sofiaplan
  data at all — a documented data limitation, not something to fix.
- **Cruise speed** is already solved by the corner-aware speed profile
  (`V_MAX` 5.0 with a backward-propagated braking curve).
