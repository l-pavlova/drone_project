# PARKDRONE — open work

Last revised 2026-08-12. Five tracks. Detail that only matters while a task is
being worked lives in the task itself; the *why* lives here so a task can be
picked up cold.

Order agreed 2026-08-12: **#2 → #3 → obstacle track (#4)**. The vision track
(#5, #6, #7) is handled separately.

---

## Obstacle avoidance

### 4. Stage B — steer around the obstacle
Stage D (detect and stop) is **done and verified**: the drone acquires the tower
at 76 m and holds a 12.20 m standoff to ±2 cm for 400+ s. The patrol never
completes — that is stage D by design.

Design decisions already taken:
- **Steer, not climb.** Climbing would break the classifier's 30 m calibration.
- **A yaw bias into the navigator's existing channel** — never a lateral position
  command. A lateral position command saturates while off-heading and tumbles the
  drone; that is a standing controller invariant. The threat *bearing* is already
  computed and logged by the stage D layer, so the input exists.

Test world `fmi_block_obst` carries four deliberate failure modes: `tower` (wide,
head-on), `slab` (offset 7 m, clips the corridor without blocking it), `mast`
(0.9 m wide — the sparse-fan blind spot, and the argument for a Lidar) and `trap`
(concave U — expected to defeat a purely reactive controller, and the argument
for keeping some route memory).

Baseline: `sim/output/fmi_block_obst_baseline/`. Stage D:
`sim/output/fmi_block_obst.stageD-halt/`.

---

## Vision — accuracy *(handled separately)*

### 2. Root-cause the ~0.6 m per-frame projection error — IN PROGRESS
Bay 17685 on `fmi_block` is a false positive from a **single view** at 146 px
eccentricity; the world sits at 97.6% instead of 100%.

**Mechanism established.** Measured with a 1-D brightness profile across each
bay's short axis — no search, so it cannot lock onto a neighbour's line:

- **Within a frame the offset is consistent** (frame 29: three bays at
  +0.14 / +0.08 / +0.14 m, sd 0.029 m) → a per-**frame** pose error, not per-bay.
- **Most frames are fine**, 0.01–0.14 m.
- **Frame 26 — which carries both false positives — is the outlier at +0.64 m.**

Paint then leaks into the core crop and lifts chroma/brightness/std on empty
asphalt past the occupied thresholds (the FP bay reads chroma 12.99 against a
12.7 threshold; a correctly-classified free bay reads 12.34).

**Seven hypotheses refuted — do not re-test:**

| # | Hypothesis | Result |
|---|---|---|
| 1 | Eccentricity/scale error | Affine fit residual 6.81 px vs raw 6.70 px — explains nothing |
| 2 | Generator vs scorer bay geometry | Agree to 2 mm, all 45 bays |
| 3 | Camera not truly nadir | Frame 26 residual tilt 0.0036 rad = 0.106 m, ~⅙ of the error; frame 29 has the *lowest* tilt yet still shows offset |
| 4 | Stale image / temporal lag | Along-track lag median 0.00 m; capture sequence verified — pose and image read on the same step |
| 5 | `ORIGIN` drift | Identical in all three files |
| 6 | Orbit-timeout / hard turning at capture | Frame 26 is 5.68 m off its waypoint, but so are several clean frames — no correlation |
| 7 | Camera-to-GPS mounting offset | A fitted constant explains 36% of variance at 0.35 m, but summing the proto's gimbal chain gives the **real** offset as ~3.8 cm forward / 1.4 cm left / 3 cm down, with GPS and IMU at the robot origin — an order of magnitude too small |

**Important caveat on #7.** Measured half-width comes out consistently 0.7–0.9 px
*under* predicted, which means the profile peak-finder has a systematic bias.
The fitted 0.35 m is therefore most likely an artifact of the measurement, not
physics. **Fix the measurement before trusting any constant-offset fit.**

**Next leads:**
1. Validate the measurement — locate each painted line by its *centroid* over a
   width window rather than by `argmax`, and see whether the residual offsets
   survive.
2. Frame 26 specifically — render several bays in that frame and confirm visually
   whether the *whole frame* is shifted or only that neighbourhood.
3. Eliminate a half-step GPS/physics sampling error (2 cm at 5 m/s — too small,
   but cheap to rule out).

Diagnostics in the session scratchpad: `profile_test.py` (1-D profile, the
reliable one), `solve_offset.py`, `align_fit.py`, `lag_test.py`, `bay_zoom.py`
(magnified overlay — clearest evidence), `pose_outliers.py`.

**Partial fix already applied:** the crop's two shrink axes were coupled through
one uniform `INNER`, so buying clearance from the bay's own paint also ate the
bay's length, which `CORE_LEN` already guarded. They are now independent
(`CORE_W = 0.70`, clearance 0.21 m). `fmi_block` 95.2% → 97.6%,
`fmi_block_4st` unchanged at 100%. `CORE_W` was chosen **on the calibration world
only**; the held-out curve is non-monotonic, so part of that gain is luck.

### 5. Learned classifier v2 — *blocked by #6*
`classify()` is five hand-tuned thresholds calibrated on `fmi_block_4st`. It is
visibly at its limit: bays now separate in the third decimal place (a false
positive at chroma 12.99 against a correct free bay at 12.34), the `CORE_W` sweep
is non-monotonic because a bay flips across a threshold, and the light-pole
parallax case is a physically real occlusion no colour heuristic can fix.

`classify()` is deliberately the only swap point — projection, view selection,
voting and scoring stay unchanged. Labelled crops are already on disk.

**Sequence this *with* scene hardening, not before it.** On the current
uniform-lighting scene a learned model has nothing to beat.

### 6. Shadows, as its own controlled experiment
`castShadows FALSE` is not an aesthetic choice: shadow mapping over a 1.2 km
ground plane paints streak artifacts across the nadir frames. Turning it on as-is
would harden the classifier against a *rendering defect* rather than real
shading. Fix shadow quality first (window-sized ground plane, or a tighter
frustum), verify the artifacts are gone, then measure the accuracy effect **on
its own** so the result is attributable.

The single largest untested hardening lever.

### 7. Reduce capture scatter (~3 uncovered bays)
Timeout arrivals capture up to ~15 m off the waypoint, so ~1% of bays fall
outside every footprint: 3 of 45 on `fmi_block`, 0 of 111 on `fmi_block_4st`.

More interesting than the count: bay 17685 (#2) is decided by a **single** view
while correctly-classified free bays get 2–3. The multi-view majority vote is the
designed defence against one bad look, and it is exactly the thinly-covered bays
that fail — so coverage work also hardens the vote.

Untried: fly higher for a larger footprint (costs resolution, 16 → ~10 px/m at
50 m, and needs the classifier re-verified at that scale); re-aim the capture
yaw; tighten the orbit-timeout bail-out. **Do not tighten `WP_REACH` below 6 m** —
at cruise speed the turn radius is ~4 m and the drone settles into a stable orbit
inside a tighter basin, hanging the patrol forever.

---

## Infrastructure

### 3. A consistency test for constants duplicated across files — **DONE**
`tools/check_consistency.py`, 26 checks, exits 1 on mismatch. Covers the ENU
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

### 1. Score the 1 km survey
Flight restarted 2026-08-12 00:33 on the angled-bay-corrected world: 1593 bays,
728 cars, 1976 waypoints, 19.0 km. When it lands, score it and archive the result
as the world's first clean fixture.

This is the project's first neighbourhood-scale accuracy number, and the first
score of a world containing angled bays **and** 31 bays sitting on green
polygons — neither exists in the two small worlds, so the bays-on-grass effect
should show up here for the first time.

Old partial run (three world builds stale) archived at
`sim/output/fmi_block_1km.2026-08-05-partial/`.

### 9. Street-name matching for the 4 PCA-fallback streets
4 of 49 streets in the 1 km cut have no OSM name match and fall back to a
straight-line PCA bearing: жк Лозенец, Арх. Йордан Миланов, Кръстю Сарафов,
Св. Седмочисленици. The fallback is only correct for *straight* streets.

**Check whether any of the four actually curve before investing** — if they are
all straight this is a non-issue and should be closed as such. Likely cause is
abbreviation/transliteration (Арх. / Архитект, Св. / Свети), not missing OSM data.

---

## Web

### 8. Reconcile the web tier's status, then P6/P7
Untouched for over a week. **First resolve a documentation contradiction**:
`CLAUDE.md`'s status line says Phase 6 (admin dashboard) remains, while the same
file documents `apps/web-admin` as built and running on :5174. Establish which is
true by running `cd web && pnpm quickstart` before planning any work — this is the
same documentation-drift class as the stale fixture.

Then P7 hardening, whose items are already known: put `/api/v1/metrics` and
`/metrics` behind admin auth (currently unauthenticated like every read route);
and note that **single-replica is a correctness requirement, not a preference** —
`jobs.recover()` re-enqueues every queued row with no ownership filter, so two
replicas would double-count the vote. Scaling out needs job claiming, a shared
delta channel and a global cursor first.

Also: the replay golden expectation moved 43/43 → 42/42 tonight and the fixture
is now 97.6%; it moves again when #2 lands. Re-run
`python -m parkdrone_vision.replay fmi_block` to confirm it still matches.

---

## Not tasks

- **Angled-bay direction** (left/right of the street) is not in the Sofiaplan
  data at all — a documented data limitation, not something to fix.
- **Cruise speed** is already solved by the corner-aware speed profile
  (`V_MAX` 5.0 with a backward-propagated braking curve).
