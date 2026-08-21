# PARKDRONE — open work

Last revised 2026-08-20. Five tracks. Detail that only matters while a task is
being worked lives in the task itself; the *why* lives here so a task can be
picked up cold.

Order agreed 2026-08-12: **#2 → #3 → obstacle track (#4)**. #2 and #3 are done;
the vision track (#5, #6, #7) is handled separately.

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

### 5. Learned detector — groundwork BUILT 2026-08-21, zero-shot baseline measured
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

**Next:** dataset volume is bounded by flight length (~0.9 cars/frame), so more data means more
flights; fine-tune from aerial-pretrained weights if the HF download blocker is cleared (CLAUDE.md
§2b), else from the Ultralytics path that already works.

### 5b. The old framing — learned *classifier* v2 — *blocked by #6*
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

**Sequence this *with* scene hardening, not before it.** On the current
uniform-lighting scene a learned model has nothing to beat.

### 6. Shadows — generator side BUILT 2026-08-21, flight pending
`castShadows FALSE` is not an aesthetic choice: shadow mapping over a 1.2 km
ground plane paints streak artifacts across the nadir frames. Turning it on as-is
would harden the classifier against a *rendering defect* rather than real
shading. Fix shadow quality first (window-sized ground plane, or a tighter
frustum), verify the artifacts are gone, then measure the accuracy effect **on
its own** so the result is attributable.

The single largest untested hardening lever.

**Built (not yet flown):** `--shadows` and `--sun AZ,EL` in `generate_world.py`, opt-in so the three
survey worlds regenerate byte-identical (verified). The quality fix is in the same flag: the ground
plane shrinks from `2*WINDOW+200` to `2*WINDOW+40`, because a directional light spreads one shadow
map over the scene extent and the plane is the extent. `fmi_block_4st_sh` is generated and is a
clean A/B — same `ground_truth.json` and `route.json` as `fmi_block_4st`, only the light differs.

**Next, in order:** fly it (Webots is busy with the 1 km survey until ~09:00), LOOK at the frames
for artifacts, and only then score. Prediction on record: `T_BRIGHT_LO/HI` (82-102) is an absolute
asphalt tone envelope, so shaded free bays fall below 82 and read as occupied — false positives on
the shaded side of the street. Then give the heuristic its fair form (normalise the bay core against
surrounding asphalt in the same frame) before any model is compared against it.

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

### 1. Score the 1 km survey — **FLYING as of 2026-08-20 23:xx**
Relaunched 2026-08-20; it resumed from frame 87 and is running headless
(`worlds/fmi_block_1km.wbt`, log `sim/output/fmi_block_1km.run.log`). At ~6
captures/min the remaining ~1840 waypoints are roughly 5 h of wall clock. It was
paused once mid-session to free Webots for the standoff-capture flights and
resumed from disk, which is the documented behaviour and cost only the pause.
When it lands, score it and archive the result as the world's first clean
fixture.

*(Original note, kept because it is the reason there was nothing to debug:)*

**The flight was DEAD, not pending**
`sim/output/fmi_block_1km/poses.json` stopped at **87 of 1976 frames** on
2026-08-17 15:29 — stopped by hand (confirmed with the author 2026-08-20), not
by a crash or by the obstacle track's `taskkill`. So there is nothing to
diagnose: this task starts by simply *relaunching* it. A folder with existing
captures makes the controller resume, so a long flight can be picked up rather
than restarted.

Flight parameters on the angled-bay-corrected world: 1593 bays, 728 cars, 1976
waypoints, 19.0 km. When it lands, score it and archive the result as the
world's first clean fixture.

This is the project's first neighbourhood-scale accuracy number, and the first
score of a world containing angled bays **and** 31 bays sitting on green
polygons — neither exists in the two small worlds, so the bays-on-grass effect
should show up here for the first time.


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

Still open in P7:
and note that **single-replica is a correctness requirement, not a preference** —
`jobs.recover()` re-enqueues every queued row with no ownership filter, so two
replicas would double-count the vote. Scaling out needs job claiming, a shared
delta channel and a global cursor first.

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

---

## Not tasks

- **Angled-bay direction** (left/right of the street) is not in the Sofiaplan
  data at all — a documented data limitation, not something to fix.
- **Cruise speed** is already solved by the corner-aware speed profile
  (`V_MAX` 5.0 with a backward-propagated braking curve).
