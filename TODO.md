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
- **21 of 97 waypoints are still skipped.** The floor is not the arc radius but
  stage D's 12 m standoff: a waypoint closer than that to a structure cannot be
  flown to at all, so it can never be captured. Getting that coverage back needs
  a different mechanism — capture at closest *legal* approach and flag the
  frame, rather than skipping the waypoint. That changes what the frame means to
  the scorer, so it is a decision, not a tweak. **This is the next task on the
  track.**
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

### 5. Learned classifier v2 — *blocked by #6*
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

### 1. Score the 1 km survey — **the flight is DEAD, not pending**
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


### 9. Street-name matching for the 4 PCA-fallback streets
4 of 49 streets in the 1 km cut have no OSM name match and fall back to a
straight-line PCA bearing: жк Лозенец, Арх. Йордан Миланов, Кръстю Сарафов,
Св. Седмочисленици. The fallback is only correct for *straight* streets.

**Check whether any of the four actually curve before investing** — if they are
all straight this is a non-issue and should be closed as such. Likely cause is
abbreviation/transliteration (Арх. / Архитект, Св. / Свети), not missing OSM data.

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

Then P7 hardening, whose items are already known: put `/api/v1/metrics` and
`/metrics` behind admin auth (currently unauthenticated like every read route);
and note that **single-replica is a correctness requirement, not a preference** —
`jobs.recover()` re-enqueues every queued row with no ownership filter, so two
replicas would double-count the vote. Scaling out needs job claiming, a shared
delta channel and a global cursor first.

### 10. A re-flight should ingest itself — no manual clear
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

Until this lands, `pnpm clear <area>` — or `pnpm quickstart --clear --fly
<world>` — is the documented way to re-fly and be scored.

---

## Not tasks

- **Angled-bay direction** (left/right of the street) is not in the Sofiaplan
  data at all — a documented data limitation, not something to fix.
- **Cruise speed** is already solved by the corner-aware speed profile
  (`V_MAX` 5.0 with a backward-propagated braking curve).
