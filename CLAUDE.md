# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this project is

PARKDRONE: an autonomous drone that surveys **parking-space occupancy** (occupied vs. free) over a Sofia city block. `docs/PARKDRONE_FULL_MASTER_PLAN*.md` describes the full hardware vision (Pixhawk + Raspberry Pi 4 + Coral USB + IMX219 camera, GPS nav, obstacle avoidance, AI detection), but the **active work is in simulation**: fly a lawnmower patrol in Webots over a real-data parking block, capture downward camera frames, and (next) detect cars and score occupancy against ground truth.

There is no build system and no test suite — the code is standalone Python scripts plus one Webots robot controller. "Running tests" means running the data scripts or the sim headless and inspecting their output.

## Pipeline & commands

Three stages. Stages 1–2 are built and working; stage 3 (`vision/`) is not started.

### 1. GIS data prep (run from `tools/`, writes to `data/`)
```bash
python build_demo_area.py ["Лозенец"]     # clip spaces_25.geojson to an OSM district boundary
python cut_block.py "<address or lat,lon>" 200   # cut a square block (200 = half-size m) -> block_spaces.geojson
python make_bays.py                        # space POINTS -> oriented bay RECTANGLES -> block_bays.geojson
python get_roads.py                        # OSM street centerlines for the block (Overpass) -> block_roads.geojson
python get_areas.py                        # OSM building footprints + green areas (Overpass) -> block_areas.geojson
```
`spaces_25.geojson` (31,705 parking-space points) and `zones_34.geojson` are the source datasets from the Sofiaplan API; see `data/README.md` for schema and field meanings (Bulgarian property values).

### 2. Webots simulation (run from `sim/`)
```bash
python generate_world.py [half_m] [occ_frac]   # default 75 0.5 -> worlds/fmi_block.wbt + worlds/ground_truth.json
python generate_world.py 75 0.5 fmi_block_closed --closures   # drop closed streets from the route
```
This reads `../data/block_bays.geojson` (plus `block_roads.geojson` and `block_areas.geojson` if present), projects to local metres, and emits the world (ground, OSM streets as Webots `Road` protos, painted bays, real car models — 7 vehicle Simple protos — on a known-occupancy subset, scenery, follow-drone viewpoint) plus `ground_truth.json` (bay_id -> occupied). Car/model randoms come from a separate `random.Random(7)` stream and scenery cosmetics from a third (`random.Random(11)`) so `ground_truth.json` stays stable — after any change here, regenerate all three worlds and check `ground_truth.json`/`route.json` are byte-identical.

**`--shadows` / `--sun AZ,EL` — the scene-hardening experiment (built and FLOWN 2026-08-21).**
Opt-in like every other world flag, so the three survey worlds regenerate byte-identical and every
accuracy number on record stays valid. `--shadows` sets `castShadows TRUE` on the `DirectionalLight`
**and shrinks the ground plane** from `2*WINDOW+200` to `2*WINDOW+40`: a directional light spreads
one shadow map over the whole scene extent, and the ground plane *is* the extent, so the 1 km world
was handing one map a 1.2 km square — which is exactly what painted the streak artifacts that made
`castShadows FALSE` necessary. **The shrink works:** flown frames show car, building and light-pole
shadows cleanly with no streaks, checked by eye before any number was computed. `--sun AZ,EL` takes a
compass azimuth (where the sun *is*, clockwise from north) and an elevation, and emits the direction
the light travels; without it the direction is the literal `0.4 0.5 -1` every world on record used
(~60 deg elevation), so nothing shifts.

**Measure the two flags SEPARATELY — they are not one experiment.** `--sun` changes *global
irradiance* as well as shadow length, and on a horizontal ground plane that term dominates. The 2x2,
all four worlds sharing `fmi_block_4st`'s byte-identical `ground_truth.json`/`route.json` and all
flying 97/97:

| | shadows OFF | shadows ON |
|---|---|---|
| default sun | **100.0%** `fmi_block_4st` | **89.2%** `fmi_block_4st_shd` (FP=12) |
| `--sun 135,25` | **44.1%** `fmi_block_4st_sun` (FP=62) | **44.1%** `fmi_block_4st_sh` (FP=62) |

Recall is **100% in every cell** — no car is ever missed; every loss is a false positive on a free
bay. A 25 deg sun cuts Lambert irradiance to ~0.48 of the default, dropping *every* free bay
(sunlit ones included) from `core_brightness` 85 to 72, under `T_BRIGHT_LO` 82 — so the low sun alone
accounts for the whole collapse and the shadow flag adds nothing on top of it. Shadows' own distinct
contribution is the 12-bay cell: a *lower tail* (`bright` 53, `dark_frac` 1.00) on genuinely shaded
bays, which is the physically real, harder case.
**The mechanism is NOT the brightness envelope alone** — that was the first reading and
`vision/diag/classifier_ablation.py` disproved it. On all 62 false positives `chroma` AND `bright`
fire together (free-bay `core_chroma` 11.00 -> **14.00** against T=12.7; `core_brightness` 84.7 ->
72.3 against a floor of 82), so dropping either alone still scores 44.1% with the same 62 FP. Chroma
**rises** as the light dims, because a shallower sun means proportionally more of each surface's
light comes from the tinted ambient sky — a change in the light's SPECTRUM, which no intensity
normalisation can undo.
**And `chroma` is the only load-bearing test** (dropping it costs `fmi_block_4st` 100% -> 89.2%;
dropping `dark`/`paint`/`std` costs nothing), so every colour normalisation attacks the signal
itself: grey-world correction takes the golden world to 93.7% with 7 missed cars. Measured across
all six worlds, **nothing recovers the lighting worlds without breaking the calibrated ones** — best
sun result 93.7% at the cost of `fmi_block_4st` 100 -> 92.8. Two structural reasons: the free-core
margin to the floor is **3.5%** (85 vs 82), and the scene is three flat tones (ground 121 = 56% of
pixels, road 50, pad 85) so a local ring reference is bimodal and a frame-wide one is
composition-dependent. **Conclusion: there is no fair form of this heuristic that survives a lighting
change** — see TODO #6. That is evidence FOR the learned detector, and it says what to train it on:
varied illumination, which `--sun`/`--shadows` now generates cheaply with exact labels.

**`--closures [path]` — a street that is CLOSED (TODO #13, built 2026-08-25).** Opt-in like every
other world flag; without it the three survey worlds regenerate byte-identical. A closure is the
first authoritative fact in this project that is **not derived from the camera** — roadworks, a
market, an accident — and it overrides the answer: an empty bay on a closed street is not available
parking, however clearly the detector saw that it was empty.
**Matched by GEOMETRY, never by street name.** The obvious key is the name and it is the wrong one:
bays carry only `mestopoloz`, roads carry OSM `name`, and `norm_street()`'s substring reconciliation
already fails on 2 of the 49 streets in the 1 km cut (TODO #9) — and the web tier would need a third
copy of that rule, in SQL. A polygon needs no reconciliation, is the same test in the generator
(`point_in_poly` on the bay centre) and in the server (`ST_Intersects` on `bay.centroid`), and can
cover half a street or a square. `tools/check_consistency.py` asserts the two agree (checks 8a–c).
`data/closures.geojson` is **one source file with two consumers** — this flag and
`python -m parkdrone_vision.closures load` — so the sim and the map cannot disagree about what is
closed, which is the exact failure the feature exists to prevent.
Closed bays are dropped from the coverage set but their **road runs stay in the `full` graph**: the
street is closed to *ground traffic*, not to a drone at 30 m, so it must remain available as a
transit/deadhead or the MST fallback starts making straight hops. Closed bays keep their
`ground_truth.json` entry and their RNG draw — nothing about car placement moves — so a closure
world's `ground_truth.json` is exactly the same file it would otherwise be. Output is a sidecar,
`worlds/<name>.closures.json` (active closures + the `bay_id -> closure_id` map), written only when
the flag is passed; a sidecar rather than a field on `route.json` because that file is a bare
`[[x, y], ...]` array and the controller's loader, its resume path and the 1:1 frame↔waypoint
numbering all depend on that shape.
**The controller is deliberately untouched.** A skipped street's waypoints never enter `route.json`,
so there is nothing for it to skip; and `coverage.json` means *flight-time* unreachability (a
waypoint inside an obstacle disc), so folding a plan-time exclusion into `unreachable` would destroy
the distinction the scorer prints it to preserve. Measured on `fmi_block`'s window: 34 → **20
waypoints**, 2 → 1 covered streets, deadhead still 0 m.

**Scenery** (from `block_areas.geojson`): OSM green areas triangulated by ear clipping and painted at **z = 0.005** — above the ground, *below* the roads (0.01+), so they can never cover a bay pad (0.04–0.06); OSM building footprints as `SimpleBuilding` protos at real heights (`building:levels`, else `height`, else 4 floors), kept/dropped **whole** by centroid (clipping a footprint can break the proto's roof triangulation) while greens are Sutherland-Hodgman **clipped**; `StreetLight` poles walked along the centerlines at adaptive spacing, `on FALSE` (the proto ships a 1000 m-radius SpotLight that would shift the daylight the classifier is calibrated against) and never placed on a bay. Buildings and light poles get **no bounding object** unless you pass `--collide` — the controller flies a fixed 30 m with zero obstacle logic, so collision geometry would crash it; Webots range sensors only see nodes that have one, so that flag is the obstacle-avoidance stage's entry point. Either way the generator warns about structures reaching 30 m within 10 m of the route and writes them to `worlds/<name>.hazards.json`. `--chase` swaps the GUI viewpoint from the trailing "Tracking Shot" to a ride-along "Mounted Shot" (viewing only — it never touches the drone's own camera or any captured frame); it is a flag rather than a hand edit because hand edits to a `.wbt` are silently lost on the next regeneration. `DirectionalLight` has `castShadows FALSE` — shadow mapping paints streak artifacts on the road/ground in the nadir frames. Then run the world (see "Running Webots" below). The route (`worlds/route.json`) is an open-path rural-postman walk of the OSM street centerlines of every street that has bays: disconnected coverage components are joined by shortest road transits (MST), odd-degree nodes are evened out with a minimum-weight matching (exact blossom if `networkx` is installed, stdlib fallback otherwise; two virtual endpoints make it an open path whose start is the endpoint nearest the origin), then a Hierholzer Euler walk flies every coverage edge once with deadheads only along the matched repeats — waypoints every 10 m because the camera footprint at 30 m is only ~25×15 m. The controller `controllers/parkdrone/parkdrone.py` takes off to 30 m, flies that route (square-lawnmower fallback if route.json is missing), and writes `output/<survey_area>/frame_###.png` + `output/<survey_area>/poses.json` at each waypoint (plus timed diagnostic `snap_###.png`). Worlds are per-scale file sets: `generate_world.py [half_m] [occ_frac] [name]` writes `<name>.wbt` + `<name>.route.json` + `<name>.ground_truth.json` (default name `fmi_block` keeps legacy `route.json`/`ground_truth.json`); the `.wbt` passes its route file to the controller via `controllerArgs`, which also keys the output subfolder. E.g. the 1 km world: `python generate_world.py 500 0.5 fmi_block_1km`. The local-metre frame is pinned by `ORIGIN` in `generate_world.py` — do NOT let it drift when re-cutting data at other sizes.

**Bay paint is MERGED geometry, not per-bay nodes** (2026-08-11). Each bay used to emit 5 `Solid`s
(an asphalt pad + 4 outline boxes); they are now accumulated into exactly **two `IndexedFaceSet`s**
(`bay_pads`, `bay_lines`). A Webots `Solid` costs ~0.145 MB of RSS however trivial its geometry, so
at 1593 bays the 1 km world was spending ~1.1 GB on flat rectangles — and, contrary to
`docs/webots_1km_performance.md`'s prediction of "~0 s", **~115 s of load time** (~14 ms per Solid;
the doc's probe measured Solids in an otherwise-empty world where that vanished into the noise
floor). Measured on `fmi_block_1km`: load **282.6 s → 167.6 s**, peak RSS **6359 → 5268 MiB**,
top-level Solids **8379 → 416**. `DEF`/`USE` is NOT the alternative — it was measured at only 9%,
because Webots already shares primitive meshes via `WbTriangleMeshCache` and the residual cost is
the *node*. Two invariants make this safe and must hold if it is ever touched: the merged quads sit
at the **top face** of the boxes they replace (pads z=0.090, lines z=0.101 — *not* the box centres,
or the paint shifts 10 mm and reopens the coplanar-flicker fight in `Z_LAYERS`), and each quad is
wound **counter-clockwise** so its normal faces +Z and the nadir camera (`rect_corners` walks
clockwise, so `quad_mesh` reverses it). Nothing addresses a bay by scene-node name — the classifier
works from `block_bays.geojson` + `poses.json` — but if that ever changes, `--bay-solids` restores
the old per-bay emission and reproduces the pre-2026-08-11 generator **byte-for-byte**. Verified
accuracy-neutral by an A/B of 4 flights per variant on `fmi_block` (95.2% at the time, identical confusion
matrix, 8/8 runs) and 1 each on `fmi_block_4st` (100% both).

**`--lowpoly` — proxy box cars, big worlds ONLY.** A "Simple" vehicle proto is not low-poly
("Simple" = no physics/joints/interior; the exterior mesh is still full detail, ~20–30k triangles),
and costs a measured **5.1 MB of RSS and 0.136 s of load each** — at 727 cars, ~3.7 GB and ~99 s,
the biggest single line item in the 1 km world. `--lowpoly` swaps the proto for a hand-built
`Solid` (body + cabin + windscreen + 4 wheel boxes) and drops the vehicle `EXTERNPROTO`s.
Everything upstream — every `rng_cars` draw, the model choice, `CAR_DIMS`, the fit-and-fallback
loop that can free a bay and rewrite `gt` — is deliberately untouched, so `ground_truth.json` and
`route.json` come out **byte-identical to the proto-car world of the same size** (verified). It is
opt-in and changes nothing unless passed.
**It changes what a car LOOKS like, so it changes what the accuracy number MEANS.** Measured on
`fmi_block_4st_lp`: **99.1%** (TP=48 TN=62 FP=0 **FN=1**) vs 100% for the proto-car `fmi_block_4st`
(98.2%/FN=2 before the 2026-08-20 camera-model fix). The miss is a *dark* car (bay 17607,
`brightness` 68, `core_std` 10, `core_dark_frac` 0.00) — a
uniform dark box on dark asphalt looks like empty tarmac, because it has none of the panel gaps,
glass or under-car shadow `classify()` leans on. So a box world is **harder** for dark vehicles, not
easier as `docs/webots_1km_performance.md` originally guessed. **Never pass `--lowpoly` for
`fmi_block` or `fmi_block_4st`** (the calibration world and the golden fixtures), and report
`fmi_block_1km_lp` as a coverage/logistics result, never an accuracy one.
Worlds: `fmi_block_1km_lp` (`generate_world.py 500 0.5 fmi_block_1km_lp --chase --lowpoly`) loads in
**57 s / 1663 MiB** vs 282.6 s / 6359 MiB for the original `fmi_block_1km` — 4.9× faster, 3.8×
smaller. `fmi_block_4st_lp` exists only as the evidence for the accuracy claim above.

**Webots runs are deterministic.** Re-flying the same world with the same controller reproduces
frames, poses and scores exactly (8/8 identical runs). So a changed accuracy number always means a
changed *world or controller*, never noise — and conversely, a stale archived flight cannot be
compared against a fresh one. This is why both golden fixtures were re-flown on 2026-08-11.

**`fmi_block_obst` — the obstacle-avoidance test world** (`python generate_world.py 130 0.5 fmi_block_obst --collide --obstacles --chase`). It exists because the only real world with anything to hit at 30 m is `fmi_block_1km`, which took >10 min to load when this world was built (2.8 min headless since the bay-paint merge), so the avoidance edit/run/observe loop needed a fast world with a deliberate conflict. Same window/route/bays as `fmi_block_4st`, plus `--obstacles`: four synthetic structures anchored to fractions along the finished route, each tall enough to reach cruise altitude, each a *different* failure mode — `tower` (45 m, head-on, wide), `slab` (38 m, offset 7 m so it clips the corridor without blocking it), `mast` (40 m but 0.9 m across — the sparse-DistanceSensor-fan blind spot, the case that argues for a Lidar) and `trap` (34 m U opening toward the drone — the concave deadlock that pure reactive avoidance circles inside forever; expect it to fail first). Placement is deterministic and nudged forward past any bay it would cover (a box on painted tarmac is a ground-truth bug) and enforces 30 m between structures, because a postman route doubles back and two anchors 12 waypoints apart can land 15 m apart. They land in `<name>.hazards.json` next to the real buildings, flagged `"synthetic": true`, so a controller arming sensors off that list needs no special case. `--obstacles` changes nothing unless passed — the three survey worlds regenerate byte-identical. **Baseline (verified 2026-08-09):** the obstacle-blind controller flies into `tower` at wp12 and stops there, 13 frames in; archived at `sim/output/fmi_block_obst_baseline/`.

**How the whole avoidance layer works, in pseudocode with the reasoning:
`docs/obstacle_avoidance.md`.** The sections below stay as the constants-and-failures record.

**Obstacle layer, stage D (detect & stop) — done, verified.** `--collide` also mounts a **9-ray `DistanceSensor` fan** (±40°, 80 m, `type "laser"` so Webots draws the beams) in the Mavic2Pro's `bodySlot`; survey worlds get none, `getDevice` returns `None`, and the whole layer switches itself off. `DS_N`/`DS_SPREAD_DEG`/`DS_RANGE` are duplicated in `generate_world.py` and `parkdrone.py` — keep in step, same convention as `ORIGIN`. **The layer emits only a speed limit into the existing `v_des` channel** — never a position target — so the stabilizer sees nothing new and every controller invariant survives by construction; stage B's yaw bias will use the same principle (the threat *bearing* is already read and logged for it). Frames captured while it is in control get `"avoiding": true` + `"obstacle_m"` in `poses.json`, so the scorer can exclude them rather than silently mis-score. Telemetry goes to `output/<area>/flight_log.csv` (line-buffered, one row/s) because Webots' stdout is routinely lost and the interesting seconds fall *between* waypoint captures.

Five things had to be right, each found by a failed run — do not regress them:
- **80 m sensor range is a dynamics spec, not a perception one.** `A_BRAKE` is ~0.22 m/s² (`TILT_MAX` caps tilt at ~2°, no drag in sim), so stopping from `V_MAX` 5 m/s takes `v²/2a` ≈ 57 m. A 40 m rangefinder cannot stop this aircraft at all.
- **Gate returns by bearing/corridor** (`OBST_BEARING` 20°, `OBST_CORRIDOR` 8 m). Braking on the raw closest return made the ±40° rays pick up scenery off to the side, and the minimum then *flipped* between targets so the limit kept rising — the drone accelerated back to cruise mid-brake and hit the tower at 2 m/s.
- **Ratchet the limit down only** (`obst_cap`), so a momentarily lost return can't hand cruise speed back.
- **Charge a reaction allowance** (`OBST_REACT` 3 s of travel) against the range. The bare curve lets a drone strictly below it keep accelerating, which is only true for a vehicle that can brake instantly; this one took 25 m to reverse a trend and never caught up.
- **Brake with a saturated gain** (`K_VEL_BRAKE` 2.0 vs `K_VEL` 0.4) and **station-keep on a latched point** when stopped (`K_HOLD`, `HOLD_V_MAX` 0.3 m/s). Proportional braking fades as speed→target (measured 0.133 m/s², below the planned 0.15) and pure damping cannot null a steady hover drift — the drone stopped correctly at 12.7 m and then crept 13 m into the tower over 95 s. The hold converts position error to a *clamped velocity target*, which is why it does not reintroduce the lateral-position-command tumble.
- Subtle one: `avoiding` must be `v_obst <= v_des` **and** a threat tracked. With a strict `<` the layer switched itself off exactly when stopped (the navigator's own `v_des` is 0 while yawing), so the hold never latched — two runs came out identical to four significant figures before this was spotted.

**Verified result:** the drone acquires `tower` at 76 m, brakes, and **halts at a 12.20 m standoff, holding it to ±2 cm for 400+ s** at 30.00 m altitude; 7 of 11 frames carry `avoiding`. The patrol does not finish — that is stage D by design. Archived at `sim/output/fmi_block_obst.stageD-halt/`.

**Obstacle layer, stage B (steer around) — first working version, verified 2026-08-19.** Steering,
not climbing, so the classifier's 30 m calibration is never broken. Like stage D it writes only into
channels the stabilizer already has — the aim **heading** (`yaw_err`) and the speed target — and
never a lateral position command, so every controller invariant survives by construction. On
detecting a gated return between `STEER_NEAR` (16 m) and `STEER_ENGAGE` (45 m) that lies within
`CLEAR_R` of the route, it commits a **detour**: pick the side with more room from the ray fan
(`open_side`, scored on each side's *minimum* range — one blocked ray is what a collision is), then
fly the tangent to a remembered obstacle, `psi_des = bearing(T) ± (asin(R/d) + margin)`, capped at
`STEER_V` 2 m/s. Waypoints buried inside the obstacle are **skipped and recorded** — a declared
coverage gap is a correct survey result, an endless orbit is not. Frames captured during a detour
get `"steering": true` in `poses.json` alongside stage D's `"avoiding"`; both mean *off the
calibrated path, exclude rather than score*. `sim/analyze_stage_b.py` reports a run from disk.

**Verified result:** the drone acquires `tower` at 45 m, turns, rounds it in a **21 s detour at
2.54 m/s**, skips the 4 waypoints inside it, and **resumes the route on the far side** — reaching
**wp20 of 97 with 17 frames**, where stage D halted at wp12 with 11 and the blind baseline flew into
it at wp12. Closest approach over the whole flight is **12.16 m**, i.e. the standoff holds. It stops
on the route's *second* pass through the same tower, which is stage B's honest limit today.
Archived at `sim/output/fmi_block_obst.stageB-pass/`.

Six things had to be right, each found by a failed run — do not regress them:
- **The threat is remembered as a growing DISC, not a point.** Refreshing a point to the latest
  return makes the memory *follow the drone*: on a wide structure the nearest face point slides
  around it as the drone circles, so the threat is permanently "ahead" and the manoeuvre orbits —
  measured as a full 150 s lap of the tower. Freezing a single point instead under-clears: the arc
  misses that one face by `CLEAR_R` while the rest of the 8 m-wide tower still juts into the path,
  and the drone re-detects at 13 m. A bounding disc that only ever grows has neither failure. It
  grows only by **contiguity** (`STEER_MERGE` 12 m from the surface, capped at `STEER_GROW`): a
  loose "within 40 m" window pulled unrelated buildings in and inflated the disc until it swallowed
  waypoints 30 m clear of the tower.
- **The leave condition is Bug2's, not a geometric one.** "Swept 110° around it" and "it is abeam"
  both release while the goal is still on the *far* side, so the drone turns straight back into the
  obstacle, re-engages, and wanders (measured: 60 m off-route, re-detouring the same tower ~10
  times). Release only when the drone is **measurably closer to the waypoint than when it committed**
  *and* can fly straight at it clear of the disc. (a) is what makes it terminate.
- **Suppress the stop-and-turn latch while steering.** `hold_stop` exists for route reversals. Mid
  detour the drone is deliberately 20 m off-route on a tangent, so the bearing to the next waypoint
  is always wildly off and the latch fires on essentially every capture — pinning `v_des` to 0 until
  ground speed drops below `HOLD_SPEED`, which it never does, because `K_VEL` is proportional and the
  braking fades (the same mechanism stage D hit). Every detour crawled at a pinned 0.37 m/s and ran
  itself into the timeout.
- **Do NOT suppress the station-keep latch while steering.** The mirror image, and the dangerous one:
  with it suppressed, a detour whose obstacle curve had already gone to zero drifted **11.94 m →
  5.84 m over 110 s** at 0.07 m/s, straight into the tower. The hold only writes `v_des`/`v_lat_des`
  — the detour's yaw command is untouched — so it does not freeze the manoeuvre; if the threat stays
  inside the close-range corridor whatever the heading, there is genuinely no way round from here and
  a safe stop is the right answer.
- **Do not start a detour inside `STEER_NEAR`.** Turning needs room ahead; inside the standoff the
  only correct answer is stage D's brake. Without the floor the controller committed to a detour with
  a return **5.4 m** off the nose.
- **Giving up is per-threat, not global.** A timed-out detour blacklists that disc and reverts to
  stage D *there*. One unsolvable structure — `trap` is built to be exactly that — must not cost the
  drone its avoidance for the remaining 90 waypoints.

**Clearance is one constant, and the drone escalates rather than gives up.** `CLEAR_R` is the
clearance flown past the obstacle's *measured* surface; the arc radius, the release test, the skip
radius and stage D's standoff against the committed obstacle all derive from it, so they cannot
disagree. It is **tight by default (5 m)** because a wide berth is what costs coverage and what
makes the drone visibly wander: at 18 m the patrol skipped 28 of 97 waypoints and flew a mean 21.3 m
off its own route, against 21 and 9.6 m at 5 m clearance (76 frames vs 69). Reliability is bought
back not by widening every detour but by **retrying the awkward one**: a detour that times out
doubles the clearance *for that obstacle* and tries again (`STEER_ESCALATE`, 3 levels), so the common
case stays tight and only the difficult structure gets the wide berth.
Three things make a tight arc flyable, and each was a failed run:
- **Speed is limited by stopping distance, not just by turn radius.** Flying an arc at the fastest
  the radius allows (`sqrt(A_LAT*r)`) means staying stoppable requires clearance ≥ 4.9× the
  obstacle's own radius — ~16 m around this 8 m tower, which is exactly the wide, wandering
  behaviour. Adding `sqrt(2*A_BRAKE_OBST*CLEAR_R)` inverts it: fly slowly and close is safe.
- **The engage corridor must be at least stage D's standoff.** Narrower (8 m vs 12 m) opens a band of
  structures that halt the drone but never qualify for a detour; the patrol froze 11.78 m from one.
  And symmetrically, **anything that has actually stopped the drone is detourable regardless of the
  route** — stage D brakes on bearing, stage B routes on the plan, and a structure beside the route
  that the nose happens to point at froze the drone at 10.47 m with the route clear.
- **A stopped drone may start a detour inside `STEER_NEAR`.** The floor exists because turning needs
  room to stop; a drone at zero ground speed has already stopped. Without the exception, flying close
  deadlocks routinely — the drone ends up inside the floor with no detour committed and stage D
  station-keeps forever.
**0.5 m clearance was tried and does not work** — it ends 1.3 m from the tower. Each detour releases
as soon as it is marginally past, the drone re-aims at a route running through the structure, and it
ratchets in: 45 → 30 → 19.6 → 8.4 → 4.3 → 1.3 m. Independently, the 9-ray fan is 10° apart, so at
5 m range the gaps *between* rays are 0.9 m — wider than the clearance being asked for. That is the
`mast` case arriving early, and it is a sensor argument, not a tuning one.

**Verified result (2026-08-19, `sim/output/fmi_block_obst.stageB-tight/`): the patrol COMPLETES** —
wp96 of 97, 76 frames, and all four structures passed, `trap` (the concave U built to defeat a
reactive controller) included. Closest approach 6.21 m. The earlier wide-berth run is kept at
`sim/output/fmi_block_obst.stageB-complete-r18/` for comparison.

**A controller exception looks like a drone that simply stops flying.** Webots keeps running when the
Python controller dies, and the filtered console shows nothing: the symptom is a `flight_log.csv`
whose last timestamp stops advancing while `webots-bin.exe` is still alive at full memory. Cost an
hour when `steer_off` entries grew a field and one stale 3-name unpack was left behind. Check the log
mtime against the process before assuming the drone is merely stuck.

**Survey worlds are unaffected, verified by A/B not by argument.** Stage B touches the shared
navigator, so `fmi_block_4st` was flown twice — once with this controller, once with the committed
one — from empty output directories. Both complete 97/97 waypoints and the trajectories agree to
**under 0.1 mm**. Note that is *agreement*, not
bit-equality: re-launched Webots runs match to ~1e-6, so comparing poses with `==` reports every
frame as different and means nothing — compare with a tolerance.

**Stage B, orbit fix — a detour must be a way PAST the obstacle, not a lap of it (2026-08-20).**
The tight-clearance patrol completed, but watching it showed the drone flying **full circles** around
structures instead of rounding them: measured in `sim/output/fmi_block_obst.stageB-tight-orbits/`,
ten detours of which four swept **188°, 231°, 293° and 338°** — the last two around `mast`, a 0.9 m
pole, at ~15 m radius, 70 s and 81 s each. Two causes, both about the question "is it in my way?"
being asked differently at commit and at release:
- **It committed for things it was not yet flying at.** The commit probed the route over the full
  `STEER_HORIZON` (70 m), so a structure blocking the route two legs later took the drone off a
  waypoint 12 m off its nose. The tangent law then flew it *to* that structure, and the leave
  condition — which requires closing on the current waypoint — could not fire, because that waypoint
  now lay behind. The only way to satisfy it was to come all the way round. The commit gate now uses
  `STEER_LOOK` (12 m past the current waypoint, so the window is "this leg and the next"): the
  obstacle has to be between the drone and where it is going, which makes rounding it *progress* and
  makes the detour terminate in ~180°.
- **It released while the obstacle still lay across the next leg.** The release only tested the run
  to the current waypoint, so a detour committed at 43 m let go a few metres later and re-committed a
  second afterwards — five engage/release cycles closing on `mast`, each nudging the approach further
  off line, and a 270° orbit at the end of it. Release now also requires the *route ahead* (same
  window) to be clear of the disc, or the disc to be behind.

`fmi_block_4st` was re-flown against this controller as usual: 97/97 waypoints and poses **identical
to the golden fixture to 0.000000 m** — every line of the fix sits inside the `if t_now` / `detour is
not None` branches, and a survey world has no ray fan to produce a return.
And a bound, because reactive avoidance can always find a new way to circle: `STEER_ORBIT` abandons a
detour that has swept **270°**, suppressing re-commit against that disc for `STEER_COOL` (45 s) and
handing the drone back to its route with stage D still protecting it. It deliberately does **not**
escalate the clearance — an orbit is the one failure a wider berth makes worse. **270° and not less:
200° was tried and the run ended in a CRASH** — bailing out earlier leaves the drone on the near side
with the structure still across its route, it turns straight back at it, and at wp32 it clipped a
building the ray fan had last seen 8 m away.
**Verified result (`sim/output/fmi_block_obst.stageB-noorbit/`): the patrol still completes 97/97
with all four structures passed, in 6 detours instead of 18, none of them a circle** (largest sweep
238°, and that one is the tower the route re-enters three times), **79 frames** (76 before) in
**1599 s**, closest approach **6.55 m**.

**Stage B, standoff capture — an unreachable waypoint is not unseen ground (2026-08-20).**
A waypoint inside an obstacle's disc is skipped, and it used to mean **no frame there at all** —
18 of 97 on `fmi_block_obst`, the layer's single biggest coverage cost. But unreachable is a fact
about the *waypoint*, not about the ground under it: the detour passes at `CLEAR_R` from the
obstacle's surface, and a street centerline crosses a structure's shadow rather than aiming at its
middle, so most skipped waypoints pass within a footprint of the arc. The controller now remembers
each one and takes the shot at **closest approach on the path it already flies** — it never steers
for a frame, and the real waypoint always wins the camera (the scheduler is gated on `not
arrived`). The frame keeps the waypoint's own index and `wp`, so numbering stays 1:1 with the
route, and carries `"standoff": true` + `"standoff_m"` (measured **at capture**, not at the
trigger, because that is the frame that exists). `analyze_stage_b.py` reports recovered vs still
uncovered.

**The gate is the footprint, and the footprint is not a circle.** The image is 400×240 px over a
45° horizontal FOV with **up = the drone's heading**, so at 30 m it reaches 12.4 m *across* the
nose and only 7.5 m *along* it. A waypoint passed abeam at 10 m is fully in shot; the same 10 m
dead ahead is not. The first version tested the inscribed circle (7.4 m, the safe radius when yaw
is unknown) — but yaw **is** known when the shot fires, and since a detour passes its obstacle
abeam, the circle throws away most of what is recoverable. Measured on `fmi_block_obst`, same 18
unreachable waypoints, **all three variants scoring 100%**:

| gate | recovered | bays classified | uncovered |
|---|---|---|---|
| none (skip, as before) | — | 100 | 11 |
| circle, 7.4 m | 4 | 102 | 9 |
| **footprint, ±10.6/±6.3 m** | **9** | **104** | **7** |

**What is still unreachable is DECLARED, not hidden.** Nine waypoints get no frame at all on
`fmi_block_obst`, five of them consecutive — a structure the route runs straight *through*. That is
a legitimate survey result, but only if the drone says so, so the controller writes
`sim/output/<area>/coverage.json` (`waypoints`, `captured`, `unreachable`, `standoff`,
`uncovered`), rewritten on every change like `poses.json` so an interrupted flight still leaves an
honest account. `score_occupancy.py` prints it beside the accuracy number, because an uncovered bay
has two very different causes — a bad pass (fixable, capture scatter) or an obstacle — and the bare
count cannot tell them apart.

Two things to keep right: a standoff capture **must not advance `idx`** or touch `wp_min`/
`wp_steps` — the drone is still flying to its current waypoint, and eating that state would eat the
capture the route asked for. And note a standoff capture *in flight* can delay a waypoint capture
by up to `CAM_WARMUP` steps, which is why the obstacle world's ordinary frames are not
bit-identical between the two gates; on a survey world `pending` is never populated (no ray fan, no
detour) so the whole layer is dead code and the survey path is untouched. `CAM_FOV`/`CAM_ASPECT`
are a **third copy** of the camera intrinsics (proto, `score_occupancy.py`, here) — the same
standing duplication as `ORIGIN` and `DS_*`.

**Remaining:** the tower the route doubles back through still costs one 238° swing and one
`STEER_ORBIT` bail-out; `trap` is passed but only via the escalation ladder (two 150 s timeouts).

### 2b. Vision datasets & detector tooling (built 2026-08-21, `vision/`)
```bash
python vision/diag/projection_selftest.py            # project/unproject round-trip
python vision/dataset.py fmi_block --overlay 6 --yolo --crops
python vision/detect_baseline.py fmi_block --conf 0.10 [--scale N] [--model M]
python tools/dji_srt.py flight.SRT --json poses.json # real DJI telemetry -> poses
python tools/dji_stills.py flight.MP4 --hz 1         # video -> stills + poses.json
python vision/detect_real.py <stills_dir> --labels <dir>  # zero-shot on REAL frames
python vision/check_labels.py vision/data/dji_0035 --overlay 6
python vision/train_detector.py --epochs 80 --imgsz 1024  # fine-tune probe
```
- **`unproject(u, v, pose)`** in `score_occupancy.py` is the exact inverse of `project()`: the pixel
  names a ray, the answer is where it meets z=0. A whole-frame detector needs it (a detection is a
  box in pixels and has to land on the ground to become a bay verdict); the per-bay classifier never
  did. `projection_selftest.py` round-trips **every bay corner of both fixtures** — 49,188 of them,
  worst error 0.000000 mm. Note what it does *not* prove: both functions share `camera_axes()`, so an
  error in the axes cancels — `paint_align.py` is what tests the axes against the world.
- **Closed bays are EXCLUDED from the score, not counted as free.** `score_occupancy.py` reads
  `worlds/<area>.closures.json` beside `coverage.json` and takes those bays out of the confusion
  matrix entirely — neither TP/TN/FP/FN nor `uncovered` — because "is a car parked here" is not the
  question being asked of them. Counting a closed-and-empty bay as a correct FREE call would inflate
  accuracy with bays the product deliberately hides. Measured on `fmi_block` with the example
  closure: 42 → 38 classified, 4 excluded, and the golden run with no closure file is unchanged at
  **42/42, 100%**. (This file now also does `sys.stdout.reconfigure(encoding="utf-8")` — it prints
  a Bulgarian street name for the first time, and Git Bash defaults to cp1252, which raises.)
- **`<name>.cars.json`** — the generator now writes every parked car (bay, model, body centre,
  angle, L, W) beside `ground_truth.json`. Sidecar only: the `.wbt`, `ground_truth.json` and
  `route.json` regenerate **byte-identical** (verified on both survey worlds).
- **`vision/dataset.py`** turns a flown area into a labelled detection set: project each car's
  footprint *and roof* through `project()` to a pixel box, clip, drop boxes less than 55% visible.
  Exact labels, no hand annotation, and they cannot drift from the world because both come from the
  same placement loop. The roof matters — a car 12 m off nadir has its roof displaced ~0.5 m (9 px)
  outward, so a footprint-only box is systematically tight on exactly the cars furthest from the
  image centre. `--overlay N` draws the boxes on frames, which is how you check them.
  **Volume is bounded by flight length, not world size**: ~0.9 cars per frame (30 boxes on
  `fmi_block`, 87 on `fmi_block_4st`), so a bigger training set means more/longer flights.
- **`vision/diag/classifier_ablation.py`** answers "which of `classify()`'s five tests earn their
  place, and does normalising help?" It extracts every per-view feature once per world and caches it
  (`--refresh` to rebuild), then evaluates a classifier variant in milliseconds against the exact
  vote `score_occupancy.main()` uses. Its `current` row **must** reproduce each world's committed
  accuracy — that is the harness's self-test, and it is how a broken harness gets caught before a
  conclusion is drawn from it. Findings are in TODO #6; the short version is that `chroma` is the
  only load-bearing test and every colour normalisation therefore damages the calibrated worlds.
- **`detect_baseline.py`** scores an off-the-shelf detector two ways: detection (IoU≥0.5 vs the
  exact boxes) and **occupancy** (unproject each box, assign to a bay within 3 m, vote across frames
  exactly as the scorer does). The second is the only one comparable with `classify()`.
  **Result: COCO-pretrained YOLOv8m detects 0 of 52 cars** across both worlds — 0% recall, no false
  positives either, and occupancy lands on the base rate (58.8% / 45.8% = "everything is free").
  It is **not** resolution: 2×/3×/4× upscaling changes nothing, and at conf 0.01 the model calls a
  nadir car a *tie* (0.19) and a van a *traffic light* (0.64). It is the domain — COCO cars are
  photographed from the side. So fine-tuning is mandatory, not an improvement, and the sim's free
  exact labels are what makes that cheap. (The supervisor's `ground-vehicles-localization` runs this
  same model zero-shot successfully on *real* footage, which makes "does it work on real nadir
  video" a specific, worthwhile experiment rather than an assumption.)
- **`tools/dji_srt.py`** parses the DJI SRT sidecar (both the modern `[latitude: ...]` layout and
  the legacy `GPS(lon,lat,alt)` one) into `project()`-compatible poses, preferring `BAROMETER` over
  the GPS triple's altitude. It deliberately does not trust the GPS (1–3 m against a 2.2 m bay — the
  pose is the *starting point* for registration) and does not assume nadir (it carries gimbal angles
  through when present and leaves them absent when not, so the fallback is the caller's choice).
- **Real footage is IN (2026-08-21 flight, scored 2026-08-22).** `pics/dji/` holds three DJI
  clips over the FMI block at a constant 30.1 m — the sim's calibration altitude.
  `tools/dji_stills.py` cuts a video to stills at a fixed rate and pairs each with its SRT pose **by
  frame index, not timestamp** (the SRT has exactly one block per video frame, 1-based `FrameCnt`,
  so there is no clock alignment to get wrong); it grabs sequentially and drops frames rather than
  seeking per still, which on long-GOP H.265 is both far slower and can land off-target.
  Real numbers that matter downstream: **GSD 11.7 mm/px**, footprint **45.0 x 25.1 m**, a 4.5 m car
  is **~385 px**. That is a *fourth* copy of the camera intrinsics (proto, `score_occupancy.py`,
  `parkdrone.py`, and now real-drone values that differ from all three — 73.7 deg H / 45.4 deg V
  against the Mavic2Pro proto's 45 deg), so anything reusing `CAM_FOV`/`CAM_ASPECT` needs a
  real-drone variant.
- **`captured_at` — the file records the truth, the REPLAY does the shifting.** `dji_srt.py` now
  parses the camera's wall clock (`2026-08-21 16:31:09.139`, its own line in each block — it had no
  regex before and was silently dropped) into every pose, and `dji_stills.py` carries it into
  `poses.json` beside the relative `t_s`. It is stored **naive** because DJI writes no timezone
  offset; stamping a UTC marker on local aircraft time would be a lie that is hard to unpick later.
  `dji_srt.rebase(poses, start=None)` (CLI `--as-now`) shifts every stamp so the first frame lands
  at `start`, **keeping the flown intervals** (verified: 133.468 s preserved against 133.5 s as
  flown). It is deliberately **not** applied when the file is written — `poses.json` records what
  happened, rebasing is a consumer decision, and writing a fake "now" at extraction time would
  destroy the only copy of the real time. Why it exists: occupancy votes over `OCCUPANCY_WINDOW_S`
  (2 h), so replaying yesterday's flight with yesterday's stamps yields a map where every bay has
  already expired to `occupied: null`.
  **Decided 2026-08-22: `observation.observed_at` stays `DEFAULT now()`.** For the live uplink,
  capture and ingest are seconds apart; for an `--as-now` replay the rebase has already made ingest
  time the intended truth. So `now()` is *correct*, not merely tolerable, and it avoids a nullable
  column plus a "which timestamp does the vote use" branch in every read path. `captured_at` is
  still worth storing on the **`frame`** row as provenance whenever that table next changes — it is
  the append-only ingest ledger, it stays out of the vote, and after the stills are separated from
  the SRT it is the only copy of when the footage was shot.
- **`vision/data/dji_0035/` — the first HAND-labelled dataset, 31 frames / 108 boxes.** Rules in
  its `LABELING.md`, record in its `README.md`. YOLO format, single class `car`, axis-aligned,
  **byte-identical in shape to `vision/dataset.py --yolo`** so sim and real frames pour into one
  training run. Labels are **tracked**; the 87 MB of JPEGs are gitignored and regenerate from the
  video. Two conventions to keep: an **empty `.txt` is not a missing one** (empty = a human checked
  and there were no cars, missing = nobody looked, and ultralytics reads missing as "no objects" —
  which would teach the model that a frame full of cars is empty), and the **train/val split is
  SPATIAL, not random**. That last one is load-bearing: this flight goes out east, doubles back over
  the same street, then heads northwest, and frames 55-85 sit within **3-6 m** of frames 5-30 — the
  same parked cars. A random split puts one photo of a car in train and another in val and the model
  scores brilliantly by memorising it. `check_labels.py` validates what a hand-made file can get
  wrong and a generated one cannot (pixel coords instead of normalised, wrong class index,
  zero-area boxes, boxes off the edge, boxes too big or small to be a car) and draws them back onto
  the frames.
- **Zero-shot on REAL nadir footage: ~30% recall, ~50% precision — the sim's 0% did NOT transfer.**
  `detect_real.py` is the real-footage counterpart of `detect_baseline.py` (which needs exact boxes
  and a `ground_truth.json`, neither of which real footage has); `--labels` scores it against the
  hand labels at IoU >= 0.5. Over 31 labelled frames / 108 true cars, COCO YOLOv8m:

  | | whole frame | 2x2 tiles |
  |---|---|---|
  | recall, class `car` | 28.7% (31/108) | 30.6% (33/108) |
  | recall, any vehicle class | 34.3% | 31.5% |
  | precision of `car` boxes | 56.4% (31/55) | 50.8% (33/65) |

  **This splits a conclusion the sim could not.** On Webots frames the same model detects **0 of
  52**; on real photographs of the same task it finds a third. So the renderer was a large part of
  that 0%, and the residual gap is viewpoint, colour and canopy. **Tiling to native scale does not
  rescue it** (28.7 -> 30.6%, precision *drops*), so resolution is not the binding constraint — a
  car is ~385 px and the model still misses it. The dominant failure has a name: the tiled pass
  emits **224 `cell phone` detections at median confidence 0.78**, because a dark car roof on pale
  pavement is a glossy rounded rectangle with a lighter inset. Same failure the sim showed as "a
  nadir car is a *tie* (0.19)", but at high confidence, because the pixels are real. Failures skew
  hard by **body colour and tree canopy**, neither of which the sim generates.
- **Fine-tuning augmentation is nadir-specific, and that is what makes 82 boxes worth training on.**
  `vision/train_detector.py` sets `degrees=180` and `flipud=0.5`, both **0 by default** in
  ultralytics. Straight down there is no canonical "up" — the drone's heading is arbitrary and a car
  may point any way in the frame — so full rotation and vertical flips are *truthful* expansions of
  a tiny dataset rather than distortions. (In side-on COCO imagery a vertical flip means an
  upside-down car, which is why the defaults are off.) On CPU, imgsz 1024 costs ~64 s/epoch on 22
  images; imgsz 1280 costs ~98 s.
- **Known blocker: `huggingface.co` downloads fail on this machine.** Something intercepts TLS and
  its CA is non-compliant ("Basic Constraints of CA cert not marked critical"), which **Python 3.13+
  rejects by default** now that strict X.509 is on. certifi does not help, nor does adding the 230
  Windows store roots, nor clearing `VERIFY_X509_STRICT` on urllib3's context. `ultralytics` gets
  its weights only because it falls back to `curl`. So HF-hosted models (OWLv2 for label
  bootstrapping, RT-DETR) need their files fetched with `curl --ssl-no-revoke` into the HF cache —
  the same workaround `data/README.md` already uses for the Sofiaplan API.

### 2c. Real footage -> the live map (built 2026-08-23)
```bash
python tools/dji_yaw.py <stills_dir> --self-test     # prove the estimator first
python tools/dji_yaw.py <stills_dir> --write         # recover + write `yaw`
python vision/diag/real_align.py <stills_dir> --every 20   # LOOK at the overlays
python vision/detect_occupancy.py <stills_dir>       # bay verdicts, offline
# live, through the whole stack:
OCCUPANCY_BACKEND=detector python -m parkdrone_vision.server
API_KEY=$(cat web/.quickstart/drone-1.key) python -m parkdrone_vision.sim_uplink     dji_0035 --root pics/dji/stills/DJI_..._0035_D --pattern 'frame_%04d.jpg'
```
**Verified end to end 2026-08-23: 137 real frames ingested, 0 failed, 88 bays now
carry a `detector` verdict on the live map.**

- **The blocker was YAW, and the SRT does not have it.** These flights record only
  `latitude`/`longitude`/`rel_alt` -- no `gb_yaw`/`gb_pitch`/`gb_roll` block at all -- so
  `poses.json` had a position and no orientation, `project()`/`unproject()` could not orient a
  frame, and `frame.yaw` is `NOT NULL` in the web schema, i.e. a real frame could not even be
  ingested. `tools/dji_yaw.py` recovers it: the GPS gives the world displacement between two
  stills and image registration gives the ground content's pixel shift, and those are the same
  vector in two frames, so `yaw = course + atan2(-du, dv)`. It writes `yaw` + `yaw_src`
  (`flow`/`course`) + `yaw_resid_deg`.
  **Phase correlation does NOT work on this footage and normalised cross-correlation does.**
  The textbook choice for a pure translation returned peak responses of 0.002-0.02 and shifts
  6.5x off the GPS, because a 30 m nadir frame over Lozenec is mostly summer canopy -- broadband,
  self-similar, with a parallax of its own -- so there is no single global translation to lock on
  to. `cv2.matchTemplate` on the central 40% scores 0.66-0.84 on the same pairs.
  **Measured on flight 0035: implied GSD 12.50 mm/px against 11.74 expected (ratio 1.065), and
  flow-vs-GPS-course agree to a median 1.9 deg / max 8.4 deg** -- two independent estimates
  converging, which is the evidence the number is real. 125 of 137 poses come from flow.
  `--self-test` is not optional and gates everything: a wrong yaw does not look wrong, it
  produces a confident, plausible, wrong map. It proves the algebra against `project()` itself
  (352 yaw/course pairs, 5e-14 deg), the registration against known shifts on a real frame
  (worst 1.4 px), and the two composed (5 headings, worst 0.28 deg).
- **`vision/cameras.py` -- intrinsics are now a VALUE, not module globals.** `project()`,
  `unproject()` and `footprint_reach()` take an optional `cam=`; the default is built from
  `score_occupancy`'s own `IMG_W`/`IMG_H`/`FOV`, so this file still owns the sim numbers and every
  existing caller is byte-identical. `DJI_NADIR` is 3840x2160 at 73.7 deg -- a figure that lived
  only in prose until now. `projection_selftest.py` round-trips both cameras (49,188 corners,
  0.000000 mm) and `tools/check_consistency.py` now has a real-camera check (35 checks, was 33).
- **The projection was verified by eye across all three flight legs before any number was
  computed** (`vision/diag/real_align.py`): bay rectangles land on the parked cars, aligned with
  the street and the right size. **No GPS-bias correction was applied, and that was a measurement,
  not an oversight** -- the best global ENU shift over 115 detections only moves the median
  detection-to-bay distance 4.35 -> 3.47 m and the within-3 m count 34 -> 42, at an implausible
  4.5 m (about a car length). *Re-measured 2026-08-29 on all 599 detections: 4.47 -> 3.60 m,
  175 -> 230 of 599 (29% -> 38%) at (-4.0, +0.5) m -- same conclusion, 5x the sample. Run this over
  EVERY detection: the same search over the 473 UNASSIGNED ones alone reports a spurious 10% -> 37%
  at 6.5 m, because that set is defined as the detections that already missed.* A real bias would show as a tight cluster at a small offset. What
  the spread actually says is a DATA fact: **most cars on this street are not in a Sofiaplan-mapped
  bay** -- visible directly in the overlays, where a whole column of bays sits over a pavement
  strip while the cars are parked on the cobbles beside it.
- **`vision/detect_occupancy.py::bay_votes_from_dets` is the one detections->bays rule**, extracted
  from `detect_baseline.py` (whose numbers are the check that the extraction changed nothing:
  still 0/52 detections, 58.8% base-rate occupancy on `fmi_block`). Box CENTRE unprojected to the
  ground, bay occupied if a hit lands within `ASSIGN_MAX_M` 3.0 m of its centroid, only bays FULLY
  in shot vote, strict majority across frames -- so a real verdict and a sim verdict are produced
  by identical geometry and only the box source differs.
- **`OCCUPANCY_BACKEND` selects the model, and `heuristic` stays the default.** The detector is
  for real photographs, where `classify()` is meaningless; `classify()` is for rendered frames,
  where a COCO-scale detector finds 0 of 52 cars. They are not interchangeable, so this is a
  deployment-level choice -- a survey area is either simulated or real and whoever starts the
  server knows which. `DETECTOR_IMGSZ` defaults to 1024 to match what `merged1` was trained at.
  **Each classify thread gets its OWN model instance**: ultralytics keeps mutable predictor state
  on the model object, so sharing one across threads is a data race, and a 22 MB copy is cheaper
  than the lock that would undo the pool's parallelism.
- **Migration `0010` records WHICH model decided a bay** (`observation.backend`/`det_score`,
  `bay_state.backend`, all nullable, no backfill -- stamping pre-0010 rows would assert a fact the
  migration never saw). It is not decoration: a detector observation has no `core_chroma` and a
  heuristic one has no `det_score`, so NULL would otherwise be indistinguishable from a failed
  measurement, and the two results must never be averaged. `/api/v1/bays` returns `backend` gated
  on the same freshness as `occupied`, and the map popup shows it as a Source row.
- **The client's freshness TTL was wrong and is now the server's.** `types.ts` hard-coded 10
  minutes against `OCCUPANCY_WINDOW_S`'s 2 h, so a landed survey greyed out in the browser while
  the API still reported it. `/health` now publishes `occupancy_window_s` and the client adopts it
  at startup (2 h fallback).
- `sim_uplink.py` gained `--root`/`--pattern` rather than gaining a real-footage twin -- mission
  start/end, resume detection, the retry that leaves an unsent frame unsent and the
  image-before-pose ordering rule are all identical for a directory of DJI stills. It stamps
  `frame_idx` into the pose it POSTs, because a real pose is identified by `file` and has none.
- **Known limit, by design this round:** there is no per-bay ground truth for flight 0035, so this
  is a working pipeline and a demo, NOT a real accuracy number. `/api/v1/metrics` correctly reports
  accuracy as `null` -- the production case. Getting a number means hand-labelling the ~86 bays
  under this flight's footprint.

### 2d. The demo: annotated video + the live map (built 2026-08-23)
```bash
python vision/render_demo.py pics/dji/stills/DJI_..._0035_D --out pics/dji/demo_0035.mp4
# then, side by side:
cd web/apps/vision-worker && OCCUPANCY_BACKEND=detector python -m parkdrone_vision.server &
cd web/apps/web-user && pnpm exec vite &                      # map on :5173
API_KEY=$(cat web/.quickstart/drone-1.key) python -m parkdrone_vision.sim_uplink     dji_0035 --root pics/dji/stills/DJI_..._0035_D --pattern 'frame_%04d.jpg'     --rate 1.0 --once                                          # start with the video
```
**Pre-rendered, deliberately.** Running the detector live during a presentation puts a CPU
inference pass on the critical path, where a slow frame is a stall in front of an audience.
`render_demo.py` does everything expensive once and caches the detections beside the video
(`<out>.dets.json`), so a re-render with different drawing options costs seconds.

- **It plays in real time and stays there.** The stills were cut at 1 Hz, so `--hold 1.0` shows
  each for one second: 137 frames, 137 s, the speed the drone actually flew. `--fps` is a separate
  knob -- each still is simply repeated `hold*fps` times -- because a 1 fps MP4 scrubs badly and
  stutters in some players while a 10 fps one holding each image for ten frames plays smoothly.
  `--rate 1.0` on the uplink replays the frames at the same 1 Hz so the map advances in step.
- **Three layers are drawn and each earns its place**: amber detection boxes with confidences
  (the model), projected bay outlines coloured by *this frame's* verdict — red occupied, green
  free (the product decision, computed by the same `bay_votes_from_dets` the server runs, so a
  viewer can watch a box land inside an outline and see it turn red), and a HUD carrying frame,
  time, **yaw and its source**. The yaw is on screen because it is *estimated*, and a demo that
  hides that is overclaiming. Note the per-frame colour is the SINGLE-VIEW verdict: a bay can
  flicker here and still be decided correctly on the map, which is the multi-view vote working
  and is worth pointing at rather than hiding.
- **Codec:** `mp4v`. This machine has neither OpenH264 nor ffmpeg, so H.264 silently falls back
  and produces nothing usable. mp4v plays in VLC and Windows Media Player but **not reliably in
  Chrome**. At 1920x1080/10 fps the file is ~159 MB; `--width 1280 --fps 6` brings it to ~45 MB
  at the same real-time speed.
- **`localhost` COSTS 2 SECONDS PER REQUEST ON THIS MACHINE — always use `127.0.0.1`
  (found 2026-08-28).** This was the real cause of the demo desync, and it is worth internalising
  because it is invisible and it is everywhere. uvicorn binds **IPv4 only** while vite binds
  **`[::1]`, IPv6 only** — the two dev servers sit on opposite sides of an ambiguous name — so any
  client resolving `localhost` to `::1` first must wait for that connection to fail before falling
  back. Measured on this box:

  | path | via `localhost` | via `127.0.0.1` |
  |---|---|---|
  | `GET /health`, Python urllib | **2.050 s** | **0.005 s** |
  | ingest POST of one still | **2.13 s** | **0.24 s** |
  | `/api/v1/summary` through vite's proxy | **2.032 s** | **0.043 s** |

  **The cost is FIXED, not proportional** — a 2.78 MB frame and a 0.24 MB frame both took 2.15 s,
  which is the fingerprint to recognise it by. It made the 1 Hz demo replay take **301.5 s for 137
  frames** against a video that is exactly 137.0 s, and it added 2 s to every fetch the map itself
  made. Fixed in `sim_uplink.py`'s default `api_base`, both `vite.config.ts` proxy targets, and
  quickstart's `/health` probe. **Anything new that talks to :4000 must use `127.0.0.1`.**
- **The replay also paces to a WALL CLOCK, not a per-frame sleep.** `--rate` used to
  `time.sleep(1.0 / rate)` *after* reading and POSTing the still, making the real period
  `send + 1/rate` with an error that **accumulates**. That is a genuine defect and is fixed (sleep
  until `t0 + n/rate`, which absorbs the send cost and self-corrects) — but on its own it was **not**
  the cause above: no pacing scheme can absorb a 2 s stall inside a 1 s frame period. Both are
  needed, and they compose: 127.0.0.1 brings the send to 0.24 s, under the frame period, and the
  deadline then holds real time exactly (with the old fixed sleep it would still run 1.24 s/frame,
  finishing 24% behind the video). If the replay ever genuinely cannot hold the rate it now says so
  once instead of drifting in silence.
- **It was never a throughput problem, and more replicas would NOT have helped.** Measured, the
  whole classify path is ~0.41 s/frame against 1 frame/s arriving (detector inference 0.36 s,
  `observed_spans` over all 141 runs 0.034 s, `bay_votes_from_dets` over 1698 bays 0.012 s), and
  torch already saturates all 12 cores, so extra threads or replicas contend rather than add
  capacity. Note CLAUDE.md's "~103 frames/s per classify thread" is the *heuristic* figure; the
  detector is ~2.8 frames/s. Scaling out remains about availability, not speed.
- **Watch out for two survey areas owning the same bays.** `bay_state` is keyed per bay, and
  `recompute_states` resolves the newest mission *within one survey_area* — so `fmi_block` (sim)
  and `dji_0035` (real) cover the same physical block and whichever ingested last owns the
  overlap. Running the `fmi_block` replay after the real flight silently took 30 of the 88 real
  bays back to `heuristic`. Not new behaviour and not a bug in this work, but **run the real
  flight LAST before a demo**, and the Source row in the popup is how you check.

### 2e. Test-run record — the real pipeline, verified 2026-08-23
Every gate below was run on this date, in this order. They are ordered because each one is only
meaningful if the previous passed: a projection cannot be trusted before the yaw estimator is, and
no accuracy claim means anything before the projection has been looked at.

| # | Command | Result |
|---|---|---|
| 1 | `python tools/dji_yaw.py --self-test` | **pass** — algebra vs `project()` 352 pairs worst **5e-14 deg**; registration on a real frame, 4 known shifts, worst **1.41 px**; end-to-end yaw→image→yaw, 5 headings, worst **0.28 deg** |
| 2 | `python vision/diag/projection_selftest.py` | **pass** — 49,188 corner round-trips **0.000000 mm**, and the same again through `DJI_NADIR` |
| 3 | `python vision/diag/real_align.py <stills> --every 20` | **pass, by eye** — bays land on the parked cars on all three flight legs (yaw 30 / 205 / 108 deg) |
| 4 | `python tools/check_consistency.py` | **35 checks pass** (was 33; +2 for the real camera) |
| 5 | `python vision/detect_baseline.py fmi_block` | **unchanged after the extraction** — TP=0 FP=0 FN=30, occupancy 58.8% (the base rate on record) |
| 6 | `python -m parkdrone_vision.replay fmi_block` | **42/42, 100%** (TP=17 TN=25 FP=0 FN=0) — the heuristic path is untouched |
| 7 | `pnpm db:migrate` then `replay_ingest fmi_block` | `0010` applied; **42/42**, 14 WS deltas, new rows carry `backend='heuristic'` |
| 8 | real uplink of flight 0035 | **137 frames, 0 duplicate, 0 failed**; 88 bays with a `detector` verdict; `/api/v1/metrics` accuracy `null` (no GT — the production case) |

**Measurements taken during the run, worth keeping:**
- yaw estimator on flight 0035: implied GSD **12.50 mm/px** vs 11.74 expected (**ratio 1.065**);
  flow vs GPS course **median 1.9 deg / 90th 4.2 / max 8.4**; 125 of 137 poses from flow, 12 from
  course; 1 frame screened out as non-nadir.
- detector on the 137 real frames: **473 detections, 3.5/frame** (median 3, max 10), confidence
  p10 0.33 / p50 0.68 / p90 0.83, only **20 overlapping pairs** at IoU>0.3. Inspected the busiest
  frame: **10 boxes on 10 real cars.** The detector is not over-firing.
- detection-to-nearest-bay distance over 115 detections: **median 4.35 m**, p10 1.68, p90 11.16;
  only 30% within the 3 m assignment radius. Best global ENU shift (grid search +/-6 m) reaches
  median 3.47 m / 42 of 115 at an implausible **4.5 m** — so there is **no GPS bias worth
  correcting**, and the spread is the data instead. *(Reproduced 2026-08-29 on all 599 detections:
  4.47 -> 3.60 m, 29% -> 38% at 4.03 m.)*
- 674 detector observations recorded, **102 carrying a `det_score`** (avg 0.641), zero heuristic
  colour statistics — the intended shape for migration `0010`.

**Re-running gate 6 or 7 AFTER gate 8 silently steals bays back from the real flight** — measured,
it took 30 of the 88. `bay_state` is per bay and the two survey areas cover the same block. See
the warning in 2d.

### 2f. Flights 0074 & 0075 — a second day, a channel bug, and the bay data (2026-08-25)
```bash
python tools/dji_stills.py pics/dji/DJI_20260825125321_0074_D.MP4 --hz 1
python tools/dji_yaw.py <stills> --self-test && python tools/dji_yaw.py <stills> --write
python vision/check_nadir.py <stills> --list          # 22% of 0074 is OBLIQUE
python vision/detect_occupancy.py <stills> --json vision/runs/<name>.json
python vision/make_split.py <stills> --every 3 --exclude 81-83 --exclude 111-138        --out vision/data/dji_0074 --val-from 9999 --prefix f0074_   # train-only
python vision/prelabel.py vision/data/dji_0074 --split train        --model vision/runs/merged1/weights/best.pt --conf 0.25 --imgsz 1024
```
Two clips over the same block at ~30 m nadir, flown at **12:53 midday** where 0035 was 16:31.
Full record in `docs/training.md`; the parts that change how this repo is used:

- **ultralytics reads a numpy array as BGR**, the cv2 convention it trains with. Three call sites
  built theirs from PIL `.convert("RGB")` (or reversed a cv2 array into RGB) and so ran the detector
  with **red and blue swapped**: `vision/detect_occupancy.py`, `vision/render_demo.py`, and
  `vision-worker/.../vision/scoring.py` — **the live backend**. Fixed. `prelabel.py` and
  `detect_real.py` use `cv2.imread` and were always right, so the training labels and every reported
  recall/precision number are unaffected; the live map was not. **Cost on flight 0035: 473 -> 599
  detections (+27%).** It was found only because two paths disagreed on identical frames (108 vs 173
  on 0074), and the two now agree to the exact count — which is the check that the fix is right. In
  `scoring.py` only the detector's argument is reversed: `img_arr` stays RGB, because the heuristic's
  crops depend on that.
- **The nadir screen runs inside the occupancy pass.** `check_nadir.is_oblique()` is now a function
  (split out of that script's `main()`) and `detect_occupancy.run()` drops flagged frames;
  `--no-screen` opts out. It matters because an oblique frame does not fail to project — it silently
  places its cars tens of metres away with full confidence — and **22% of flight 0074 is oblique**
  (frames 81-83 and 111-138, confirmed by eye). On 0035 it removes one frame and reproduces the 88
  bays already on record.
- **The detector generalises across the light**: on 0074's nadir frames confidence is p10 0.33 /
  p50 0.63 / p90 0.80 against 0.33 / 0.68 / 0.83 on 0035, and 0075 gives 4.3 detections/frame
  against 3.5. No hand labels exist for either flight, so these are distributions and eye checks,
  **not** a recall number.
- **Neither flight produces a single occupied bay** — 0 of 49 on 0074, 0 of 9 on 0075, from 273
  detections — and that is the bay data, not the vote. 0035 is the positive control: there an
  occupied bay reads **11/13, 10/12, 7/7, 6/6** views, because a car in a mapped bay is seen in
  nearly every look at it. On 0074 the best bay in the flight is **6 of 21**; on 0075 all nine bays
  have **zero** occupied views. Not a yaw error either (correcting yaw alone moves 0075's median
  8.95 -> 8.19 m), and no shared bias to correct: 0075's best shift is (-7, -8) and 0074's is
  (+3, -9), three minutes apart. The `real_align.py` overlay shows it directly — **painted, occupied
  bays are plainly visible in the photograph** while the Sofiaplan rectangles for that frame sit on
  a garden and under canopy.

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
- **`vision/data/dji_0074` (36 frames / 60 boxes) and `dji_0075` (23 / 100) are pre-labelled and
  waiting on hand correction.** Both **train-only**, forced by geometry: `--suggest` finds no split
  point on 0074 reaching the 25 m footprint, because the flight hovers and doubles back throughout.
  Benchmark them against `dji_merged`'s hand-drawn val set — checked at **68 m (0074)** and
  **238 m (0075)** from those val frames before building, since training on frames that overlap a
  benchmark leaks it.
- `detect_real.py` decided "is this a vehicle" by **COCO class index**, but a fine-tuned single-class
  model calls `car` index 0 — COCO's `person`. Per-frame counts, size stats and overlay colours were
  wrong for our own model while `--labels` scoring (matching on the name) was right, so it never
  showed up in a reported figure. Now matched by name. `make_split.py --exclude` is repeatable (a
  flight's oblique stretches are not one block), applies **before** `--suggest` rather than after,
  and no longer reports a train-only dataset's vacuous `inf` separation as "genuinely spatial".


**2026-08-26 — ground control run, and the control flight settles the method.**
`vision/diag/paint_ground_control.py` unprojects painted road markings (paint is ON the ground
plane, so unlike a roof it has no parallax) into an ENU paint map, rasters the Sofiaplan bay
outlines on the same grid, and slides one over the other. `--self-test` recovers injected shifts
exactly (0.00 m on four vectors) and gates the result.

Run on both flights through the **identical** mask, so the extractor's own imperfections cancel and
the comparison carries the argument:

| flight | overlap at (0,0) | peak | headroom | peak/median | occupancy result |
|---|---|---|---|---|---|
| **0035** (control) | 1204 | 1337 @ 4.03 m | **9.9%** | 1.55x — flat | 11 bays occupied, 11/13 views |
| **0075** | 19 | 133 @ 4.51 m | **85.7%** | 3.98x — sharp | 0 bays occupied |

**The georeferencing method is sound** — but note the 0035 row was over-read at the time and is
**RETRACTED** (`docs/bay_geometry_verification.md` Finding 2a): a flat surface buying 9.9% means the
paint mask carried too little signal to localise anything, which is *uninformative*, not a positive
control saying the bays sit on the paint. On бул. Джеймс Баучер (61 of that flight's 88 bays) the
rows in fact land on the tram rails and the pavement while the cars are on the cobbles. **The
0035-vs-0075 CONTRAST is what survives** (flat vs sharp), and the pose chain is cleared instead by
references this mask does not touch: OSM road centerlines land on the roads in 0035's own frames
(`vision/diag/ref_align.py`, `out/ref0035/`, 2026-08-29), and the best global ENU shift over all
599 detections moves within-3 m only 29% -> 38%, at an implausible 4.03 m.
**On 0075 there IS a real disagreement**, ~4.5 m, and the bays plainly do not sit on that street's
paint. But 4.5 m does **not** explain the occupancy result: the detections' median distance to the
nearest bay there is 8.95 m, so correcting it would still leave most cars unassigned. So the honest
reading of that street is BOTH — a few metres of geometry disagreement, and cars genuinely parked
away from the mapped bays.

**What the paint extractor cost, and its limit.** A brightness-and-saturation threshold is useless
here: it marked 2.9% of the frame, because a white car roof is bright and grey exactly like paint,
and produced a map that was a solid blob (peak 1.43x = meaningless). Removing blobs by erosion, then
masking the detector's own car boxes (a car OUTLINE survives erosion and is a bay-sized rectangle
lying exactly where the question is — left in, the method would have been assuming its answer), then
keeping only elongated components, got the peak to 3.98x. The adversary specific to this footage is
**midday sun through summer canopy**, which scatters dappled highlights that are bright, grey and
thin — paint by every test except length. The mask now finds the strong markings (it picks the zebra
crossing cleanly) but **misses worn bay lines**, so the absolute offsets above are indicative; the
0035-vs-0075 CONTRAST is the load-bearing result, not the metre values.


### 2g. Unattributed detections — the cars the rule throws away (migration `0013`, 2026-08-26)
```bash
python vision/detect_occupancy.py <stills> --json out.json   # now reports them
curl -s localhost:4000/api/v1/metrics | jq .unassigned
```
`bay_votes_from_dets` loops over **bays**, so a detection matching no bay within `ASSIGN_MAX_M`
was silently discarded and left no trace anywhere — not in the JSON, not in the database, not in
the metrics. That made the pipeline capable of a confident half-truth: flight 0075 reported
**"9 bays, all free"** when what happened was *"9 bays free, and 100 cars we saw and could not
place"*. Zero occupied bays is a quiet signal; 100 unplaced cars is a loud one, and the loud one
was the one being dropped.

**Pass a list as `unassigned=` to be told.** It is an opt-in out-parameter, not a second return
value, because the return shape is a contract shared with the heuristic backend and
`process_frame` picks between them blind. Every existing caller (`detect_baseline.py`,
`render_demo.py`, `scoring.py`) passes nothing and is unchanged by construction — verified,
`detect_baseline fmi_block` still reports TP=0 FP=0 FN=30 / 58.8%.

**Two counts, not one, because they call for opposite fixes.** A detection a few metres outside
the radius means the bay geometry is slightly off (fixable by alignment); one nowhere near any bay
means that street's parking is not in the dataset at all (fixable only by extending the map). The
split is at 2x `ASSIGN_MAX_M`.

**What it immediately revealed, and it is bigger than the flight that prompted it:**

| flight | detections | matched no bay | median distance | within 6 m |
|---|---|---|---|---|
| 0075 | 100 | **100 (100%)** | 9.02 m | 12 |
| 0035 (the *working* flight) | 599 | **473 (79%)** | 5.21 m | 298 |

Flight 0035 is the one that produces sensible occupancy and whose bays sit on its paint — and even
there **four cars in five are attributed to nothing**. So the bay-coverage gap is a property of the
dataset across this block, not a quirk of the new flights; 0035 simply has enough cars landing in
mapped bays to yield 10 occupied. That number was being computed and thrown away, frame by frame,
since the detector backend was built.

**Storage:** `frame_job.unassigned_dets` / `unassigned_near`, on the work-state row rather than on
`frame` — `frame` is the append-only ingest ledger and is never UPDATEd, while this is a *result*
of classifying, produced with `status` and `finished_at`. Clearing a survey area drops the counts
with it through the existing CASCADE. `record_unassigned` deliberately does **not** commit: it is
part of the same unit of work as the observations, and a count surviving a rolled-back
classification would describe a frame nobody scored. NULL means "not recorded" and is the honest
value for pre-0013 rows **and for every heuristic frame** — a bay-crop classifier has no notion of
a detection belonging to nothing, so 0 would be a false claim rather than a missing number.
`GET /api/v1/metrics` gains an `unassigned` block (beside `coverage`, because it measures the
opposite gap: coverage counts bays the survey did not see, this counts cars it saw and could not
place) and three Prometheus gauges.

*Verified 2026-08-26:* migration applied to the live DB, columns nullable ints; writer round-trip
(91, 12) and `unassigned_counts` aggregating them, both inside a transaction that was rolled back
clean. Golden replay **42/42 at 100%**, `detect_baseline` unchanged, **39** consistency checks pass
(the "40" once quoted here is wrong; note the count is FIXTURE-DEPENDENT — one check reads a frame from `sim/output/fmi_block/`, so a tree whose golden fixture has been cleared reports 38 and nothing is actually broken).

### 2h. Curb runs — parking as a 1-D resource (migration `0014`; gaps, `0015`, 2026-08-28)
```bash
python tools/make_runs.py                          # bays -> data/curb_runs.geojson
python vision/label_runs.py <stills>               # hand-count cars per stretch
python vision/score_runs.py <stills> --gt-runs vision/data/gt_runs_0035.json \
       --dets pics/dji/demo_0035.mp4.dets.json     # the real accuracy number
cd web && pnpm quickstart --real                   # the demo, KERBS layer on
```
**The bay RECTANGLE was abandoned as the primitive.** Sofiaplan's geometry is wrong by more than a
bay width and no rigid correction fixes it (global shift 3.99 -> 3.59 m, per-street -> 3.35 m,
per-row-side -> 2.84 m, on the 56 hand-corrected bays of flight 0035). But the error is
**anisotropic** — cross-street 3.14 m against along-street 1.64 m — and the two axes behave
oppositely once parking is a line: the cross-street component decides *which run* a car is on and is
largely common-mode within a run (one robust scalar absorbs it, `estimate_lateral`), while the
along-street component merely slides a car along the kerb, and a **gap LENGTH is invariant** to
sliding every car by the same amount. A per-bay boolean is not — 3 m of along-street error flips it.
The representation does not fix the geometry; it puts the irreducible error on the axis where it
costs nothing.

**The measured result** (`vision/score_runs.py`, flight 0035, 18 hand-counted segments / ~280 m /
29 cars). Paired, the run layer recovers **+0.58 cars/segment more than the incumbent, CI
[+0.21, +0.96]** — significant. **The MAE differences (0.83/0.81/1.11) are NOT significant at n=18**
— every CI crosses zero, so the advantage is in *bias*, not per-segment precision. Do not quote the
MAEs as an improvement.

| prediction | cars found | bias/segment | 95% CI |
|---|---|---|---|
| instances (detector + cross-frame clustering) | 34 (117%) | +0.28 | [-0.22, +0.83] **unbiased** |
| run layer via occupied length / 4.4 m | 19 (66%) | -0.53 | [-0.91, -0.15] |
| **per-bay `bay_votes_from_dets` (the incumbent)** | **9 (31%)** | **-1.11** | **[-1.50, -0.78]** |

**Cars are counted as INSTANCES; intervals are for GAPS only.** Dividing voted occupied length by a
car length was the first version and it loses cars twice — once when a cell fails the strict
majority, again when two adjacent cars merge into one interval that divides to fewer than two. So
`run_summary` counts distinct cars (`cluster_points`, single-link at `CLUSTER_M` 2.0 m over
**frame-distinct** views) and then reconstructs intervals *from that count* (`car_intervals`) purely
to measure the gaps. The reconstruction is approximate by construction — a car's true extent is not
measured, only its centre — which is exactly why it is never allowed back into the count.

**`free` is a MEASUREMENT, not a subtraction (2026-08-28, migration `0015`).** It used to be
`capacity_observed - cars`, which can advertise a space that does not physically exist: four
badly-spaced cars on a 30 m run leave the subtraction reporting 2 free while the real gaps are 1.5 m
each and nothing fits. A driver feels that error directly, and the subtraction cannot say *where* to
go at all. `free_gaps` now measures what fits between the reconstructed intervals. **Measured on
flight 0035: 1102 spaces from gaps against 1164 by subtraction — 62 advertised spaces that do not
fit.** The old number is kept beside it as `free_by_subtraction` rather than replaced in silence.
`run_state.gaps` is **jsonb, not a child table**: a gap has no identity, no history and no
independent lifetime — it is derived wholesale on every recompute and only ever read with its run.
Both new columns are nullable with **no backfill**, the same posture as `0010`/`0013`.

**A half-open interval dropped every car at the END of a kerb.** `locate` CLAMPS anything past a
run's end to exactly `s == length`, and the observed-span test was `a <= s < b` — so cars at the
tail of a run failed it and vanished. Measured on flight 0035, **3 of 49 placed instances (6%)**,
silently. `_on_observed` closes the interval at the run end, and anything still outside every span
(a genuine contradiction — a detection on kerb the geometry says was never in frame) is now
*reported* as `cars_off_observed` rather than dropped without trace. It was found by the invariant
`score_runs.py` now asserts: **what `run_summary` publishes must equal what the scorer measured.**
Keep that check — the scored `inst` column and the shipped path are different code, and the reported
bias means nothing if they disagree.

**Four earlier bugs, each of which produced a confident wrong answer:**
- **Gate on distance to the POLYLINE, never on `lateral`.** `lateral` is measured against the
  segment's *infinite* line, so a car 20 m past a run's end but collinear reads ~0 and clamps to
  `s = length`. That assigned 50 detections to one 45 m run as degenerate zero-length intervals,
  reporting it empty while claiming 42 cars seen on it.
- **A union of intervals across frames inflates occupancy.** ~1 m projection scatter means one car's
  interval grows with every view — worst exactly where the evidence is best. A strict-majority
  per-cell vote (deliberately the same rule as `detect_occupancy.vote()` and `_RECOMPUTE_SQL`, so
  the two layers stay comparable) moved occupancy **83% -> 60%**.
- `observed_fraction` read 1.01 — the cell grid overran the run's end.
- Runs reporting "all free" while detections landed on them were silent; now counted, the same
  posture as the `unassigned` counters in §2g.

**Capacity comes from Sofiaplan's point COUNT and must never be recomputed as length/pitch.** Every
individual coordinate in that dataset is distrusted, but the number of spaces on a stretch is what
the city actually knows and the drone cannot see. Published pitch 5.41 m vs real 4.48 m would
undercount a 35 m run by about one space. Capacity is then **scaled to the observed stretch**: a
flight that saw a third of a run cannot speak for the rest.

**Runs are clustered by GEOMETRY, never by street name** — the same rule as the street closures
(see the web tier below). A name also cannot express the unit that matters: one street's two sides
need opposite-signed corrections (the two published rows on бул. Джеймс Баучер are 12.1 m apart), so
the row-side is the minimum honest unit. 141 verified runs of >=4 bays cover 1252 of 1698 bays,
6.84 km of kerb, every bay in exactly one run.

**The observed mask is GEOMETRIC and OVERSTATES observation** — it says the kerb was in frame, not
that it was visible. Flight 0074's `frame_0050.jpg` is a street entirely under canopy and passes.
Treat `observed_fraction` as an upper bound until a radiometric occlusion test exists. Note also
that `run_summary` applies **no `MIN_VIEWS` gate** to its observed spans while `vote_cells` applies
one (3) to its own — tightening it would move `observed_fraction` and `capacity_observed` at the
same time as `free`, so it is deliberately left as a separate, single-variable question.

**Thresholds are knobs and must be tuned against ground truth, never against the layer's own
output** — that circularity is what this redesign exists to escape.

**Separation from the sim is by construction.** `process_frame` runs the run layer only when
`backend == "detector"` **and** `config.RUN_LAYER`, so the heuristic sim never writes
`run_detection`/`run_observation`/`run_state`, and `bay_votes_from_dets` is untouched by all of the
above. *Verified 2026-08-28:* golden replay **42/42 at 100%**, `detect_baseline fmi_block`
**TP=0 FP=0 FN=30 / 58.8%**, 39 consistency checks, projection self-test 0.000000 mm.
**`replay.py` still does `DELETE FROM bay_state` GLOBALLY and leaves the run tables alone**, so
running it between a real flight and a demo leaves the two layers describing different flights.
**Running it BEFORE the real flight is not enough either** — measured 2026-08-28, the golden-replay
gate left 24 `fmi_block` heuristic verdicts in `bay_state`, the real flight reclaimed only the bays
it could see, and 19 sim bays were still painted under the real kerbs at the end. The bay layer then
showed two worlds at once while the kerb layer showed one. Ordering does not help, because nothing
overwrites a bay the real flight never covers.
**And `pnpm clear fmi_block` is the WRONG tool for that cleanup:** `clear_area.py` resolves which
`bay_state` rows to drop as "bays this area observed", which is right when one area owns the block
and over-deletes when two overlap and the *other* one won — on this data it would have blanked the
25 bays both flights cover, all of them legitimately the detector's. The narrow fix is to delete the
stale rows by `backend` while excluding bays the winning area observed; they recompute from the
surviving observations, so it is reversible. Better still, **run the golden regressions against a
separate database from the demo** — they are a sim gate and it is a real deployment, and they should
never have shared `bay_state` in the first place.

**Web tier:** migration `0014` (`curb_run`, `run_detection`, `run_observation`, `run_state`) +
`0015`; `db/run_db.py`, `vision/runs.py`, `GET /api/v1/runs`, `CurbRunLayer.tsx` with a KERBS
toggle — and the **bays are a switchable layer too** (`showBays` on `BayMap`, a BAYS row in
`SurveyReadout` beside KERBS/AIRSPACE), because the two layers answer the same question from
incompatible geometry and disagree by design, so reading either alone is what makes the comparison
legible. Both switches are **viewing choices only**: `fc` still feeds the COVERAGE line and
`nearestFree` whether the bays are drawn or not, or hiding a layer would change what the survey
reports. The gap segments are drawn **`interactive: false`** — with `preferCanvas` every vector
shares one canvas and Leaflet's `Canvas._onClick` keeps the *last* interactive layer under the
cursor, and a gap lies exactly on its own run's bays by construction, which makes it the worst
possible case of that trap.

### 3. Flight-log analysis (real-flight debugging, separate from sim)
```bash
pip install pymavlink
python tools/analyze_log.py logs/your_flight.bin   # ArduPilot .bin: modes, GPS/EKF, commanded vs actual attitude
```

## Running Webots

Webots is installed at `C:\Program Files\Webots\msys64\mingw64\bin\webots.exe`. To run a world headless and capture controller output:
```bash
"/c/Program Files/Webots/msys64/mingw64/bin/webots.exe" --batch --mode=fast --minimize --stdout --stderr worlds/fmi_block.wbt 2>&1 | grep -aE "reached|complete|t=" | head -1500
```
Critical run-time gotchas (each cost real debugging time):
- **Capture stdout by PIPING, not file redirect.** Webots block-buffers stdout to a file and loses it when the process is killed (timeout). Piping to `grep`/`head` flushes line-by-line; `head -N` also stops the otherwise-infinite controller loop.
- **Kill stray Webots first.** A leftover instance causes a port conflict and the next run hangs with zero output: `taskkill //F //IM webots-bin.exe; taskkill //F //IM webotsw.exe; taskkill //F //IM python.exe`.
- The controller writes `frame_###.png` / `poses.json` to `sim/output/<survey_area>/` (survey area name derived from the route file passed in `controllerArgs`), so disk output is the reliable source of truth even if console is lost. Pre-2026-07-04 runs live loose in `sim/output/`.
- `EXTERNPROTO` for `Mavic2Pro.proto` is pinned to **R2023b** via `WEBOTS_VER` in `generate_world.py`; change it if your Webots release differs, then regenerate the world. Controller device names (`camera`, `inertial unit`, `gps`, `gyro`, `camera roll`, `camera pitch`, `front/rear left/right propeller`) assume that proto.

## Architecture (the big picture)

**Coordinate flow / georeferencing is the spine of the project.** Everything is tied together by a single local projection: lon/lat (EPSG:4326) → local ENU metres about the block centroid, using `mlat = 111320`, `mlon = 111320*cos(lat0)`. Both `generate_world.py` and the controller use this same convention. `poses.json` records the drone's `x, y, alt, yaw` in those metres at each captured frame — so a detected car's image position can be projected to ground metres and matched to the nearest bay in `block_bays.geojson`, then scored against `ground_truth.json`. When touching projection math, keep `generate_world.py`, the controller, and (future) the detector consistent.

**Camera model: the camera is NOT nadir, and pretending it was cost 2.4 points of accuracy**
(fixed 2026-08-20). `score_occupancy.project()` used to place a bay from `x, y, alt, yaw` alone.
It now builds the optical axis from the attitude in `poses.json` — `camera_axes()` — because the
Mavic2Pro gimbal does not do what the controller asks it to:
- its **pitch** compensation works, so only the RESIDUAL pitch `pitch + (cam_pitch - pi/2)` tilts
  the camera fore/aft — a few mrad of servo lag behind its own command;
- its **roll** compensation never reaches the image. The lateral error tracked the FULL body roll
  at slope **-1.02, R² 0.994** — exactly as if the joint were not there;
- what that joint does instead is **spin the image** about the optical axis by `cam_roll`.
All three follow from the joint order: the roll joint sits *below* the pitch joint, so once pitch
is at the +pi/2 a nadir survey flies, the roll axis has been rotated onto the optical axis. It can
no longer level the camera; it only rolls the picture. This is why `--chase`-era intuitions about
"the gimbal keeps it level" are wrong at nadir specifically.
**Result:** median per-frame misalignment **0.269 m → 0.043 m**, worst frame **0.97 m → 0.060 m**;
`fmi_block` **97.6% → 100%** (the last false positive, bay 17685, was a *projection* error all
along), `fmi_block_4st` unchanged at 100%, `fmi_block_4st_lp` 98.2% → 99.1%.
Two committed diagnostics under `vision/diag/` (the previous set lived in a session scratchpad and
was lost — do not repeat that): `paint_align.py` registers the projected outlines against the paint
actually visible in a frame, sub-pixel, and `--self-test` proves the estimator on injected known
shifts (0.10 px worst error, against a 0.63 m effect) before any number is believed; `offset_report.py` regresses the
result against the pose covariates and is what named the cause. **Re-run
`python vision/diag/offset_report.py fmi_block` after any change to the camera, the gimbal or the
capture logic** — a correct model leaves every R² near zero, and a slope near ±1 against an
`alt*angle` term names the angle being ignored.
A pose carrying no attitude still projects as nadir, so every `poses.json` already on disk stays
scorable — the same fallback discipline as `pose_idx()`'s legacy `i` key.

**`parkdrone.py` is one control loop.** Each step: read IMU/GPS/gyro → point gimbal to nadir → optionally capture a frame → compute roll/pitch/yaw/vertical disturbances → mix into four propeller velocities (mixing & base gains adapted from Webots' official Mavic2Pro sample). On top of the stock stabilizer sits a **lawnmower waypoint navigator** that steers car-style: yaw to point the nose at the next waypoint, then throttle forward.

Hard-won controller invariants — **do not regress these** (they are why the sim works now; details in the project memory):
- **Camera nadir uses POSITIVE pitch.** The Mavic2Pro `camera pitch` range is ~`[-0.5, +1.7]` rad where **down is positive**; set `+pi/2` for true nadir. `getMinPosition()` (-0.5) points the camera *up* and yields sky-only frames.
- **Altitude needs vertical-velocity damping** (`K_VD`), or it overshoots ~30→50 m and crashes.
- **Yaw needs rate damping** (`K_YAWD`, from the gyro's yaw rate), or it is pure-proportional and the drone spins in circles, never facing a waypoint.
- **Steer with yaw + forward only; never roll-strafe toward the target** — a lateral *position* command saturates while off-heading and tumbles the drone. Roll is used only to damp sideways drift.
- **Waypoint arrival: keep `WP_REACH` at 6 m and capture at closest approach.** At cruise speed the turn radius is ~4 m, so the drone can settle into a stable ORBIT inside a tighter basin (constant distance — the patrol hangs forever, circling). Arrival fires on `WP_CAPTURE` (2.5 m), receding >1 m past the closest pass, or a `WP_TIMEOUT` (8 s) orbit bail-out.
- **`TILT_MAX` > ~1.0 dips lift and crashes.** Forward speed is a velocity-target controller (`v_des = clamp(K_POS*fwd_err, 0, V_MAX)`) that ramps down on approach so row-end U-turns stay tight; once the patrol finishes the controller **lands** (see below) instead of sailing off.
- **The patrol ends with a landing, in two phases, and `yaw_d` must be DAMPED throughout.** The old
  end-of-patrol branch damped forward and lateral drift but never touched yaw: `yaw_d` was reset to 0
  every step, and zero yaw *command* is not zero yaw *rate* — there is no aerodynamic drag in the
  sim, so whatever rotation the last waypoint left behind simply persisted and the drone hovered at
  30 m turning on the spot indefinitely. The landing uses a **pure rate damper** (`-K_YAWD*yaw_rate`,
  no heading target — there is no waypoint left to face); measured, yaw then holds within 0.3° for
  the whole 35 s descent.
  Phase 1 **brakes to a hover, then latches the spot**; phase 2 descends over it. The order matters:
  the patrol ends at cruise speed and braking authority is ~0.22 m/s², so descending immediately
  means descending along a ballistic curve — measured at **6.5 m** of sideways travel before this
  phase existed, against **0.22 m** after. The descent walks a commanded altitude `alt_cmd` down at
  `LAND_RATE` (1 m/s, easing to 0.35 below 6 m) rather than stepping the target to zero, so the
  existing altitude loop tracks a ramp it can follow instead of dropping. Motors are cut at
  `LAND_CUT_ALT` and the loop then short-circuits — `setVelocity` is sticky, so cutting once is
  enough.
  Two structural notes: the landing branch runs **outside the `settled` gate** (a descent is
  unsettled by definition, and the navigator branch would otherwise stop producing any attitude
  command at all — the drone would fall with roll, pitch and yaw all commanded to zero), and
  `alt_cmd` exists precisely so `settled` and the altitude loop mean the *commanded* altitude. During
  the patrol `alt_cmd == TARGET_ALT`, so the survey path is untouched — verified by A/B flight of
  `fmi_block_4st`, 97/97 waypoints, max position delta 0.000000 m.
- Working gains live at the top of the loop: `K_YAW=1.0 K_YAWD=0.8 K_POS=0.6 K_VEL=0.4 TILT_MAX=1.0 V_MAX=2.5` (plus the Webots-sample stabilizer gains `K_VT/K_VP/K_ROLL/K_PITCH`).

## Windows / Git Bash conventions

- **Use `127.0.0.1`, never `localhost`, for anything talking to the dev server.** uvicorn binds IPv4
  only and vite binds `[::1]` only, so `localhost` is ambiguous here and a client that tries `::1`
  first eats a ~2 s connect stall on **every request** before falling back. Measured 2.050 s vs
  0.005 s on `GET /health`. The cost is fixed rather than proportional to payload, which is how to
  recognise it. Full record and the numbers in the demo section (§2d).

- The shell is Git Bash. **Windows backslash paths break** in commands — use forward slashes (`/c/Users/...`) or quote carefully.
- The data scripts deliberately use Python `urllib` + UTF-8 (`sys.stdout.reconfigure(encoding="utf-8")`) instead of shelling out, because **Git Bash mangles Cyrillic** (street names, zone types are in Bulgarian). For manual downloads use `curl --ssl-no-revoke` (Windows cert revocation is flaky); the scripts already disable cert verification for these public read-only GETs.

## Web infrastructure (`web/`) — live occupancy product

A separate stage 4 turns the on-disk occupancy report into a live product: an ingest API, a
real-time occupancy push, and an end-user parking map. It lives in **`web/`** (a pnpm monorepo),
independent of the Python sim/vision code. Full design in `docs/web_infra_plan.md`.
Diagrams: `web/docs/architecture.drawio` (system level), `docs/server_modules.md` +
`docs/server_modules.drawio` (inside the server), `docs/db_schema_er.md` (schema).

**Status: Phases 1–6 built & verified end-to-end (re-verified 2026-08-24: migrations through
`0011`, golden replay 42/42 at 100%, full-stack ingest 42/42, restart recovery reproducing both,
and a two-replica run splitting one backlog 17/17 with cross-replica delta fan-out). Phase 7 (prod
hardening) remains — horizontal scaling no longer blocks it.** See project memory
`project-web-infra.md` for the running log.

### The no-fly map is a SEPARATE product, in its own repo
`https://github.com/l-pavlova/nofly-map` (public, GPL-3.0) — a standalone static map of Bulgaria's
published UAS geographical zones with an altitude-aware "can I fly here?" check. It was built here
on 2026-08-21 out of `web-user`'s airspace layer and then **moved out entirely**; there is no copy
in this monorepo, deliberately, because two copies of the same app is the duplication hazard this
project already knows to avoid. Deployed from that repo to GitHub Pages.

What is worth knowing from here:
- It shares **no code** with PARKDRONE — no backend, no DB, no API; the zones are baked into a
  committed `public/zones.json` by its own `scripts/build-zones.mjs`.
- **ED-269 parsing now exists in two places**: that script, and
  `vision-worker/parkdrone_vision/nofly.py` here. They emit different shapes on purpose (48-gon
  circles for an API consumer vs native centre+radius for a downloaded asset), but the *rules* —
  split a zone's several altitude volumes into several entries, prefer the AUTHORIZATION contact,
  read `permanent` off `applicability` — are copied. Same risk class as the constants in TODO #3,
  and `tools/check_consistency.py` does not cover it. If the CAA republishes, both need the new file.
- The airspace layer that stayed here (`web-user/src/components/NoFlyLayer.tsx` + `/api/v1/nofly`)
  is unaffected and is documented below.

### Layout
- `packages/contracts` — shared TS types + zod schemas + the ENU projection (mirrors
  `generate_world.py`/`score_occupancy.py`; **must** stay in lockstep — same ORIGIN/MLAT/MLON).
- `packages/db` — Postgres+PostGIS migrations and the geojson→`bay` seeder (dev tooling, run via
  `pnpm db:migrate`/`db:seed`; not on the runtime path).
- `apps/vision-worker` (Python) — **the whole server** (FastAPI monolith), organized by concern
  into subpackages: `api/` (routes in `app.py`, drone auth, the WebSocket `hub.py`), `processing/`
  (`jobs.py`'s in-process queue + classify threads, `pipeline.py`'s per-frame `process_frame`),
  `db/` (`pool.py`'s connection pool, `vision_db.py` for bay geometry/observations/bay_state,
  `web_db.py` for reads/ingest/mission/auth SQL), and `vision/scoring.py` (the classifier bridge).
  Also serves **`GET /api/v1/nofly`** (`nofly.py`): the Bulgarian CAA's published UAS geographical
  zones (ED-269 JSON in `data/bgr_zones_<ddmmyyyy>/`, 881 zones in the 30-07-2026 edition) as
  GeoJSON, filtered by `bbox`/`restriction`. **Deliberately not in Postgres** — it is static national
  reference data that changes when the CAA republishes, there is nothing to join it against and
  nothing to update transactionally, so it is parsed once and cached in memory, the same posture as
  `vision/ground_truth.py`. It lives on the server rather than in the frontend because the file sits
  in `data/` next to the sim and the browser cannot reach it. ED-269 geometry is a `Circle`
  (centre + radius in metres) or a `Polygon` and GeoJSON has no circle, so circles are emitted as
  48-gons (worst radial error <0.2% of the radius) with `circle_radius_m` kept in the properties;
  one zone with several altitude volumes becomes several Features, because drawing them as one
  shape would report the wrong ceiling. A missing file yields an empty layer, not a 500.
  `config.py`, `s3.py`, and the CLI entry points (`server.py`, `replay.py`, `replay_ingest.py`,
  `register_drone.py`) stay at the package root. Reuses `vision/score_occupancy.py`'s
  `project`/`bay_features`/`classify` **verbatim** (via `vision/scoring.py`/`processing/pipeline.py`),
  and adds the web edge: ingest (`POST /api/v1/ingest/frame`, per-drone API key), read
  (`/api/v1/bays` GeoJSON, `/summary`, `/bays/:id`), `WS /ws/occupancy` push, and the dev toggle.
  Ingest → in-process `queue.Queue` → classify threads → `bay_state` + direct WebSocket push.
  Entry point `parkdrone_vision.server` (uvicorn on :4000, serving `parkdrone_vision.api.app:app`).
- `apps/web-user` (React + react-leaflet) — the parking map (drone "survey-readout" UI identity).
  It also draws the **published UAS airspace** over the block (`NoFlyLayer.tsx`, fed by
  `GET /api/v1/nofly`), toggled from a row in `SurveyReadout`. **Zones render BEFORE the bays, and
  that is load-bearing:** with `preferCanvas` every vector shares one canvas and Leaflet's
  `Canvas._onClick` keeps the *last* interactive layer under the cursor, so a zone drawn after the
  bays would swallow every bay click inside it — the same trap the 25 m accuracy circle fell into.
  Drawing them first also puts the hazard shading under the data, which is the right visual order.
  The overlay is fetched **once** for a ~6 km box around the block rather than per viewport: it is
  reference data, so re-fetching on pan would re-download identical polygons. A failed fetch is
  swallowed deliberately — the map is fully usable without it and the switch simply stays hidden.
  Styling convention: **CSS Modules, one `Component.module.css` per component** (no shared
  per-component classes in `styles/global.css` — that file is trimmed to CSS variables/reset/base
  sizing only). Use `:global(...)` only for classes owned by a third party we don't render
  ourselves (e.g. Leaflet's injected `.leaflet-popup-content`).
  **The basemap is SELF-HOSTED vector tiles** (2026-08-28): `public/sofia.pmtiles`, a 14 MB ~20 km
  bbox cut from the ODbL Protomaps planet build and committed, drawn by `protomaps-leaflet`'s
  `leafletLayer` through the `ProtomapsLayer` wrapper in `BayMap.tsx`. Every *hosted* raster
  basemap tried was withdrawn or unusable — CARTO's `light_all` now watermarks `API KEY REQUIRED`
  without an account, plain OSM raster is a general-purpose map competing with the data drawn over
  it, and Esri's `World_Light_Gray_Base` stops at z16 and was visibly pixelated at the z17–19 this
  map runs at. **Vector is what makes a z15 archive legal at z19:** the geometry is re-rasterized
  at display resolution, so detail thins above z15 (correct) but sharpness does not (which is
  precisely what the raster attempt could not do). Regenerate with the `pmtiles extract` command in
  `web/README.md`; build dates rotate weekly. Two things to keep right: `maxZoom={19}` lives on
  `MapContainer` because it used to come from the deleted `TileLayer` and its loss un-clamps zoom
  silently, and the layer's default `Protomaps © OpenStreetMap` attribution is an **ODbL
  requirement**. It is an `L.GridLayer` in the *tile* pane, so unlike every vector overlay here it
  never joins the shared `preferCanvas` canvas and cannot swallow a bay click.
- `apps/web-admin` (React, no map) — the **ops dashboard** on :5174: pipeline health, ingest rate,
  fleet/mission progress, coverage. Same visual identity and CSS-Modules convention as `web-user`;
  no shared component package yet (the two apps overlap only in CSS variables — copy, don't
  abstract, until a third consumer exists). It reads **only** `GET /api/v1/metrics`, polled every
  3 s (`hooks/useMetrics.ts`); the sparkline series is accumulated client-side from those polls,
  which is why it is labelled "since this page opened" — the endpoint returns gauges, not history.
  Health thresholds (stall/failure/idle) live in one place, `lib/format.ts`, so the tiles, the
  fleet table and the alert banner cannot disagree.
- `infra/docker-compose.yml` — postgis + minio.

### Architecture invariants (do not regress)
- **Postgres + PostGIS from the start** (no SQLite). Bay geometry is WGS84; bbox/nearest queries
  push down into PostGIS. The ENU projection is only for pose math.
- **One Python process owns everything** (web edge *and* CV) because the classifier is Python and
  reused verbatim. No cross-language boundary, so **no broker**: the job queue is an in-process
  `queue.Queue` drained by dedicated classify threads (numpy releases the GIL, so real parallelism
  off the event loop), and deltas are pushed **straight** to WebSocket clients the same process
  holds. Read/ingest handlers are sync `def` (Starlette threadpool) so blocking psycopg2/boto3 never
  touch the loop; only the WS endpoint is async.
- **Durability without a broker:** an in-memory queue loses in-flight jobs on restart, so on startup
  the server re-enqueues jobs with `status='queued'` (rebuilt by joining `frame_job` to `frame`,
  plus S3). A frame whose image has expired from the store is marked `failed` so recovery won't
  loop on it.
- **`frame` and `frame_job` are one thing each.** `frame` is the append-only ingest ledger (pose,
  payload pointer, provenance — never UPDATEd); `frame_job` is the 1:1 classify work state
  (`status`, `enqueued_at`, `finished_at`). Both are written in one transaction before the job is
  enqueued; a duplicate ingest creates no job row. `ON DELETE CASCADE` means clearing a survey
  area's `frame` rows still clears its jobs.
- **The pose stored with a frame must include the GIMBAL angles** (`cam_pitch`, `cam_roll`,
  migration `0008`), not just body roll/pitch. `project()` needs them (see **Camera model**), and
  `web_db.claim_frames` rebuilds a claimed job's pose from the `frame` row — without them a job
  picked up after a restart (or by another replica) would be re-projected as if the camera were
  nadir and could classify differently from the same frame processed live. Nullable: pre-0008 rows and any drone that reports no gimbal fall back
  to nadir, exactly as they did before.
- **A street CLOSURE overrides the camera at READ time; `bay_state` is never touched** (migration
  `0012`, 2026-08-25, TODO #13). `street_closure` is a polygon + a real `[valid_from, valid_to)`
  window (NULL = open at that end), and `web_db._CLOSED` is a second shared SQL fragment applied
  beside `_FRESH` at all four read paths (`feature_collection`, `summary`, `detail`,
  `coverage_counts`), emitting `closed` + a `closure` info object. Read-time derivation is the
  pattern this codebase already chose for freshness, and for the same reason.
  **`process_frame`, `insert_observations` and `_RECOMPUTE_SQL` are deliberately unchanged**:
  frames over a closed street are still classified and still write `observation` rows. The closure
  overrides the published *answer*, not the *record* — which is what keeps it falsifiable and what
  lets a closure be lifted without re-flying the street. *Verified with teeth: 143 observation rows
  on the affected bays before, 143 after.* A third state was NOT put in `bay_state`: `occupied` is
  `NOT NULL` and `source` carries a CHECK, and writing "closed" there would assert into the camera's
  own record a fact the camera never observed.
  **It is NOT gated on freshness**, unlike `occupied`/`backend`: a closure is asserted by an
  authority rather than observed by a drone, so it does not go stale when the flight does — it ends
  when its own validity window ends. `bayStatus()` on the client therefore checks `closed` *first*,
  before the freshness test, or a live closure would hide behind an expired observation.
  Counts: `summary` and `coverage_counts` give `closed` its own bucket taken out of the other three
  (`bays_unknown` is still derived by subtraction, so it has to be), because folding it into
  `unknown` would say the survey failed to see the bay and into `free` would advertise parking that
  does not exist. Manage them with
  `python -m parkdrone_vision.closures load|list|end|rm` — `load` reads the same
  `data/closures.geojson` the generator does and publishes a `bay_delta` per affected bay, so the
  map repaints live on every replica through the existing channel. `GET /api/v1/closures[?all=true]`
  serves them as GeoJSON.
  **Known limit, by design:** a closure expiring on its *own clock* pushes no delta — nothing runs
  at that instant to notice — so such a bay corrects itself on the client's next fetch or reconnect.
  Same already-accepted limitation as a `bay_state` row crossing `OCCUPANCY_WINDOW_S`.
  *Verified 2026-08-25:* summary 25/24/0 → 24/20/5 → back on lifting (reversible, idempotent), 5
  live WS deltas carrying `closed`, golden replay **42/42**, full-stack ingest **49 deltas, 42/42**
  — i.e. the override is completely inert when no closure is loaded.
- **Bay ids: int in `block_bays.geojson`, string everywhere in the web tier**.
- **Frame ingest is idempotent per MISSION, not per survey area** (migration `0009`, 2026-08-21):
  the unique key is `(drone_id, COALESCE(mission_id, 'area:'||survey_area), frame_idx)`. It used to
  be `(drone_id, survey_area, frame_idx)`, and `frame_idx` restarts at 0 every flight — so a
  re-flight of an area posted frames that came back 200-duplicate and did *nothing*: no job, no
  `bay_state` change, no delta, a map that never moved for a whole patrol, and no error anywhere.
  The only fix was to DELETE the history first, which is backwards and impossible in production
  where `observation` IS the record. A re-send **inside** one flight is still a duplicate (the
  retry case the constraint exists for); a new flight is new data. The `COALESCE` sentinel matters:
  `mission_id` is nullable and NULLs are distinct in a unique index, so without it a mission-less
  frame would lose deduplication entirely — instead it falls back to exactly the old behaviour.
  **The stored-image key carries the mission too**, or a re-flight would overwrite the earlier
  flight's image while that flight's `frame` row still pointed at it.
- **Occupancy is a vote over a freshness window, and inside it the NEWEST MISSION WINS the bay**
  (`OCCUPANCY_WINDOW_S`, default 2 h). Only observations inside the window count, and a `bay_state`
  row older than it is reported as `occupied: null` (unknown) by every read path — derived at read
  time, not swept. The window **must exceed the survey period**, or a long patrol expires its own
  early bays before it lands (the 1 km route is >60 min). Since re-flights ingest (above), two
  flights can fall inside one window, so the vote first resolves which mission saw each bay last
  and counts only that flight's views — otherwise a bay that emptied between flights would keep
  voting "occupied" on the strength of history. Earlier observations stay as history, which is what
  they are for. Legacy rows with a NULL mission group together (`IS NOT DISTINCT FROM`), so
  pre-`0009` data votes exactly as it did before.
  **`observed_at` stays `DEFAULT now()`** (decided 2026-08-22) — the vote windows on *ingest*
  time, and uploaded footage is rebased to now before it is sent (`dji_srt --as-now`, §2b)
  rather than carrying its own capture time into the vote.
  *Verified with teeth:* 20 contrary views from an older mission leave the state untouched; the
  same 20 rows re-labelled to the newest mission flip it.
- **Frames are transient.** `cleanup.py` deletes `frame` rows and their stored images past
  `FRAME_RETENTION_S` (4 h); `observation` and `mission` are kept as the analytics history. It
  refuses to collect a frame whose job is still `queued` — that is unclassified work, and dropping
  it silently would hide a stalled pipeline.
- **N replicas are safe (migration `0011`, 2026-08-24). Work is CLAIMED, not pushed.** Until this
  a single replica was a correctness *requirement*: `jobs.recover()` re-enqueued every
  `status='queued'` row with no ownership filter, so two replicas both drained the whole backlog and
  double-counted the vote, and the WebSocket hub and its replay cursor were per-process. Three
  changes, all in Postgres — no Redis, no broker, the DB stays the only shared state:
  * **Claiming.** `web_db.claim_frames` takes rows with `FOR UPDATE SKIP LOCKED` and stamps a
    `claimed_by` / `lease_expires_at` lease (`frame_job` also gained `attempts` and `last_error`).
    One dispatcher thread per replica (`processing/jobs.py`) claims into the existing local
    `queue.Queue`; the classify threads are unchanged. **`recover()` is gone**, and that is an
    upgrade rather than a removal: the claim query's second arm takes any lease that stopped being
    renewed, so a dead replica's work is picked up by a *live* one within `LEASE_S` instead of
    waiting for the dead process to restart. Recovery stopped being a startup step.
  * **`observation` is idempotent per `(frame_id, bay_id)`.** Claiming stops two replicas doing one
    frame at once; it does not stop the same frame being done twice in *sequence*, and that window
    is real even on one replica — `process_frame` commits the observations, then
    `mark_frame_processed` commits separately, so a crash in between re-scores the frame. A partial
    unique index is the only place that can actually be guaranteed. **Measured:** re-running all 34
    frames of `fmi_block` leaves the observation count at 87, not 174.
  * **Deltas cross replicas over `LISTEN`/`NOTIFY`, ordered by a `bay_delta` table.** `process_frame`
    publishes in the **same transaction** as the state it announces — NOTIFY fires on COMMIT, so
    nothing is ever announced for state that rolled back. Every replica's listener thread
    (`api/delta_listener.py`) delivers to its own clients, and a replica hears **its own** deltas
    back the same way: one delivery path means every client sees one order. `bay_delta.id` is the
    cursor `?since=` resumes from, so it means the same thing on every replica and survives a
    restart (the old counter began at 0 each boot). Replay reads a little *behind* the cursor
    (`DELTA_REPLAY_SLACK`) because sequence ids are assigned before commit and can become visible
    out of order; a delta is an idempotent "set bay X to this state", so over-replaying is free and
    missing one is not.
  Local `queue_depth` is now this replica's **prefetch** (`CLAIM_PREFETCH`, kept deliberately
  shallow — a replica that claims the whole backlog holds leases on work it will not start for
  minutes); the shared backlog is `jobs.queued`, and `expired_leases` is the new "a replica died
  mid-frame" signal. Throughput is still not the reason to scale out (~103 frames/s per classify
  thread against ~0.5 frames/s per drone) — availability and rolling deploys are.
  **Verified 2026-08-24 with two replicas** (A on :4000, B on :4001, one Postgres, one MinIO):
  a staged 34-frame backlog split **17/17**, zero duplicate `(frame_id, bay_id)` observations, and
  both replicas' `/api/v1/bays` matched the golden fixture 42/42 at 100%. Deltas published by A were
  delivered in full (49/49, cursors identical) to a client on **B**. A resume with A's cursor against
  B replayed correctly and both reported the same `snapshot_cursor`. Twelve jobs left `running` by a
  vanished replica were reclaimed 8-then-4 (the prefetch gate) and finished with no duplicates. And
  the single-replica path is **unchanged, proved by A/B rather than by argument**: the same E2E on
  the stashed pre-change code returns the identical `/api/v1/metrics` model block
  (`views_scored 524, view_accuracy 0.9695, bays_scored 49, state_accuracy 0.8571`) and the same 49
  WS deltas. *(The "52 deltas" figure below is stale — it predates the 2026-08-20 camera-model fix.
  49 is exactly the number of `bay_state` transitions in the observation history, checked in SQL.)*
  A job that fails now stops: a missing image is terminal on the first attempt, anything else is
  released for a retry and given up on after `MAX_ATTEMPTS` with `last_error` recorded — before
  this, such a job stayed `queued` forever and was re-run on every restart.
- The server classifies **all** visible bays (production has no ground truth); `gt` is eval-only.
  Labels are resolved inside `processing/pipeline.py` from
  `sim/worlds/<area>.ground_truth.json` (`vision/ground_truth.py`, `GROUND_TRUTH_ROOT`, cached
  including misses, survey-area name whitelisted since it reaches a file path from HTTP). An area
  with no file records `gt = NULL` and reports accuracy as unknown — that is the production case,
  not a failure.
- `score_frame` takes an optional `BayIndex` and rejects bays outside the camera footprint before
  projecting them (footprint half-width is `alt*tan(FOV/2)`). Results are identical — the test is
  conservative — but per-frame cost stops scaling with the size of the bay dataset.

### Run it (dev)
One command brings the whole stack up for a full-app test — infra, migrations, seed, server, both
UIs (driver map :5173, ops dashboard :5174), plus a registered dev drone whose API key lands in
`web/.quickstart/drone-1.key`:
```bash
cd web && npm i -g pnpm    # corepack isn't on PATH here
pnpm quickstart            # --replay drives a survey through the live stack,
                           # --clear <area> wipes it first so a re-flight ingests
                           #   (bare --clear takes the area from --fly/--uplink),
                           # --no-admin skips the ops UI, --no-web skips both UIs,
                           # --stop tears everything down
```
It is idempotent and self-healing: it creates `.env` with generated credentials on first run,
waits on real readiness probes (`pg_isready`, MinIO health, `/health`), and kills whatever stale
process is still holding :4000 / :5173 (announcing it). Ctrl-C stops the server and dashboard and
leaves the containers up; logs are in `web/.quickstart/`.

The manual equivalent, step by step:
```bash
cd web && cp -n .env.example .env   # REQUIRED: compose has no baked-in credentials,
                                    # it interpolates POSTGRES_*/S3_* from .env and
                                    # fails loud if they're unset
pnpm install
pnpm infra:up           # postgis + minio (needs Docker Desktop running)
pnpm db:migrate && pnpm db:seed         # loads all 1698 bays
# Server (:4000) — the whole web edge + in-process vision, one uvicorn process.
# Needs fastapi/uvicorn/websockets/python-multipart + numpy/Pillow/psycopg2/boto3
# (pip install -r apps/vision-worker/requirements.txt):
(cd apps/vision-worker && python -m parkdrone_vision.server &)
# Dashboards (:5173 driver map, :5174 ops — both proxy /api + /ws to :4000):
(cd apps/web-user && pnpm exec vite &)
(cd apps/web-admin && pnpm exec vite &)
```
Verification harnesses (all Python, run from `apps/vision-worker`):
- Vision golden test: `python -m parkdrone_vision.replay fmi_block`
  (expect 42/42 match vs `occupancy_results.json`, **100%** vs GT since the camera-model fix).
  Imports the classifier only —
  no server needed. The fixture was re-flown 2026-08-11; the older "43/43, 100%" figure came from
  frames captured 2026-07-06 that were re-scored, never re-flown, after the scenery landed, so it
  no longer reproduced; the 95.2% and 97.6% figures that followed it are both superseded by the
  camera-model fix below.
- Restart recovery (what migration `0008` protects): stage frames with the server started as
  `CLASSIFY_THREADS=0`, kill it, start it normally — the startup banner reports the staged count as
  "N frames claimable", the dispatcher claims them, and the resulting `bay_state` must match the
  live-path result exactly. (Since `0011` the same test works with a *second* replica doing the
  claiming, which is the stronger version of it.) Verified 2026-08-20
  (34 frames, 42/42 at 100% both ways). It is a real check, not a formality: stripping
  `cam_pitch`/`cam_roll` from a pose moves bay 17685's projected outline by 0.35–1.55 m on the
  frames that see it, against a 0.21 m core-crop clearance.
- Full stack E2E: register a drone `python -m parkdrone_vision.register_drone drone-1`, then
  `API_KEY=<key> python -m parkdrone_vision.replay_ingest fmi_block` (expect **49** WS deltas +
  final `/bays` matching the offline result, 42/42 — 49 is the number of `bay_state` transitions the
  observation history actually contains; the 52 on record here predates the camera-model fix). To re-run, clear the survey area first
  (`pnpm clear <area>`, which is exactly this): `frame` because idempotency skips duplicates, and
  `observation`/`bay_state` because deltas only fire on a *change* — replay straight after the golden test
  leaves the state already correct and reports a green "0 deltas".
- Frame retention: `python -m parkdrone_vision.cleanup` runs one sweep by hand (the server also
  runs it every `CLEANUP_INTERVAL_S`; set that to 0 to disable). Exits 1 if it found frames past
  retention still queued, so a scheduler surfaces a stalled pipeline.

### Operational metrics (`api/metrics.py`, the P6 admin data source)
`GET /api/v1/metrics?window_s=300` (JSON) and `GET /metrics` (Prometheus text, no client library)
render one snapshot with two halves: **in-process** counters from `processing.jobs.stats()` — queue
depth and in-flight, which exist only in this process's `queue.Queue`, plus lifetime
classified/failed/recovered/deltas and mean classify time (they reset per process, by design) — and
**durable** queries in `db/web_db.py`: `frame_job` status counts (`queued` = unclaimed by anyone,
`running` = claimed and being classified) + `oldest_queued_age_s` (the stall signal — deliberately
*unclaimed* work, since counting claimed jobs as backlog would make a healthy pipeline look stalled)
+ `expired_leases`, ingest rates, enqueue→finish latency avg/p50/p95 + failure rate, fleet/active-mission
progress, bay coverage, and **model accuracy** (`state_accuracy` = voted bay verdicts vs `gt`, the
product-level number; `view_accuracy` = single looks before voting; both `null` where there is no
ground truth, never 0). **Both endpoints require `x-admin-key` when `ADMIN_API_KEY` is set** (P7, 2026-08-21). They expose
queue depth, ingest rates, fleet state and model accuracy — an operational map of the system, which
is a different thing from a bay's occupancy being public. With the variable unset they stay open
and the server prints a loud one-line warning at startup, so "unset" cannot quietly pass for
"secured"; `.env.example` ships it empty, i.e. local dev is unchanged. The ops dashboard's vite
proxy reads `web/.env` and injects the header server-side, so the key never reaches the browser
bundle and :5174 works either way. A shared key is the smallest thing that closes the door — a real
admin login (sessions, users, audit) is still open P7 work. To see a stall by hand: run the server with `CLASSIFY_THREADS=0`, ingest,
and watch `jobs.queued` / `oldest_queued_age_s` climb (`CLASSIFY_THREADS=0` starts no dispatcher
either, so nothing is claimed); restarting normally then reports the backlog as "N frames claimable"
and drains it. Starting a *second* replica instead drains it just as well — which is the point.

### Live sim uplink (watch a flight land on the map in real time)
`sim_uplink.py` is a **sidecar**, not part of the server: it watches
`sim/output/<area>/` and POSTs each frame as the flight writes it, so the map updates while the
drone is still flying instead of after a manual `replay_ingest`.
```bash
cd web && pnpm quickstart --fly fmi_block_4st     # stack + uplink + START the flight headless
cd web && pnpm quickstart --uplink fmi_block_4st  # stack + uplink; you start Webots yourself
# or standalone, next to a running Webots flight:
cd web/apps/vision-worker && API_KEY=$(cat ../../.quickstart/drone-1.key) \
  python -m parkdrone_vision.sim_uplink fmi_block_4st --idle-exit 60
```
`--fly <world>` runs `sim/worlds/<world>.wbt` headless (`--batch --mode=fast --minimize`) and
implies `--uplink <world>`, since the controller keys its output folder off the route file — the
survey area *is* the world name. It kills stray Webots first, pipes Webots' stdout instead of
redirecting it (the block-buffering gotcha above), stops it by image name on Ctrl-C/`--stop`, and
gives the uplink `--idle-exit 120` so the mission closes when the patrol ends. Two guards, both
checked before anything starts: a missing world fails in 0.2 s, and it **refuses to fly a world
whose `sim/output/<world>/` holds `occupancy_results.json`** — that is a scored golden fixture and
a new flight would overwrite the frames it was computed from (move the folder aside to opt in).
Note a folder with existing captures makes the controller **resume**, not re-fly.
- **The controller stays offline by design.** `parkdrone.py` is one control loop; a blocking POST
  inside it costs physics steps, and a hung server would fly the drone into a wall. It keeps
  writing frames + `poses.json` to disk and knows nothing about the web tier.
- **What makes it race-free:** the controller saves `frame_###.png` *before* appending the pose and
  rewriting `poses.json`, so a pose appearing in the file proves its image is complete. The
  rewrite itself is non-atomic, so a partial read is normal and simply retried next poll.
- Flags: `--idle-exit S` (close the mission and exit after S seconds with no new frame),
  `--poll S`, `--from N`, `--once`. `frames_expected` comes from `sim/worlds/<area>.route.json`, so
  the ops dashboard's mission progress bar is a real plan-vs-actual.
- **Re-flying the same survey area DOES reprocess** since migration `0009` — each run starts a new
  mission and ingest is keyed per mission, so a second flight ingests, classifies and repaints
  without anything being cleared first. Duplicates now mean what they say: the same frame re-sent
  inside one flight.
- Its stdout is UTF-8 **and line-buffered**: it runs for the length of a patrol (>1 h on the 1 km
  route) with its output redirected, and Python block-buffers a redirected stream — same lesson as
  the Webots stdout gotcha above.

### Clearing a survey area before a re-flight (`pnpm clear`)
```bash
cd web && pnpm clear                    # list stored areas, delete nothing
pnpm clear fmi_block                    # DB rows + stored images + sim/output captures
pnpm clear fmi_block --db-only          # keep the captures on disk
pnpm clear fmi_block --yes              # no confirmation prompt
```
`scripts/clear.sh` -> `parkdrone_vision/clear_area.py`; `pnpm quickstart --clear --fly <world>`
runs it as part of launching a flight (between the migrations and the server start, so startup
recovery cannot re-enqueue jobs whose frames are about to go).
**Since migration `0009` you no longer need this to re-fly** — a re-flight is a new mission and
ingests on its own. It is now for what its name says: wiping an area, e.g. to re-run a scored
comparison from a clean slate or to drop a bad flight. Four deletes have to happen together, which
is why it is a command and not a snippet: `frame` (CASCADE takes `frame_job`) + its object-store
images, `observation` (stale votes out-vote the new looks), `bay_state` (an already-correct state
pushes no delta), and `mission` (stale progress on the ops dashboard). `bay_state` has no
`survey_area` column, so the rows to drop are resolved from the observations **before** those are
deleted. On disk it removes `frame_*.png`/`snap_*.png`/`poses.json`/`flight_log.csv` — the
controller *resumes* from `poses.json`, so leaving it means continuing the old patrol instead of
re-flying — and **refuses an output folder holding `occupancy_results.json`** (a scored golden
fixture; `--force` opts in). A running `sim_uplink` re-posts by itself once `poses.json` restarts at
a lower index; after `--db-only` it does not, so restart it.

### Dev/test occupancy toggle (drive the dashboard by hand)
Manual override endpoints (mounted only when `ENABLE_DEV_ROUTES=true` — they have no auth, so the
default is off; `.env.example` opts local dev in) upsert `bay_state` and push a
delta straight to the WebSocket hub, so the map updates live — no drone/vision needed:
```bash
curl -X POST http://localhost:4000/api/v1/dev/occupy   # occupy the bay nearest FMI (default 17596)
curl -X POST http://localhost:4000/api/v1/dev/free     # free it again
curl -X POST "http://localhost:4000/api/v1/dev/occupy?bay_id=17571"   # target a specific bay
```
Both return `{bay_id, occupied, updated_at}`; watch the bay flip red/green on :5173.

Gotchas: native Windows Python needs `D:/...` paths, not Git Bash `/d/...`. The server binds :4000;
free it by PID (`netstat -ano | grep :4000` → `taskkill //F //PID <pid>`) rather than blanket-killing
`python.exe` (also kills sim Python). `CLASSIFY_THREADS=0` starts the server without draining the
queue (used to stage frames for the restart-recovery test).

## Known open issues

**How the occupancy classifier works, with the feature units spelled out:
`docs/occupancy_classifier.md`.** The five thresholds are bare numbers in an 8-bit
colour space and nothing in the source says so — `core_chroma > 12.7` is a mean per-pixel
(max channel - min channel), i.e. ~5% of full scale away from pure grey. That doc also
carries the four-step mechanism, the calibration provenance, and why the low-sun collapse
has no normalisable fix. The learned detector's own record is `docs/training.md`.

**Off-nadir projection FPs on `fmi_block` — FIXED 2026-08-20, see "Camera model" above.**
The world now scores **100%** (was 95.2%, then 97.6% after the `CORE_W` change). Root cause was
`project()` assuming a nadir camera: the gimbal's roll joint sits below its pitch joint, so at the
+pi/2 pitch this survey flies it spins the image about the optical axis instead of levelling the
camera, and the FULL body roll displaced every frame laterally. Kept here only as the pointer —
the diagnosis, the numbers and the two committed diagnostics are in **Camera model** above.

**A re-flight cannot ingest itself — `pnpm clear <area>` is a workaround (TODO #10).** Ingest is
idempotent on `(drone, survey_area, frame_idx)` and `frame_idx` restarts at 0 every flight, so a
second flight of the same area is silently ignored end-to-end: no job, no delta, a map that never
moves, and no error. Wiping the history to record new state is backwards and no real deployment
can do it. The fix is to key ingest per `mission_id` and to decide what the vote means when two
flights fall inside one `OCCUPANCY_WINDOW_S`.

**The 1 km survey is FLOWN AND SCORED (2026-08-21): 1976/1976 waypoints, 98.4%** — TP=692 TN=799
FP=25 **FN=0**, 1516 of 1593 bays classified, 77 uncovered. All 25 errors are false positives and
all are explained by the *data*, not the classifier: **14** are bays whose centroid falls inside an
OSM building footprint (the camera sees the roof — `bright` 165-198, `std` 4-12, and 14 of those 20
bays are wrong, a 70% error rate), **7** sit within 3 m of a building whose wall leans into the crop
by parallax at 30 m, and **4** are bays whose rectangle *overlaps* a neighbour holding a car, where
the ground truth itself is ambiguous. Ordinary street bays: **11 wrong of 1463 (0.75%)**; excluding
the building-covered bays the world scores **99.7%**. Two data facts fall out: **20 bays sit under
buildings** and are unscoreable by construction, and **222 bay pairs overlap** across the 1698-bay
dataset. The predicted bays-on-grass effect **did not occur** — all **33** bays on green polygons
classified correctly, because the bay pad (z=0.04-0.06) paints over the green (z=0.005).

**Longer-standing, unchanged:** capture scatter leaves ~3 bays uncovered on `fmi_block`; the learned
classifier v2 is not started.
