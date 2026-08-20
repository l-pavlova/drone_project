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
```
This reads `../data/block_bays.geojson` (plus `block_roads.geojson` and `block_areas.geojson` if present), projects to local metres, and emits the world (ground, OSM streets as Webots `Road` protos, painted bays, real car models — 7 vehicle Simple protos — on a known-occupancy subset, scenery, follow-drone viewpoint) plus `ground_truth.json` (bay_id -> occupied). Car/model randoms come from a separate `random.Random(7)` stream and scenery cosmetics from a third (`random.Random(11)`) so `ground_truth.json` stays stable — after any change here, regenerate all three worlds and check `ground_truth.json`/`route.json` are byte-identical.

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

**Remaining:** the tower the route doubles back through still costs one 238° swing and one
`STEER_ORBIT` bail-out; `trap` is passed but only via the escalation ladder (two 150 s timeouts).

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

- The shell is Git Bash. **Windows backslash paths break** in commands — use forward slashes (`/c/Users/...`) or quote carefully.
- The data scripts deliberately use Python `urllib` + UTF-8 (`sys.stdout.reconfigure(encoding="utf-8")`) instead of shelling out, because **Git Bash mangles Cyrillic** (street names, zone types are in Bulgarian). For manual downloads use `curl --ssl-no-revoke` (Windows cert revocation is flaky); the scripts already disable cert verification for these public read-only GETs.

## Web infrastructure (`web/`) — live occupancy product

A separate stage 4 turns the on-disk occupancy report into a live product: an ingest API, a
real-time occupancy push, and an end-user parking map. It lives in **`web/`** (a pnpm monorepo),
independent of the Python sim/vision code. Full design in `docs/web_infra_plan.md`.
Diagrams: `web/docs/architecture.drawio` (system level), `docs/server_modules.md` +
`docs/server_modules.drawio` (inside the server), `docs/db_schema_er.md` (schema).

**Status: Phases 1–6 built & verified end-to-end (re-verified 2026-08-20: migrations through
`0008`, golden replay 42/42 at 100%, full-stack ingest 42/42, and the restart-recovery path
reproducing both). Phase 7 (prod hardening) remains.** See project memory `project-web-infra.md` for the running log.

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
  `config.py`, `s3.py`, and the CLI entry points (`server.py`, `replay.py`, `replay_ingest.py`,
  `register_drone.py`) stay at the package root. Reuses `vision/score_occupancy.py`'s
  `project`/`bay_features`/`classify` **verbatim** (via `vision/scoring.py`/`processing/pipeline.py`),
  and adds the web edge: ingest (`POST /api/v1/ingest/frame`, per-drone API key), read
  (`/api/v1/bays` GeoJSON, `/summary`, `/bays/:id`), `WS /ws/occupancy` push, and the dev toggle.
  Ingest → in-process `queue.Queue` → classify threads → `bay_state` + direct WebSocket push.
  Entry point `parkdrone_vision.server` (uvicorn on :4000, serving `parkdrone_vision.api.app:app`).
- `apps/web-user` (React + react-leaflet) — the parking map (drone "survey-readout" UI identity).
  Styling convention: **CSS Modules, one `Component.module.css` per component** (no shared
  per-component classes in `styles/global.css` — that file is trimmed to CSS variables/reset/base
  sizing only). Use `:global(...)` only for classes owned by a third party we don't render
  ourselves (e.g. Leaflet's injected `.leaflet-popup-content`).
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
  `jobs.recover()` rebuilds a restarted job's pose from the `frame` row — without them a recovered
  frame would be re-projected as if the camera were nadir and could classify differently from the
  same frame processed live. Nullable: pre-0008 rows and any drone that reports no gimbal fall back
  to nadir, exactly as they did before.
- **Bay ids: int in `block_bays.geojson`, string everywhere in the web tier**.
- Frame ingest is idempotent on `UNIQUE(drone_id, survey_area, frame_idx)` — a re-send does NOT
  re-enqueue; to reprocess a survey area *within the retention window*, clear the `frame` table
  first. Past `FRAME_RETENTION_S` the rows are gone anyway, so a re-send is ingested as new.
- **Occupancy is a vote over a freshness window, not over all history** (`OCCUPANCY_WINDOW_S`,
  default 2 h). Only observations inside the window count, and a `bay_state` row older than it is
  reported as `occupied: null` (unknown) by every read path — derived at read time, not swept. The
  window **must exceed the survey period**, or a long patrol expires its own early bays before it
  lands (the 1 km route is >60 min).
- **Frames are transient.** `cleanup.py` deletes `frame` rows and their stored images past
  `FRAME_RETENTION_S` (4 h); `observation` and `mission` are kept as the analytics history. It
  refuses to collect a frame whose job is still `queued` — that is unclassified work, and dropping
  it silently would hide a stalled pipeline.
- **Single replica is a correctness requirement, not a preference.** `jobs.recover()` re-enqueues
  every `status='queued'` row with no ownership filter, so two replicas would both classify the
  same backlog and double-count the vote; the WebSocket hub and its replay cursor are also
  per-process. Scaling out needs job claiming, a shared delta channel and a global cursor first —
  see `docs/web_infra_plan.md`. Throughput is not the reason to: ~103 frames/s per classify thread
  against ~0.5 frames/s per drone.
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
  `CLASSIFY_THREADS=0`, kill it, start it normally — `recovered_on_start` should equal the staged
  count and the resulting `bay_state` must match the live-path result exactly. Verified 2026-08-20
  (34 frames, 42/42 at 100% both ways). It is a real check, not a formality: stripping
  `cam_pitch`/`cam_roll` from a pose moves bay 17685's projected outline by 0.35–1.55 m on the
  frames that see it, against a 0.21 m core-crop clearance.
- Full stack E2E: register a drone `python -m parkdrone_vision.register_drone drone-1`, then
  `API_KEY=<key> python -m parkdrone_vision.replay_ingest fmi_block` (expect 52 WS deltas + final
  `/bays` matching the offline result, 42/42). To re-run, clear the survey area first
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
**durable** queries in `db/web_db.py`: `frame_job` status counts + `oldest_queued_age_s` (the stall
signal), ingest rates, enqueue→finish latency avg/p50/p95 + failure rate, fleet/active-mission
progress, bay coverage, and **model accuracy** (`state_accuracy` = voted bay verdicts vs `gt`, the
product-level number; `view_accuracy` = single looks before voting; both `null` where there is no
ground truth, never 0). Both endpoints are unauthenticated like every other read route; they go
behind admin auth in P7. To see a stall by hand: run the server with `CLASSIFY_THREADS=0`, ingest,
and watch `jobs.queued` / `oldest_queued_age_s` climb; restarting normally then shows
`recovered_on_start` and drains it.

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
- **Re-flying the same survey area does NOT reprocess** — ingest is idempotent on
  `(drone, survey_area, frame_idx)`, so the uplink reports duplicates and warns once, and the map
  never repaints while the drone flies. **`pnpm clear <area>` first** (below) if you mean to score
  a new flight.
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
recovery cannot re-enqueue jobs whose frames are about to go). It exists because "re-fly and watch the map
update" silently does nothing otherwise: ingest is idempotent on `(drone, survey_area, frame_idx)`,
so the second flight's frames return 200-duplicate — no classify job, no `bay_state` change, and
deltas only fire on a *change*, so no WebSocket push. Four deletes have to happen together, which is
why this is a command and not a snippet: `frame` (CASCADE takes `frame_job`) + its object-store
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

**Longer-standing, unchanged:** capture scatter leaves ~3 bays uncovered on `fmi_block`; the 1 km
flight is unfinished and unscored (its last attempt stopped at **87 of 1976** frames on 2026-08-17,
stopped by hand rather than by any fault); the learned classifier v2 is not started.
