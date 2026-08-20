# PARKDRONE — Simulation Documentation (thesis draft)

*Draft for thesis use. Covers the problem statement, the data and world-building
pipeline, the simulation scales and their cost, the flight controller, waypoint
navigation, the evolution of the route-planning algorithm (DFS → rural postman),
obstacle detection and avoidance, and the occupancy-scoring vision stage with
its measured failure modes. Numbers are from verified simulation runs; last
revised 2026-08-20.*

---

## 1. Problem statement

Municipal paid-parking zones ("синя зона" / "зелена зона" in Sofia) are managed
with little real-time information about which individual parking spaces are
occupied. The goal of PARKDRONE is an autonomous drone that patrols a city
area, photographs the parking spaces from above, and produces a per-space
occupancy report (occupied / free) that can be scored against ground truth.

The full hardware concept (Pixhawk flight controller, Raspberry Pi 4 companion
computer, Coral USB accelerator, IMX219 camera, GPS navigation) is described in
the project master plan. The work documented here is the **simulation stage**:
everything from real municipal parking data to a simulated drone flight that
captures georeferenced nadir photographs of every parking space, ready for a
vision-based occupancy detector. Simulation-first has two advantages: perfect,
programmable ground truth, and the ability to iterate on navigation and
coverage algorithms without flight-safety and battery constraints.

The pipeline has three stages:

1. **GIS data preparation** — real parking-space data for a Sofia district is
   cut to a study area and turned into oriented parking-bay rectangles.
2. **Webots simulation** — a 3D world is generated from that data (streets,
   painted bays, parked cars with known occupancy); a simulated DJI Mavic 2 Pro
   flies a planned route over every street with bays and captures a
   downward-facing photo at each waypoint, recording its pose for later
   georeferencing.
3. **Vision / scoring** (v1 implemented) — classify each bay occupied/free
   from the captured frames by projecting the known bay polygons into the
   images, and score the predictions against the known ground truth
   (section 8).

## 2. Source data

Two datasets from the Sofiaplan municipal API are used:

- `spaces_25.geojson` — 31,705 individual paid-parking spaces as **points**,
  with Bulgarian-language attributes: street (`mestopoloz`), zone
  (`zona` — Синя/Зелена), space type (`vid_txt_20` — Зона, Инвалид, Служебен…),
  and parking orientation (`park_txt` — Надлъжн / Напречн / Косо, i.e.
  parallel / perpendicular / angled).
- `zones_34.geojson` — the zone polygons.

Street centerlines come from OpenStreetMap via the Overpass API
(`tools/get_roads.py`), fetched for the study-area bounding box and filtered to
drivable highway types.

### 2.1 Cutting a study area

`tools/cut_block.py "<address or lat,lon>" <half_size_m>` geocodes a centre
point (Nominatim) and keeps all spaces inside a square of the given half-size,
writing `block_spaces.geojson` together with the cut centre and size
(`_center`, `_half_m`). The study area used throughout is centred on
бул. Джеймс Баучер (42.67435, 23.33052); cuts of 200 m, 250 m and 500 m
half-size have been used (the 500 m cut contains 1,698 spaces on 49 streets).

### 2.2 From space points to oriented bays

Detectors and simulators need areas, not points. `tools/make_bays.py` converts
each space point into an oriented rectangle ("bay"):

- **Bearing.** Each space takes the bearing of the nearest OSM centerline
  segment of *its own street* (name-matched, 40 m snap limit). An earlier
  version estimated the bearing with PCA over neighbouring space points; on a
  wide boulevard with spaces on both sides the principal axis of that point
  cloud is diagonal, which produced bays rotated up to 45° from the street.
  With centerline bearings, the James Bourchier bays fall in 21–26°, matching
  the street exactly; curved streets correctly follow their curvature. PCA
  remains only as a fallback when no roads file is present.
- **Shape.** The rectangle is sized and oriented by the declared parking
  orientation: parallel (5.4 × 2.2 m, long axis along the street),
  perpendicular (4.8 × 2.4 m, long axis across), angled (45° to the street).

The result (`block_bays.geojson`) can be inspected on an OSM overlay
(`block_bays.html`) and a zoomed preview image.

## 3. Coordinate frame — the spine of the project

Everything is tied together by one local projection: WGS84 longitude/latitude
→ local East-North-Up metres about a **pinned origin**:

```
x = (lon − lon₀) · 111320 · cos(lat₀)      y = (lat − lat₀) · 111320
ORIGIN (lat₀, lon₀) = (42.6747105, 23.3298956)
```

The world generator, the flight controller, and the (future) detector all use
this same frame: the drone's recorded pose (`poses.json`: x, y, altitude, yaw
per captured frame) allows any pixel in a frame to be projected to ground
metres and matched to the nearest bay polygon, then scored against ground
truth.

The origin is a **constant**, deliberately not derived from the data. It was
originally computed as the bay centroid of the first study-area cut, but a
centroid origin shifts every time the data is re-cut at a different size,
silently invalidating previously flown worlds and poses. Pinning the origin
made worlds of different scales coexist in a single frame (verified: after
re-cutting the data at double size, the default world regenerated with
byte-identical routes).

## 4. World building (`sim/generate_world.py`)

`python generate_world.py [window_half_m] [occupied_fraction] [survey_area]`
reads the bay and road GeoJSON, keeps a square window of the given half-size
around the origin, and emits a complete Webots world plus its metadata:

- **`<name>.wbt`** — the world itself;
- **`<name>.route.json`** — the flight route (section 6);
- **`<name>.ground_truth.json`** — bay id → occupied (the answer key).

The default name `fmi_block` keeps legacy filenames (`route.json`,
`ground_truth.json`) for backwards compatibility.

World contents:

- **Ground plane**, sized from the window (2·half + 200 m).
- **Streets**: OSM centerlines rendered as Webots `Road` protos (asphalt,
  dashed centre line), width from OSM lane count. Segments are clipped to the
  window with Liang–Barsky clipping (OSM nodes are sparse — filtering points
  instead of clipping segments would drop streets that merely cross the
  window). Road heights are separated by a greedy slot colouring: each road run
  takes the lowest height not used by a run it overlaps, so crossing streets stay
  apart while the maximum height is bounded by the junction degree rather than by
  the number of roads. A flat stagger — height proportional to road index — was
  tried first and failed on the 1 km world, where it lifted late roads *above*
  the bay layer and painted asphalt over whole bay rows (330 vision false
  positives).
- **Green areas**: OSM `landuse=grass|meadow|forest`, `leisure=park|garden` and
  `natural=scrub|wood` polygons, triangulated (ear clipping — Webots renders
  non-convex faces unreliably) and painted at z = 0.005: above the ground and
  *below* the road ribbons, so a park that crosses a street renders under the
  asphalt and can never cover a bay. Unlike buildings they are **clipped** to
  the window (Sutherland–Hodgman), because an unclipped 1 km park would
  the whole visible ground of the 150 m world.
- **Buildings**: OSM footprints as Webots `SimpleBuilding` protos, at their real
  heights — `building:levels` where mapped, else the `height` tag (split into
  whole floors of an adjusted height so the total stays exact), else four
  floors, the Lozenets panel-block norm. A building is kept or dropped *whole*
  by its centroid rather than clipped: it is a 3D object, not ground paint, and
  a clipped footprint can come back self-touching and break the proto's roof
  triangulation. Footprints past 24 corners collapse to their PCA-oriented
  bounding box.
- **Light poles**: a mast, an arm and a head, built from primitives and walked
  along the street centerlines at an adaptive spacing (25 m on the small worlds,
  opening up on the 1 km world so the count stays bounded), one kerb-width plus
  1 m off the centerline, alternating sides. A candidate that would stand on a
  painted bay tries the other kerb and is otherwise skipped — a pole on the paint
  would be a ground-truth bug, not scene realism. They are hand-built rather than
  taken from Webots' `StreetLight` proto because that proto is a 66 KB detailed
  mesh *and* carries a live 1000 m-radius `SpotLight`; hundreds of those would
  both wreck performance and shift the daylight exposure the classifier's
  brightness thresholds are calibrated against. This is a daytime scene — lamps
  are geometry, not light sources.
- **Painted bays**: a dark asphalt pad with a thin white painted **outline**
  (~12 cm lines), like real Sofia street parking. An earlier version filled the
  whole rectangle white, which made free/occupied trivially separable by colour
  and simultaneously made white cars invisible — unrealistic in both directions.
  All the pads are emitted as one merged `IndexedFaceSet` and all the lines as a
  second one, rather than five `Solid` nodes per bay; see "The cost of a node"
  below.
- **Parked cars**: each *public* bay is occupied with probability
  `occupied_fraction` (default 0.5). Occupied bays hold one of seven real
  Webots vehicle models (Tesla Model 3, BMW X5, Citroën C-Zero, Toyota Prius,
  Lincoln MKZ, Range Rover SVR; Mercedes Sprinter van rarely and only in
  parallel bays — it is 5.9 m long).
- **The drone**: a DJI Mavic 2 Pro proto at the origin, running the
  `parkdrone` controller; the world passes the controller its route file via
  `controllerArgs`.

Two placement details matter for realism:

- **Webots vehicle protos have their origin at the rear axle,** not the body
  centre. Placing the proto origin on the bay centre offsets the body ~1.4 m
  toward the nose (and the nose direction is randomised), which made adjacent
  cars interpenetrate. The generator carries per-model dimensions and origin
  offsets (`CAR_DIMS`) and pulls each translation back along the heading so the
  *body* is centred in its bay.
- **Collision-aware placement.** Some real bay rows are tighter than the
  nominal bay (source points ~4.2–4.8 m apart on Bourchier, while a Lincoln MKZ
  is 4.93 m long). Each car's footprint (plus 0.35 m clearance) is tested
  against already-placed cars with a separating-axis test; on conflict the
  generator falls back to progressively shorter models, and as a last resort
  leaves the bay free and updates the ground truth. Verified: zero overlapping
  bodies across all generated worlds.

**Determinism.** Occupancy uses a seeded RNG (`random.seed(42)`); car model and
colour choices come from a *separate* stream (`random.Random(7)`), and the
scenery's cosmetic choices from a third (`random.Random(11)`), so neither
parked cars nor scenery can shift the ground truth. Adding the scenery was
verified this way: all three worlds' `ground_truth.json` and `route.json` are
byte-identical before and after.

**Obstacles and the flight path.** The buildings carry no bounding object by
default (`--collide` turns them on, together with the drone's range sensors).
Without avoidance the controller holds a fixed 30 m and would simply fly into the
first tall block under the route, and Webots range sensors only detect nodes that
*have* a bounding object — so the same switch is what makes structures both
solid and visible, which is why the survey worlds leave it off and the obstacle
world turns it on (section 7.5). Either way the generator reports the
conflict: buildings that reach the flight level within 10 m of the route are
printed as a warning and written to `<name>.hazards.json` (id, height, distance
to the route, centroid in the shared local metres). On the 1 km world 7
structures reach 30 m and 3 of them sit beside the route — the tallest, 42 m,
is 8.6 m from the centerline.

### 4.1 The cost of a node

The 1 km world initially took **over ten minutes and ~6 GB of RAM to load**,
which made every generator change a ten-minute experiment. Measurement — rather
than the usual guesses about rendering settings — located the cost precisely, and
two of the answers were counter-intuitive enough to be worth recording.

The scene is far outside the envelope Webots is tuned for. The most vehicle-dense
world Cyberbotics ships has 28 cars; the 1 km world had 727, in 10 065 top-level
nodes. Reading the R2023b sources explains why that is expensive: triangle meshes
and textures *are* deduplicated across instances, but **GPU buffers are not** —
`createWrenObjects()` uploads private vertex and index buffers per geometry node,
and there is no instancing path. A "Simple" vehicle proto is also not low-poly:
"Simple" means no physics, joints or interior, while the exterior mesh stays at
full detail (~20–30 k triangles). Measured cost: **5.1 MB and 0.136 s per
vehicle**, linear to at least 800 instances.

Two changes followed, each verified to leave `ground_truth.json` and
`route.json` byte-identical:

1. **Merge the bay paint.** Each bay emitted five `Solid` nodes (a pad and four
   outline boxes). A Webots `Solid` costs ~0.145 MB *however trivial its
   geometry*, so 1593 bays spent ~1.1 GB on flat rectangles. Accumulating every
   pad into one `IndexedFaceSet` and every line into a second cut the world from
   8379 `Solid`s to 416. Two details make it exact rather than approximate: the
   merged quads sit at the **top face** of the boxes they replace, not at the box
   centres, so the surface the camera sees does not move; and each quad is wound
   counter-clockwise so its normal faces the camera rather than the ground.
2. **Optional proxy cars** (`--lowpoly`): a body, cabin, windscreen and four
   wheel boxes instead of the proto. Every random draw, model choice and
   collision test upstream is left untouched, so the ground truth cannot move —
   only the emitted geometry differs.

| `fmi_block_1km`, headless | load | peak RAM | `Solid` nodes |
|---|---:|---:|---:|
| original | 282.6 s | 6359 MiB | 8379 |
| merged bay paint | 167.6 s | 5268 MiB | 416 |
| + proxy cars | **57.4 s** | **1663 MiB** | 1143 |

**4.9× faster and 3.8× smaller.** Two negative results are as useful as the
positive one. `--no-rendering` changes load cost by *exactly zero* (56.2 s vs
56.4 s on a 400-car probe): the scene graph and GPU buffers are built whether or
not the main view draws, so rendering flags are a *step*-cost lever and not a
load-cost one. And `DEF`/`USE` sharing saved only 9%, because Webots already
shares primitive meshes — the residual cost is the node itself. **Reducing node
count is the lever; reducing geometry is not.**

The methodological lesson is the one worth carrying into the thesis. An earlier
analysis measured each node type in a world containing *only* that type and
summed the costs linearly. That attributed a `Solid` ~0 s (its true ~14 ms
vanished into an 8 s baseline) and left a 5× discrepancy in load time, which was
then blamed on GPU memory exhaustion. Direct measurement refuted that outright:
video memory peaked at 1315 MiB of an available 4096 MiB and was never close to
full. **Component costs measured in isolation do not compose**; the change to the
real system has to be measured directly.

**What the speed-up is worth.** The natural yardstick for a neighbourhood survey
is the manual alternative. The 1 km route is 19.0 km of street centerline; walked
briskly at 5 km/h that is 3.8 hours, or about 4.2 with stops to write occupancy
down. Measured by the median gap between captures, the simulated survey took
**3.4 hours before this work and about 0.9 hours after** — so the simulation went
from roughly a dead heat with a person on foot to about four times faster than
one. For scale, the real aircraft would need 1.1 hours of pure flight for the
same 19 km, plus three or four battery swaps: the simulation now completes a
survey faster than the drone it simulates could fly it.

Two honest qualifications. The load-time figures were measured directly; the
per-frame flight rate is inferred from capture timestamps, and the earlier run
predates other controller changes, so the 3.9× drop in per-frame cost is
*consistent with* the node reduction rather than cleanly attributable to it. And
the walking comparison is fair on distance but slightly generous to the drone on
route structure, since a surveyor on foot sees both kerbs at once and does not
need a waypoint every 10 m.

Finally, the proxy cars answer a question the analysis had guessed at. It
predicted that a box world would be an *easier* target for the classifier. It is
not: scored on a low-poly copy of the calibration world, accuracy fell from 100%
to **98.2%**, and both misses are *dark* cars (brightness 68–70, texture
std 10–11). A uniform dark box on dark asphalt is indistinguishable from empty
tarmac, because it has none of the panel gaps, glass or under-car shadow the
heuristic relies on. So proxy cars are legitimate for coverage and logistics work
on large worlds, but **an accuracy figure measured on them is not comparable**
with one measured on the real vehicle models — and the calibrated worlds are
therefore never built with them.

## 5. Simulation scales

The same generator covers all scales; each world is an independent file set
with its own route and ground truth, and the controller writes its captures to
a per-survey-area output folder (`sim/output/<survey_area>/frame_###.png`, `poses.json`):

| world | half-size | bays | cars | buildings | greens | poles | routed streets | route | waypoints |
|---|---|---|---|---|---|---|---|---|---|
| `fmi_block` (default) | 75 m | 45 | 17 | 28 | 8 | 8 | 2 | 286 m | 34 |
| `fmi_block_4st` (test) | 130 m | 111 | 49 | 67 | 13 | 23 | 4 | 871 m | 97 |
| `fmi_block_1km` | 500 m | 1,593 | 727 | 775 | 38 | 375 | 45 (+4 PCA-fallback) | 19.0 km | 1,976 |

Two further worlds exist for specific experiments rather than for survey work:
`fmi_block_obst` (the obstacle-avoidance test course, section 7.5) and
`fmi_block_1km_lp` (the 1 km world with proxy cars, section 4.1).

The default world is the regression baseline (its route has been flown and
verified repeatedly); the 4-street world is where the classifier is calibrated
and exercises junctions, dead-end backtracks and transits at manageable runtime;
the 1 km world is the neighbourhood-scale target. At the 2.5 m/s cruise speed the 1 km patrol is
≈2 h of simulated flight; the dominant *wall-clock* cost — continuous camera
rendering — was removed (section 7.4), after which the remaining limit is
physics: with ~700 car models the simulation runs near real time on one CPU
core, so the full patrol is an hours-scale run (made restartable by the
resume capability, section 7.4).

## 6. Route planning

### 6.1 The coverage problem

The drone must photograph every bay. Bays line the streets, and the camera
footprint at 30 m altitude (~25 m across-track × ~15 m along-track for the
Mavic's 0.785 rad FOV at 400×240 px) comfortably covers a street's full width
from the centerline. The coverage problem therefore reduces to: **fly along
the OSM centerline of every street that has bays** (routes are then densified
to a waypoint every 10 m so consecutive footprints overlap along-track).

Streets are matched to OSM ways by name (bidirectional substring match, e.g.
bay street "ул. Света гора" ↔ OSM "Света гора"); a street with no OSM match
falls back to a straight PCA line fit through its bay row. Matched centerlines
are clipped to the window and become the **coverage graph** (vertices snapped
to a 0.5 m grid so shared OSM junction nodes merge); all roads (with or
without bays) form the **full road graph**, used for transit flights, so the
drone never cuts diagonally across a city block.

### 6.2 First approach: depth-first walk with backtracking

The first working algorithm was a DFS tree-walk of the coverage graph:

- start at the westernmost street end (a degree-1 node);
- at each junction, take the branch with the **smallest total unvisited
  length** first, so short dead-end side streets are covered and backtracked
  immediately before continuing along the main street;
- every backtrack is flown (appended to the route), keeping the drone on the
  road network at all times;
- disconnected street groups are reached with a Dijkstra shortest-path transit
  over the full road graph; trailing backtracks are trimmed.

This is simple, provably covers everything, and produces natural-looking
patrols. Its weakness is cost: a DFS walks its spanning tree, so in the worst
case (and typically) **every edge is flown twice** — once out, once back. On
the neighbourhood-scale window this gave a 4,212 m route for 2,915 m of
coverage, i.e. 1,297 m (+44%) of repeated flying.

### 6.3 Current approach: open-path rural postman

Covering every edge of a graph at minimum cost is the classic **Route
Inspection / Chinese Postman Problem** (Mei-Ko Kwan, 1962). Euler's theorem
gives the key fact: a walk that uses every edge *exactly once* exists iff the
graph has 0 odd-degree vertices (closed tour) or exactly 2 (open path). Street
graphs violate this — every dead end (degree 1) and T-junction (degree 3) is
odd — so some edges *must* be repeated. The postman solution chooses the
cheapest possible set of repeats:

1. **Connect** disconnected coverage components first: a minimum spanning tree
   over components, with edge costs = shortest road-path distances (multi-source
   Dijkstra over the full road graph); the connecting paths join the walk.
2. **Even out parity**: find all odd-degree vertices and pair them with a
   **minimum-weight perfect matching**, where the cost of a pair is their
   shortest road-path distance; each matched path is duplicated (= flown
   twice). Because ours is an *open* flight (no need to return to start), two
   **virtual vertices** with zero-cost edges to every odd vertex are added to
   the matching: the two odd vertices they absorb stay odd and become the
   route's start and finish — chosen optimally by the matching itself. The
   matching is solved exactly with the blossom algorithm (networkx) when
   available, with a stdlib fallback (exact bitmask DP up to 14 vertices,
   greedy + pair-swap refinement beyond).
3. **Walk**: the multigraph is now Eulerian (0 or 2 odd vertices), so a
   Hierholzer walk traverses every coverage edge exactly once, deadheading
   only along the matched duplicates. The walk starts at whichever endpoint
   is nearer the drone's takeoff point.

Since only *bay streets* must be covered while *all* roads may be used for
deadheads, this is formally the (NP-hard) **Rural Postman Problem**; at these
instance sizes the standard connect-then-match construction is optimal or
near-optimal — the matching step itself is solved exactly.

**Results** (neighbourhood window, 94 coverage edges, 18 odd vertices):

| algorithm | coverage | deadhead | total |
|---|---|---|---|
| DFS with backtracking | 2,915 m | 1,297 m | 4,212 m |
| postman, greedy matching | 2,915 m | 1,196 m | 4,111 m |
| postman, exact blossom matching | 2,915 m | **1,076 m** | **3,990 m** (−5.3%) |

Two honest observations. First, on the small default window the postman route
is *identical* to the DFS route — there, the only repeat is a dead-end street
whose round trip no algorithm can avoid, and DFS happened to be optimal; this
served as a regression check. Second, most residual deadhead is forced
dead-end round trips; the postman's advantage grows with the number of cycles
(grid blocks) in the street network, and can never be worse than the matching
optimum.

## 7. Flight controller (`sim/controllers/parkdrone/parkdrone.py`)

One control loop, executed every 8 ms of simulated time. Each step reads the
IMU (roll/pitch/yaw), GPS (position; velocities by finite difference) and
gyro (angular rates), then:

### 7.1 Stabilisation and altitude

Propeller mixing and the inner PD stabiliser (roll/pitch attitude + rates) are
adapted from Webots' official Mavic 2 Pro sample. On top of it:

- **Altitude** holds 30 m with a cubic proportional term plus **vertical-speed
  damping** (`K_VD`). Without the damping term the drone overshot 30 → 51 m,
  tumbled and crashed.
- **Gimbal**: the camera is held at true nadir. On this proto the pitch joint's
  *positive* direction is down (range ≈ [−0.5, +1.7] rad), so nadir is +π/2;
  the intuitive "minimum position = down" assumption points the camera at the
  sky.
- Horizontal navigation is gated on a **settled** check (near target altitude
  and low vertical speed) — pitching forward mid-climb tumbled the drone.

### 7.2 Waypoint navigation

Steering is deliberately **car-like**: yaw the nose toward the target, then
throttle forward; roll is used only to damp sideways drift, never to strafe
toward the target (a lateral position command saturates while off-heading and
flips the drone). Yaw is a PD controller — proportional on heading error plus
**yaw-rate damping** from the gyro; pure-proportional yaw left the drone
spinning in circles, never converging on a heading.

Forward motion is a velocity-target controller:
`v_des = clamp(K_POS · fwd_err, 0, V_MAX)`, ramping down on approach so
row-end turns stay tight, and **zeroed while the nose is >0.5 rad off
target** — the drone will not fly where it is not looking. After the last
waypoint the controller station-keeps (brakes drift) instead of sailing away.

**Aim-point blending (pure pursuit).** Aiming at the current waypoint alone
caused a visible stop-and-pirouette at *every* waypoint: captures happen ~2.5 m
off-line (see below), so at each hand-off the next waypoint, 10 m ahead, sat at
a 15–30° different bearing — and the speed gate braked the drone while it
re-aimed, ~97 times per patrol. The fix: within 15 m (`AIM_BLEND`) of the
current waypoint the aim point slides linearly toward the *next* waypoint
(`aim = wp + (1 − dist/15)·(wp_next − wp)`), heading only. On straight legs the
nose flows through hand-offs with no bearing jump; at genuine 180° reversals
the blended aim lands behind the drone, the gate trips, and it correctly brakes
and U-turns. Measured effect: mean |yaw error| 0.253 → 0.136 rad; time spent
above the speed-gate threshold 15% → 9% of flight seconds (the residual being
real turns); slightly faster over the identical route segment; bay coverage
unchanged.

### 7.3 Arrival and capture

Distance-based "arrival" at a waypoint is unexpectedly subtle at this scale.
At cruise speed the turn radius is ~4 m, so with a tight arrival radius the
drone can settle into a **stable orbit** around a waypoint — constant distance,
so a pure distance test never fires and the patrol hangs forever. The working
logic keeps a generous 6 m basin (`WP_REACH`) and captures at **closest
approach**: a frame is taken when the drone is either very close (2.5 m,
`WP_CAPTURE`), or starts receding >1 m past its minimum distance, or has spent
8 s near the waypoint (`WP_TIMEOUT`, the orbit bail-out). At every capture the
controller saves the camera frame and appends its pose (x, y, altitude, yaw)
to `poses.json`, which is rewritten after every waypoint so progress survives
interrupted runs.

Verified coverage with this logic: 110/111 bays inside at least one captured
footprint on the 4-street world (the missed bay traces to one timeout arrival
14.9 m off its waypoint — capture scatter is a known open item; a later run at
the 5 m/s cruise cap covered all 111/111, see 7.4).

### 7.4 Simulation throughput: render on demand

Scaling to the neighbourhood world exposed a bottleneck that had nothing to do
with flight dynamics. With the camera enabled at the simulation's basic time
step (8 ms), Webots renders 125 camera frames per simulated second — of which
the controller saves roughly one per waypoint. On the 4-street world this
rendering held the whole simulation to ~2.5× real time even in fast mode.

The fix exploits the fact that frames are only needed at waypoint arrivals:
the camera stays disabled in cruise, is enabled when the arrival condition
fires, is given two control steps to produce a fresh image, and the frame and
the pose are then saved on the same step before the camera is disabled again.
The image–pose mismatch is bounded by one basic time step (~4 cm of travel at
5 m/s), the same bound as with continuous sampling, so the georeferencing in
the vision stage is unaffected. (The timed diagnostic snapshots, which do
need a continuously enabled camera, moved behind a debug flag.)

Measured on the 4-street world, together with raising the cruise cap `V_MAX`
from 2.5 to 5 m/s: wall time fell from 249 s to 46 s (5.4×) for the same
97-waypoint patrol, with zero capture failures, full 111/111 bay coverage and
the occupancy score unchanged at 100%. The higher cap itself was a mixed
result: it raises straight-leg speed (90th-percentile ground speed
2.77 → 4.21 m/s) but overshoots waypoints harder — orbit/timeout recoveries
actually lengthened the test patrol from 624 to 703 simulated seconds while
widening capture scatter (13 → 23 captures more than 4 m off their waypoint).

The neighbourhood world then delivered the verdict on the 5 m/s cap: after
645 clean waypoints the drone tumbled out of the sky entering a sharp
junction turn at full speed. Braking there saturates the brake-pitch, yaw and
drift-damping-roll commands simultaneously and for long enough that the lift
dip flips the drone — the same mechanism behind the project's standing
`TILT_MAX` limit, which brief saturations at 2.5 m/s never trigger. The
4-street world cannot catch this failure because its legs are too short to
reach full speed before a corner: a passing test on the small world does not
generalise to routes with long straights. The cap went back to 2.5 m/s
(median ground speed loses only ~15%, since the approach ramp dominates with
10 m waypoint spacing), and the crash motivated a **resume** capability: the
controller now reads the incrementally-written `poses.json` on startup and
continues from the first uncaptured waypoint, so a multi-hour patrol survives
crashes and host interruptions without re-flying.

### 7.5 Obstacle avoidance

The survey controller holds a fixed 30 m and, by design, knows nothing about
obstacles: the three survey worlds contain no collision geometry at all, so a
range sensor there would read maximum range forever. Obstacle work is therefore
gated behind a generator flag (`--collide`) which arms buildings, poles *and* the
drone's sensing together, and it is developed against a dedicated world.

**A purpose-built test world.** The only real world with anything to hit at
cruise altitude is the 1 km world, whose load time made the edit–run–observe loop
impractical. `fmi_block_obst` reuses the 4-street window and route and adds four
synthetic structures anchored to fractions along the finished route — each tall
enough to reach cruise altitude, and each a *different* failure mode rather than
four copies of one: a wide head-on `tower`; a `slab` offset 7 m so it clips the
flight corridor without blocking it; a 0.9 m-wide `mast`, thin enough to fall
between the rays of a sparse sensor fan at range; and a concave `trap` opening
toward the drone, the case where purely reactive avoidance circles forever.
Placement is deterministic, is nudged past any bay it would otherwise cover (a
box on painted tarmac would be a ground-truth bug), and enforces a 30 m
separation — because a postman route doubles back, and two anchors twelve
waypoints apart can land 15 m from each other.

The obstacle-blind controller provides the control baseline: flown on this world
it enters the tower at waypoint 12 and stops there, 13 frames into the patrol.
Every result below is measured against that.

**Sensing.** Nine `DistanceSensor` rays in a ±40° fan, 80 m range, mounted in the
drone's body slot. A fan of single-ray sensors rather than a Lidar, because that
is what the hardware plan specifies — and because its weakness is the thing worth
measuring: rays diverge, so a thin obstacle can hide between two of them at
range, which is exactly what the 0.9 m mast is there to demonstrate. If the mast
proves undetectable in time, *that* is the evidence for specifying a Lidar,
rather than an assumption made in advance.

The 80 m range is a **dynamics** specification, not a perception one. Tilt is
capped for stability (section 7.1) and the simulation has no aerodynamic drag, so
the achievable deceleration is only ~0.22 m/s²; stopping from 5 m/s therefore
takes v²/2a ≈ 57 m. A 40 m rangefinder could not stop this aircraft at cruise
speed no matter how good the logic — the sensor must be specified from the
braking distance, not from a round number.

**The channel principle.** Avoidance is built in two stages, *detect and stop*
and *steer around*, and both obey one structural rule: they write only into
channels the navigator already owns — the speed target `v_des` and the heading
error `yaw_err` — and never a lateral position target. That is the reason every
controller invariant of sections 7.1–7.3 survives untouched: the stabiliser sees
nothing it did not already see, and in particular the lateral-position command
that tumbles the aircraft (section 7.2) is never introduced. Adding avoidance
therefore cannot destabilise the survey flight, and this is verified rather than
argued: a survey world flown with and without the layer produces trajectories
agreeing to under 0.1 mm.

**Stage 1 — detect and stop.** This stage emits only a speed limit. Four
properties make that limit safe:

- Returns are **gated by bearing and corridor** before they count. A ±40° fan
  sees scenery well off to the side; without the gate the running minimum jumps
  between unrelated targets, and a limit computed from it can rise again while
  the drone is still braking.
- The limit **ratchets downward only**, so a momentarily lost return cannot hand
  cruise speed back.
- A **reaction allowance** is charged against the measured range. The bare
  braking curve permits an aircraft strictly below it to keep accelerating, which
  is only true of a vehicle that can brake instantly; this one needs about three
  seconds of travel to reverse a trend.
- Braking uses a **saturated gain**, and once stopped the drone **station-keeps
  on a latched point**. A proportional brake fades as the speed error closes, and
  pure damping cannot null a steady hover drift, so a proportional-only stop
  holds its distance briefly and then creeps. The hold converts position error
  into a *clamped velocity target*, which keeps it inside the channel principle.

Alone, this stage halts the drone at a 12.20 m standoff and holds it to within
±2 cm for over 400 s at 30.00 m. The patrol does not finish, which is the
intended scope of the stage.

**Stage 2 — steer around.** The manoeuvre is a turn, not a climb: climbing would
change the ground sampling distance and invalidate the classifier's 30 m
calibration (section 8). On detecting a gated return that lies within the
clearance radius of the route ahead, the controller commits to a **detour**,
picks the side with more room from the ray fan (scored on each side's *minimum*
range, since one blocked ray is what a collision is), and flies a tangent:

```python
# Speed on the arc is limited by two things, and the second is the one that
# makes a tight arc safe: the turn radius the lateral acceleration allows, and
# the speed from which the drone can still stop inside its own clearance.
v_arc = max(0.5, min(STEER_V,
                     math.sqrt(A_LAT * arc_R),
                     math.sqrt(2 * A_BRAKE_OBST * detour["C_R"])))

# Tangent law: to pass a point at CLEAR_R, fly a heading offset from its
# bearing by asin(CLEAR_R/d). As d shrinks to CLEAR_R the offset grows to 90
# degrees, so the drone rolls out onto a circle around the obstacle naturally,
# without needing a separate "circle" mode.
off = math.asin(clamp(arc_R / max(d_T, arc_R), -1.0, 1.0))
yaw_err = wrap(bear_T + detour["side"] * (off + STEER_MARGIN) - yaw)
```

Four design decisions carry this stage:

- **The threat is remembered as a growing bounding disc**, in world metres, not
  as a point. A point refreshed to the latest return effectively follows the
  drone: on a wide structure the nearest face slides around it as the drone
  circles, so the threat stays permanently "ahead" and the manoeuvre becomes an
  orbit. A single frozen point under-clears instead, since the rest of an 8 m
  structure still juts into the path. The disc only ever grows, and only by
  contiguity, so unrelated buildings cannot inflate it.
- **The leave condition is Bug2's, and it is about the goal, not the obstacle.**
  Geometric tests — "swept 110° around it", "it is abeam" — release while the
  waypoint is still on the far side, so the drone turns straight back into the
  structure. The detour is given back only when the drone is measurably closer to
  its waypoint than when it committed *and* can fly straight at it clear of the
  disc. The first half is what guarantees termination: every completed detour has
  made real progress, so the manoeuvre cannot cycle.
- **Commit and release ask the same question over the same window.** Both test
  the route from the drone to 12 m past the current waypoint — "this leg and the
  next". A wider commit horizon takes the drone off a waypoint just ahead for
  something blocking the route two legs later, and the leave condition then
  cannot fire, because that waypoint now lies behind; the only way to satisfy it
  is to go all the way round.
- **Bounded, and escalating rather than surrendering.** A reactive controller can
  always find a new way to circle, so a detour that accumulates 270° of sweep is
  abandoned, with a short cooldown against that disc and the drone handed back to
  its route with stage 1 still protecting it. A detour that instead *times out*
  doubles the clearance for that obstacle alone and retries. Giving up is
  per-threat, never global: one unsolvable structure must not cost the drone its
  avoidance for the remaining ninety waypoints.

**Clearance is a single constant.** The arc radius, the release test, the skip
radius and stage 1's standoff against the committed obstacle all derive from
`CLEAR_R`, so they cannot disagree. It is deliberately tight (5 m), because a
wide berth is what costs coverage: at 18 m clearance the patrol skips 28 of 97
waypoints and flies a mean 21.3 m off its own route, against 21 and 9.6 m at 5 m.
Reliability is bought back by retrying the awkward obstacle, not by widening
every detour. Tightness has a floor set by the sensor rather than by the tuning:
the nine rays are 10° apart, so at 5 m range the gaps *between* them are 0.9 m —
the mast case arriving early, and an argument for a Lidar rather than for a
smaller number.

Waypoints that fall inside the remembered disc cannot be flown to at all, so they
are skipped and recorded. A declared coverage gap is a correct survey result; an
endless orbit is not.

Frames captured while either stage is in control are tagged in `poses.json`, so
the scorer can exclude them rather than mis-score them silently, and
one-row-per-second telemetry is written to disk because the interesting seconds
fall *between* waypoint captures.

**Result.** The patrol **completes**: 96 of 97 waypoints, 79 frames, in 1599 s,
with all four structures passed — including `trap`, the concave U built
specifically to defeat a reactive controller. Six detours are flown, none of them
a circle, and the closest approach over the whole flight is 6.55 m. Against the
obstacle-blind baseline, which ended at waypoint 12, and against stage 1 alone,
which ended at waypoint 12 with 11 frames, the layer converts a collision into a
completed survey.

The cost is coverage: 21 of 97 waypoints are skipped, and the binding constraint
is stage 1's 12 m standoff rather than the arc — a waypoint closer than that to a
structure cannot be reached at all. Recovering it needs capture at the closest
*legal* approach with the frame flagged, which changes what such a frame means to
the scorer; that is a design decision rather than a tuning change, and it is
listed in section 11.

## 8. Occupancy scoring (vision stage, v1)

The first vision implementation (`vision/score_occupancy.py`) deliberately
inverts the usual pipeline. Instead of running an open-ended car detector and
matching detections to bays, it exploits what the system already knows: the
exact bay polygons (GeoJSON) and the exact drone pose per frame — both in the
shared ENU frame of section 3. This is the **georeferencing-first** approach:

1. **Projection.** For a nadir camera at pose (x, y, alt, yaw), a ground point
   projects to pixel coordinates by rotating its offset into the body frame
   (image-up = drone heading) and scaling by `IMG_W / (2·alt·tan(FOV/2))`
   pixels per metre (~16 px/m at 30 m). Every bay polygon is projected into
   every frame.
2. **View selection.** A view of a bay is usable when the *inner* 78% of its
   projected polygon lies inside the frame (the outer band is excluded from
   classification anyway — it contains the painted outline and neighbouring
   cars). Bays are typically visible in 1–5 frames.
3. **Per-view classification.** Over the inner region the classifier computes
   the fraction of "paint-like" pixels (bright, non-chromatic — the bay
   marking) and of "dark" pixels (glass, wheels, shadow — things paint never
   shows). A view votes *occupied* if paint < 70% or dark > 4%. The thresholds
   were calibrated once against the 4-street world's ground truth.
4. **Multi-view voting.** A bay is declared occupied when a **strict majority
   of its views** sees a car. Voting eliminates single-view artefacts — a
   neighbouring car's overhang contaminates a bay's crop from one angle but
   not from others.
5. **Scoring.** Predictions are compared with `ground_truth.json`: confusion
   matrix, accuracy, precision, recall; per-bay results and annotated debug
   overlays are written next to the frames.

Results on the 4-street world (111 bays, 49 occupied, 97 frames), on whose
ground truth the two thresholds were calibrated:
**107/111 bays classified — accuracy 97.2%, precision 100%, recall 93.5%**;
4 bays had no usable view (capture-scatter edge cases). As a held-out check,
the same classifier was then applied unchanged to a fresh flight of the
default world (45 bays, different occupancy pattern): **40/45 classified —
accuracy 95.0%, precision 100%, recall 88.2%** — consistent with the
calibration world, so the heuristic is not overfitted to one scene. Every
misclassification across both worlds was the same failure mode: a **white
vehicle on white bay paint** (white cars and vans whose uniform bodies read
as paint) — the natural limit of colour-statistics classification on that
scene.

### 8.1 Error-driven iteration (v1.5 and the realistic world)

Going through the v1 errors one by one — the debug overlays make each
misclassification inspectable — produced three targeted fixes and exposed two
deeper problems, one in the simulated scene and one in the aircraft itself.

The three classifier fixes (v1.5):

- **Brightness/texture thresholds.** A truly free painted bay is almost
  perfectly constant in the frames (brightness ≈ 239, σ ≈ 3 under the sim's
  uniform light), while every missed white vehicle sat at brightness 176–213
  with σ ≥ 21. Adding "occupied if brightness < 225 or σ > 15" recovered all
  five white-on-white misses (recall 100%).
- **Core sampling.** The new thresholds initially traded the misses for new
  false positives: cars can physically overhang the *adjacent* bay (a bumper
  across the line — visible in the overlays), which legitimately puts
  non-paint pixels inside a free bay. But overhang can only intrude past the
  bay's short ends, so classification moved to the lengthwise **core** of the
  bay (central 55% of the long axis, full width): a parked car always covers
  the core, an overhanging neighbour never reaches it.
- **Partial views.** v1 required the whole sampled region inside the frame;
  measuring the "uncovered" bays showed their best views missed full
  visibility by only 12–45 px. Views now count whenever ≥ 70% of the core
  area is visible, masked to the visible part.

The two deeper findings:

- **The camera was not actually pointing straight down.** The remaining false
  positives all showed the *projected* bay polygons shifted by ~1 m against
  the scene — across the whole frame, so a pose problem, not a classifier
  problem. The cause: the gimbal pitch was commanded to a fixed +π/2
  *relative to the drone's body*, and a quadcopter cruises pitched forward a
  few degrees — at 30 m altitude a 2° tilt displaces the footprint by
  `alt·tan(2°) ≈ 1 m`. The projection model assumes true nadir. The fix
  subtracts the body attitude in the gimbal command (`π/2 − pitch`, `−roll`),
  and the controller now records per-frame roll/pitch in `poses.json`. This
  is a genuinely instructive bug for the real-drone transfer: it is exactly
  the class of error a hardware gimbal hides (it stabilises mechanically) and
  a fixed-mount camera does not.
- **The scene itself was unrealistically easy — and unrealistically hard.**
  The world painted each bay as a *filled white rectangle*, whereas real
  Sofia bays are thin white **outlines on asphalt**. That single rendering
  choice both made free/occupied trivially separable by colour (inflating
  v1's apparent quality) and created the white-on-white failure mode (real
  aerial imagery rarely hides a white car, because bays are not solid white).
  The world generator now renders each bay as an asphalt pad with a ~12 cm
  painted outline; ground truth and routes stayed byte-identical.

On the realistic scene the classifier was recalibrated (same code path, new
thresholds): a free core is now *uniform asphalt*, and a view votes occupied
on any of five cues — chroma (coloured bodywork; on the calibration world
this cue alone separates the classes: free ≤ 12.0, occupied ≥ 13.5), very
dark pixels (glass/shadow, well below asphalt), bright achromatic pixels
(white/silver roofs), brightness outside the asphalt envelope, or texture.
After re-flying both worlds with the nadir-corrected gimbal:
**dev world 109/111 classified, held-out world 43/45 — accuracy, precision
and recall all 100% on both.** The remaining unclassified bays are the known
capture-scatter cases. The colour-statistics heuristic thus saturates the
current simulation; a learned classifier only becomes necessary together
with harder rendering (textures, shadows, lighting variation) or real
imagery — the projection, view-selection and scoring stages are
classifier-agnostic either way.

### 8.2 Occlusion by scenery: a parallax false positive

Adding buildings, greenery and light poles (section 4) and re-flying the
calibration world moved it off saturation for the first time, to **99.1%**
(TP 49, TN 61, FP 1, FN 0). The single false positive was diagnostic rather than
noise. Bay 17579 was photographed from 110 px off-nadir, and a 7 m light pole
standing *beside* it leaned across it under that parallax; the dark pole pixels
put the core's dark fraction at 0.03 against a 0.02 threshold, with brightness at
82 — exactly on the envelope's lower bound. The pole does not stand on the paint
(the generator rejects that placement); it overhangs from above, which a real
street lamp does too. That is the intended kind of hard case: a physically real
occlusion, not a rendering artifact, and one a core-crop colour heuristic cannot
fix because the occluding object is genuinely inside the crop.

The measurement is no longer reproducible, and *why* is itself instructive. A
later re-fly of the same world scores **100% with 111/111 bays covered**, because
the flight path changed underneath the result: the capture geometry that put bay
17579 110 px off-nadir no longer occurs, so the pole no longer leans across it.
The failure *mechanism* is real and remains an argument for the learned
classifier; the *number* was a property of one flight, not a stable property of
the world. Section 9 draws the general lesson.

### 8.3 The camera was never nadir

The held-out world `fmi_block` sat at **95.2%** while the calibration world was
perfect: two free bays, 17685 and 17686, were reported occupied. Both were
captured **136–146 px off the image centre**, and the cause was *projection*, not
classification. At that eccentricity the projected bay polygon no longer lined up
with the painted outline in the image, so white line pixels fell inside what the
scorer treats as the bay's core. The measured features said so directly — paint
fraction 0.07, brightness 105 against a free envelope of 82–102, texture std 40 —
on what is physically empty asphalt.

This is the same family as the gimbal-pitch error of section 8.1: a geometric
misalignment presenting as a classification failure, and one that no amount of
threshold tuning fixes, because the crop is looking at the wrong pixels.

**Measuring it properly was most of the work.** A first attempt sampled a
one-dimensional brightness profile across each bay's short axis and located the
painted line by its peak. That measurement is both biased — the recovered line
half-width came out consistently under the predicted one — and, being
one-dimensional, incapable of reporting the *direction* of the error. Direction
turned out to be the entire diagnosis.

The replacement (`vision/diag/paint_align.py`) registers the whole predicted
outline pattern against the paint visible in the frame, in two dimensions and at
sub-pixel resolution: the observed image is reduced to a ridge response — grey
minus a local box mean, kept only where the pixel is near-achromatic, so a thin
painted line survives and a car roof does not — and matched to the rasterised
prediction by normalised cross-correlation. Crucially, the estimator is
**validated before it is believed**: known shifts are injected into the observed
image and must be recovered, which they are to 0.10 px against an effect of ten
pixels. Two details of that validation mattered. Fitting a parabola to the
correlation peak recovers whole-pixel shifts perfectly but is biased on
fractional ones, because the correlation of a thin-line pattern is far too
sharply peaked for a quadratic; a direct sub-pixel search replaces it. And the
test must run on a frame that contains enough paint to locate, which is a
property of the frame rather than of the method.

**The result was unambiguous.** The offset was almost purely *cross-track*, which
by itself eliminates timing and lag, and it tracked the drone's body roll at a
slope of −1.02 with R² 0.994 — as if the gimbal's roll compensation did not exist.
The fore/aft component followed the *residual* pitch, the few milliradians by
which the pitch joint lags its own command, so that joint was working.

The explanation is the joint order of the gimbal. Its roll joint sits below its
pitch joint, so once the pitch joint is driven to the +π/2 that a nadir survey
requires, the roll axis has been rotated onto the optical axis. It can no longer
level the camera; it only rotates the picture. The controller was commanding roll
compensation, the joint was obeying, a position sensor confirmed the motion — and
none of it reached the image. This is a case where every individual component
behaves correctly and the composition does not, which is precisely the kind of
error that survives inspection and is only caught by measurement.

`project()` therefore no longer assumes a nadir camera. It builds the optical
axis and image axes from the attitude already recorded in `poses.json`: a tilt by
the full body roll, a tilt by the residual pitch, and an image rotation by the
gimbal's roll angle. A pose carrying no attitude still projects as nadir, so
earlier captures remain scorable.

| | before | after |
|---|---|---|
| median per-frame misalignment | 0.269 m | 0.043 m |
| worst frame | 0.97 m | 0.060 m |
| `fmi_block` accuracy | 95.2% | **100%** |
| `fmi_block_4st` accuracy | 100% | 100% |

A companion tool (`vision/diag/offset_report.py`) regresses the residual against
every candidate cause in the pose, and is the standing check on the camera model:
a correct model leaves every R² near zero, while a slope near ±1 against an
`alt × angle` term names the angle being ignored. After the fix, every R² is at
most 0.04. What remains is a constant 3.7 cm along-track bias, which matches the
computed offset of the camera ahead of the GPS antenna and is well below what the
classifier can resolve.

## 9. Verification methodology

There is no unit-test suite; verification is empirical and scripted:

- **Headless runs**: Webots runs with `--batch --mode=fast --minimize`, with
  stdout piped (not redirected — Webots block-buffers to files) and the
  controller's disk output (`frames`, `poses.json`) treated as the source of
  truth.
- **Geometric checks** after each change: every coverage edge within 1.5 m of
  the flown route; every waypoint within 0.3 m of an OSM centerline; car-body
  overlap testing (separating axis); bay-coverage counting against the camera
  footprint model; route-length comparisons old vs new.
- **Regression anchor**: the default world's 34-waypoint route, flown and
  verified in full, must survive algorithm changes byte-identical (it did for
  the postman migration and the origin pinning).
- **Byte-identical metadata as the standing test**: after any generator change,
  every world is regenerated and its `ground_truth.json` and `route.json` are
  compared byte-for-byte against the previous build. The random streams are split
  deliberately (occupancy, car choice, scenery cosmetics) so that cosmetic and
  performance work *cannot* move the labels — and the byte comparison is what
  proves it rather than assumes it. Both changes in section 4.1 were validated
  this way.
- **A/B with repeats when a change might affect accuracy**: the geometry merge
  was validated by flying four patrols per variant and comparing confusion
  matrices, not by a single before/after run.

**Determinism, and the trap of a stale fixture.** Webots runs here are
reproducible: the same world with the same controller yields identical frames,
poses and scores (verified across eight runs). That is a strong property — it
means any change in an accuracy figure points at a changed world or controller
rather than at noise. It also sets a trap, which this project fell into. A
"golden" result for the held-out world had been recorded as 100%, but the frames
behind it were captured in July and merely *re-scored* in August after the
scenery was added; they were never re-flown. The number therefore described a
world that no longer existed, and it silently stopped being reproducible. Two
rules follow, and both are now enforced: **a fixture must be re-flown, not
re-scored, whenever the world or the controller changes**, and a stored result
must carry the date of the *flight* rather than of the scoring. Re-flying both
fixtures revealed one improvement (the calibration world, 99.1% → 100%) and one
genuine defect that had been masked for over a week — the camera model of
section 8.3, since fixed.

## 10. Transfer to the real drone

The simulation was structured so that each component maps onto a concrete part
of the planned hardware stack (Pixhawk autopilot + Raspberry Pi 4 companion
computer + Coral USB accelerator + IMX219 camera). The mapping below states,
for every piece of simulation logic, what transfers directly, what is replaced
by an off-the-shelf equivalent, and what changes.

### 10.1 What transfers directly

- **The GIS pipeline and route planner transfer unchanged.** Nothing in
  stages 1–2 of route building depends on Webots: the bay construction, street
  matching, coverage graph, and rural-postman solver operate purely on
  GeoJSON and produce waypoints in the pinned ENU frame. For a real flight the
  same waypoint list is converted back to WGS84 (the inverse of the projection
  in section 3 — a two-line change) and uploaded to the autopilot as a
  standard MAVLink mission (e.g. a QGroundControl `.plan` file). Route
  optimality, coverage guarantees and the 10 m capture spacing carry over
  as-is.
- **The georeferencing frame transfers unchanged.** The pinned origin and ENU
  convention are shared by the planner, the recorded poses and the future
  detector; on the real drone the pose record is assembled from autopilot
  telemetry (MAVLink `GLOBAL_POSITION_INT` + `ATTITUDE`, i.e. GPS/EKF position
  and yaw) instead of simulated GPS/IMU devices, but the downstream
  detection-to-bay matching maths is identical.
- **The vision stage transfers by design.** The detector consumes
  georeferenced nadir frames plus poses; whether those come from
  `sim/output/<survey_area>/` or from the real camera is transparent to it.
  Simulated frames additionally serve as pre-training/validation data with
  perfect labels (with the usual sim-to-real domain gap — see 10.3).

### 10.2 What is replaced by the autopilot

The entire hand-written flight controller (section 7) is a **simulation stand-in
for ArduPilot/Pixhawk** and is deliberately *not* transferred:

- **Stabilisation, altitude hold, waypoint navigation** — ArduPilot's AUTO
  mode with its EKF, position controller and S-curve/L1 trajectory logic
  replaces the PD stabiliser, the settled gate, the car-style steering and the
  arrival logic. Notably, the problems we solved by hand in simulation
  (altitude overshoot without vertical damping, yaw without rate damping,
  orbiting a waypoint, stop-and-turn at dense waypoints) are all problems
  ArduPilot's tuned cascaded controllers and look-ahead trajectory generation
  solve internally — encountering and fixing them in simulation is precisely
  what motivates trusting a mature autopilot instead of a custom loop.
- **Frame capture** — instead of the controller's closest-approach logic, the
  camera is triggered either by the autopilot's distance-based trigger
  (`CAM_TRIGG_DIST`, matching the 10 m spacing) or by the companion computer
  watching mission progress over MAVLink and commanding the camera per
  waypoint; each trigger stores the telemetry pose alongside the image
  (the real-world `poses.json`).
- **Gimbal nadir** — a hardware gimbal (or fixed nadir mount) replaces the
  simulated gimbal-pitch command; the lesson that the mount's sign conventions
  must be verified against reality (section 7.1) carries over directly.

### 10.3 What changes materially

- **Flight-plan constraints.** EU open-category rules cap altitude at 120 m
  (our 30 m is comfortably legal) but require visual line of sight, which a
  19 km neighbourhood patrol violates from a single observer position — real
  surveys would be flown per-block, or under a specific-category
  authorisation. Real cruise speeds (5–10 m/s for a Mavic-class airframe)
  shorten the neighbourhood patrol to 30–60 min of flight, but battery
  endurance (~25–30 min) still forces the route to be split into legs with
  battery swaps — a natural extension of the postman planner (bounded-length
  open paths from/to swap points).
- **Environment.** The simulation's empty streets become streets with moving
  traffic, pedestrians, trees occluding bays, varying light and weather. This
  affects the vision stage (domain gap: the detector must be fine-tuned or
  validated on real imagery) and safety (obstacle avoidance, covered by the
  hardware plan but absent from the simulation).
- **Positioning error.** Simulated GPS is exact; real GPS/EKF is 1–3 m. Since
  a bay is ~2.2 m wide, detection-to-bay matching must tolerate that error —
  the same tolerance that already exists for capture scatter in simulation
  (nearest-bay matching plus the generous footprint overlap), but it should be
  quantified with real flights. RTK GPS is the upgrade path if plain GPS
  proves too coarse.
- **Verification tooling.** Headless-run log analysis is replaced by ArduPilot
  dataflash log analysis; the repository already contains the counterpart tool
  (`tools/analyze_log.py`: flight modes, GPS/EKF health, commanded vs actual
  attitude from `.bin` logs), which was used for debugging real test flights.

The overall claim for the thesis: the simulation is not a toy replica of the
real system but the **same mission stack** (data → plan → fly → capture →
georeference → score) with the flight-dynamics layer swapped for a stand-in;
transfer consists of replacing that one layer with ArduPilot and re-validating
the unchanged layers above it.

## 11. Current limitations and future work

- **Vision is a calibrated heuristic**: it scores **100%** on the calibration
  world (`fmi_block_4st`, 111/111 bays covered) and, since the camera-model fix
  of section 8.3, **100%** on the held-out world too — but its thresholds encode the sim's uniform lighting and untextured
  surfaces. It will not survive shadows, surface texture, weathered markings or
  real imagery — the step to a learned classifier over the identical bay crops
  belongs together with making the scene harder, so that the comparison is
  meaningful. The scenery (section 4) is the first instalment of that hardening,
  and a bounded one: the classifier samples only the lengthwise *core* of each
  bay, so scenery that merely sits beside a bay never enters the sampled pixels.
  The exposed surface not yet measured is bays that sit **on** a green polygon:
  none in the two small worlds, 31 in the 1 km one, so that world is where the
  next measurement should be taken.
- **Coverage under obstacles** (section 7.5): 21 of 97 waypoints on the obstacle
  world are skipped, and the binding constraint is the 12 m standoff rather than
  the detour arc — a waypoint closer than that to a structure cannot be reached
  at all. Recovering it means capturing at the closest *legal* approach and
  flagging the frame, which changes what such a frame means to the scorer, so it
  is a design decision rather than a tuning change.
- **Shadows are the real hardening lever, and are deliberately still off.**
  `castShadows FALSE` on the sun is not an aesthetic choice: shadow mapping over
  a 1.2 km ground plane painted streak artifacts across the nadir frames. Turning
  it on would therefore harden the classifier against a *rendering defect* rather
  than against real shading, and doing it in the same step as the scenery would
  make any accuracy change unattributable. The right sequence is its own
  experiment — a window-sized ground plane or a tighter shadow frustum first,
  measured on its own — alongside the learned classifier that has to survive it.
- **Cruise speed** is back at 5 m/s, made safe by a per-waypoint **speed
  profile** rather than a blanket cap: each waypoint takes a corner limit from
  the route's heading change there (full speed on straights, 2.0 m/s at ~90°,
  0.8 m/s at a reversal), and a backward pass then propagates those limits
  upstream along the kinematic braking curve so the drone is always slow enough
  to stop for the corner ahead. The braking constant it plans against (0.22 m/s²)
  is deliberately below the ~0.29 m/s² measured from flight logs; an earlier
  linear-ramp brake model assumed roughly five times more deceleration than the
  aircraft has and blew through waypoints into wide recovery loops. Sharp
  reversals additionally stop and turn on the spot, because a U-turn taken with
  momentum traces a wide teardrop rather than a tight pivot.
- **Faster neighbourhood patrols**: world *load* is no longer the obstacle
  (section 4.1 took it from >10 min to 57 s). What remains is simulated-flight
  time. Two levers are untried: **fly higher** — at 50 m the footprint grows
  ~1.7×, so waypoint spacing could double, halving both waypoints and route
  length, at the cost of ground resolution (16 → ~10 px/m, which would require
  re-verifying the classifier at that scale) — and **cheaper physics**, since the
  parked cars only need to be seen and never collided with, so stripping their
  collision geometry (or raising `basicTimeStep`, which would force a controller
  re-tune) targets the physics core directly.
- **The 1 km survey is unfinished**: 318 of 1976 waypoints captured, and not yet
  scored. The load-cost work removes the practical barrier to completing it.
- **Capture scatter**: occasional timeout arrivals capture up to ~15 m off the
  waypoint; ~1% of bays can fall outside all footprints at zero margin (3 of 45
  on the held-out world).
- **Name matching**: 4 of 49 streets in the 1 km cut have no OSM name match
  and use the straight-line PCA fallback (fine for straight streets only).
- **Angled bays**: "Косо" spaces are drawn at a fixed 45° to the street; the
  actual angle direction (left/right of the street) is not in the data.
- **Obstacle avoidance is reactive and has no map** (section 7.5). It completes
  the test course, concave `trap` included, but it plans from what the ray fan
  can currently see plus a remembered disc per threat; it does not build a
  persistent obstacle map, and it has never been flown on the 1 km world or with
  proxy-geometry cars. The thin `mast` remains the standing argument for a Lidar:
  at close range the gaps between adjacent rays exceed the clearance being
  flown.
- **Shadows remain off** and are the single largest untested hardening lever;
  see the note above on sequencing that as its own experiment.
