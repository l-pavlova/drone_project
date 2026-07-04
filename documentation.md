# PARKDRONE — Simulation Documentation (thesis draft)

*Draft for thesis use. Covers the problem statement, the data and world-building
pipeline, the simulation scales, the flight controller, waypoint navigation, and
the evolution of the route-planning algorithm (DFS → rural postman). Numbers are
from verified simulation runs (July 2026).*

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
3. **Vision / scoring** (not yet started) — detect cars in the captured frames,
   project detections to ground coordinates, match them to bays, and score
   against the known ground truth.

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

`python generate_world.py [window_half_m] [occupied_fraction] [world_name]`
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
  window). Road heights are staggered by millimetres to avoid z-fighting at
  intersections.
- **Painted bays**: thin white boxes at each bay rectangle.
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
colour choices come from a *separate* stream (`random.Random(7)`), so cosmetic
changes to car placement never change the ground truth.

## 5. Simulation scales

The same generator covers all scales; each world is an independent file set
with its own route and ground truth, and the controller writes its captures to
a per-world output folder (`sim/output/<world>/frame_###.png`, `poses.json`):

| world | half-size | bays | cars | routed streets | route | waypoints |
|---|---|---|---|---|---|---|
| `fmi_block` (default) | 75 m | 45 | 17 | 2 | 286 m | 34 |
| `fmi_block_4st` (test) | 130 m | 111 | 49 | 4 | 871 m | 97 |
| `fmi_block_1km` | 500 m | 1,593 | 727 | 45 (+4 PCA-fallback) | 19.0 km | 1,976 |

The default world is the regression baseline (its route has been flown and
verified repeatedly); the 4-street world exercises junctions, dead-end
backtracks and transits at manageable runtime; the 1 km world is the
neighbourhood-scale target. At the current cruise speed (2.5 m/s) the 1 km
patrol is ≈2 h of simulated flight — raising cruise speed (or altitude, for a
larger footprint) is future work.

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
14.9 m off its waypoint — capture scatter is a known open item).

## 8. Verification methodology

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

## 9. Transfer to the real drone

The simulation was structured so that each component maps onto a concrete part
of the planned hardware stack (Pixhawk autopilot + Raspberry Pi 4 companion
computer + Coral USB accelerator + IMX219 camera). The mapping below states,
for every piece of simulation logic, what transfers directly, what is replaced
by an off-the-shelf equivalent, and what changes.

### 9.1 What transfers directly

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
  `sim/output/<world>/` or from the real camera is transparent to it.
  Simulated frames additionally serve as pre-training/validation data with
  perfect labels (with the usual sim-to-real domain gap — see 9.3).

### 9.2 What is replaced by the autopilot

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

### 9.3 What changes materially

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

## 10. Current limitations and future work

- **Vision stage not started**: frames and poses are captured but no detector
  consumes them yet; scoring against `ground_truth.json` is designed but not
  implemented.
- **Cruise speed**: 2.5 m/s makes the 19 km neighbourhood patrol ≈2 h of sim
  time; speed/altitude trade-offs (footprint size vs image resolution vs
  turn dynamics) are unexplored.
- **Capture scatter**: occasional timeout arrivals capture up to ~15 m off the
  waypoint; ~1% of bays can fall outside all footprints at zero margin.
- **Name matching**: 4 of 49 streets in the 1 km cut have no OSM name match
  and use the straight-line PCA fallback (fine for straight streets only).
- **Angled bays**: "Косо" spaces are drawn at a fixed 45° to the street; the
  actual angle direction (left/right of the street) is not in the data.
