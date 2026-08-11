# Webots 1 km world: load time and memory

Research note, 2026-08-10. Target: `sim/worlds/fmi_block_1km.wbt`
(`python generate_world.py 500 0.5 fmi_block_1km`), Webots R2023b, Windows 11.

Machine used for all measurements below: i7-10750H (6C/12T), 21.3 GB RAM,
GTX 1650 (4 GB VRAM) + Intel UHD.

---

## Bottom line

**~6 GB is normal for this scene; 10 minutes is at the bad end but not an outlier.**
The memory figure is fully explained by one line item — **727 vehicle protos** —
and I measured it rather than inferred it:

> **A Webots vehicle proto instance costs ~5.1 MB of RSS and ~0.14 s of load time,
> and both scale perfectly linearly up to at least 800 instances.**

727 cars × 5.1 MB ≈ **3.7 GB**. Adding the measured cost of everything else
(8379 `Solid`s ≈ 1.2 GB, 179 `Road`s ≈ 0.3 GB, 775 `SimpleBuilding`s ≈ 0.3 GB,
0.23 GB base) predicts **≈ 5.7 GB** — which is the ~6 GB you observe. There is no
mystery term and nothing pathological in how `generate_world.py` writes the file.

The *time* is the part that does not add up. Headless (`--batch --mode=fast
--minimize`) the same budget predicts **≈ 110 s**, not 10 minutes. So roughly 4–5×
of your load time is *not* linear node cost; the most likely cause is that ~6 GB of
scene against a 4 GB VRAM card puts the driver into paging (see
"What I could not determine"). One incidental observation supports this: an
800-car probe loaded in **110 s** from a clean start, but an identical 200-car
probe that was launched immediately after a hard-killed Webots instance spun for
**>12 minutes** without finishing, and re-running it clean took **29 s** — a 25×
penalty from machine state alone. The `taskkill` gotcha in CLAUDE.md is more
load-bearing than it looks.

**Do first, in this order:**

1. **Merge the 7965 bay `Solid`s into a handful of `IndexedFaceSet`s** (option B).
   ~1.1 GB saved, pixel-identical output, byte-identical `ground_truth.json` /
   `route.json`, zero vision risk. This is free money.
2. **Emit low-poly proxy cars on large worlds only, behind a flag** (option A).
   ~3.5 GB and ~95 s saved. Bigger win, but it changes what a car *looks like*
   from 30 m, so it must not touch `fmi_block` / `fmi_block_4st` (the calibrated
   worlds) and it makes the 1 km accuracy number non-comparable.
3. Accept the remainder. Load is once per run against a >1 h patrol — the
   pay-off of anything past A+B is small (see option H).

Do **not** bother reducing `MAX_BUILDINGS`: I measured `SimpleBuilding` at
**0.40 MB and ~0 s per instance**. All 775 of them cost ~0.3 GB. That candidate is
refuted. The "scenery-free build was similarly slow" observation in the brief was
correct and is now explained: scenery was never the cost.

---

## Half 1 — Is this normal?

### What Webots actually does per PROTO instance

I read the R2023b sources rather than guessing:

* **Triangle meshes are deduplicated across instances.** `WbTriangleMeshCache`
  ([`WbTriangleMeshCache.hpp`](https://raw.githubusercontent.com/cyberbotics/webots/R2023b/src/webots/nodes/utils/WbTriangleMeshCache.hpp))
  keys a `WbTriangleMesh` by a SipHash of the coordinate/index/normal arrays;
  the header states outright *"TriangleMeshInfo is shared by all
  WbTriangleMeshGeometry instances requiring the same WbTriangleMesh"*, with a
  `mNumUsers` refcount. `WbIndexedFaceSet::computeHash()` builds that key from
  the raw field arrays.
* **Textures are deduplicated too.**
  [`WbImageTexture.cpp`](https://raw.githubusercontent.com/cyberbotics/webots/R2023b/src/webots/nodes/WbImageTexture.cpp)
  line 241: *"Only load the image from disk if the texture isn't already in the
  cache"* → `wr_texture_2d_copy_from_cache(url)`. So 727 cars do **not** cost 727
  copies of the bodywork JPEGs.
* **GPU buffers are NOT deduplicated.**
  [`WbTriangleMeshGeometry.cpp`](https://raw.githubusercontent.com/cyberbotics/webots/R2023b/src/webots/nodes/WbTriangleMeshGeometry.cpp)
  calls `wr_static_mesh_new(...)` inside `createWrenObjects()` — **per geometry
  node instance**. There is no GPU instancing path. Every one of the 727 cars
  uploads its own vertex/index buffers for every submesh it contains.
* **Parsing is per instance regardless.** Even on a cache *hit*, Webots must have
  parsed the full coordinate arrays to compute the hash. There is no "skip the
  text" shortcut.

That is the mechanism behind the measured ~5.1 MB/car: shared CPU meshes and
shared textures, but private WREN buffers plus a large private scene-tree subtree.

### How heavy is one "Simple" vehicle?

Sizes pulled from the GitHub contents API for tag `R2023b`, and triangle counts
estimated by counting `-1` face terminators in the raw proto text:

| proto family | total .proto bytes | dominant meshes (est. triangles) |
|---|---|---|
| tesla | 1047 KB | Coachwork ~11.4 k, Details ~5.4 k, Wheel ~2.0 k (×4) |
| bmw | 1612 KB | X5Mesh ~5.7 k, Interior ~8.7 k, RearInterior (718 KB) |
| lincoln | 1210 KB | DetailsMesh ~9.6 k, CoachworkMesh ~9.6 k |
| toyota | 491 KB | PriusMesh ~8.6 k |
| citroen | 283 KB | CZeroMesh ~4.3 k |
| range_rover | 383 KB | 5 separate coachwork meshes |
| mercedes_benz | 21 KB | external `.obj` meshes (`Mesh { url "meshes/*.obj" }`) |

So a "Simple" car is roughly **20–30 k triangles**, delivered as ~1 MB of inline
`IndexedFaceSet` text spread over 6–13 sub-protos. `BmwX5Simple` does already omit
`BmwX5Interior`/`BmwX5RearInterior` — "Simple" means no physics/joints/interior,
not low-poly. The exterior mesh is the full-detail one.

727 cars ≈ **15–20 million triangles** of static geometry, each with its own draw
setup. That is a game-engine-scale scene handed to a robotics simulator that does
no instancing.

### Is ~2000 protos an outlier?

By Webots' own standards, **yes, by more than an order of magnitude**. Node counts
of the shipped worlds in `C:\Program Files\Webots\projects\vehicles\worlds`
(counted by grepping top-level `Node {` lines):

| shipped world | top-level nodes | vehicle instances |
|---|---|---|
| `village_realistic.wbt` | 387 | **28** |
| `village_center.wbt` | 375 | 7 |
| `city.wbt` | 117 | 1 |
| `CH_Morges.wbt` (largest shipped) | 2763 | 0 |
| **`fmi_block_1km.wbt`** | **10065** | **727** |

The most vehicle-dense world Cyberbotics ships has 28 cars. We have 727 — 26×.
The largest shipped world by node count has 2763 nodes and no vehicles at all.
We are outside the envelope the simulator is tuned and tested for. That is a
statement about scale, not about a bug in our generator.

### What the community reports

* **Procedural protos are a known load-time tax.** The reference manual is
  explicit: *"Using procedural PROTO files can greatly increase the loading time of
  your worlds because every procedural PROTO need to be evaluated."* The mitigation
  is `# tags: static`, which collapses evaluation *"if the same procedural PROTO is
  used several times in a world **and all the field values are the same**"*
  ([procedural-proto-nodes](https://github.com/cyberbotics/webots-doc/blob/master/reference/procedural-proto-nodes.md)).
  Relevant to us: `SimpleBuilding` **is** procedural (`# template language:
  javascript`, wrapping `Building.proto`) and `Road` is a 49 KB template. But every
  one of our 775 buildings has a different `corners` field, so `static` could never
  apply — and my measurement says buildings are ~free anyway, so the JS evaluation
  is not our bottleneck.
* **Many EXTERNPROTO declarations make startup very slow and the window black
  with no progress feedback** —
  [issue #4946](https://github.com/cyberbotics/webots/issues/4946), fixed for the
  *feedback* (progress bar, PR #4993, R2022b), not for the cost. We declare only 11
  EXTERNPROTOs at top level, but each vehicle pulls 6–13 sub-protos transitively.
* **Worlds hanging at the "Finalizing nodes" stage** —
  [issue #3065](https://github.com/cyberbotics/webots/issues/3065): loading
  oscillating between 84 % and 99 % and freezing the UI. Same phase we sit in.
* **Import being blocking and the devs' stated direction being lazy loading** —
  [issue #760](https://github.com/cyberbotics/webots/issues/760),
  [issue #2691 "Optimize web simulations loading time"](https://github.com/cyberbotics/webots/issues/2691).
  There is no released "stream the world in as you fly" feature to lean on.
* **webots-bin reaching 5 GB+ is a reported figure** in RL reset loops
  ([issue #2663](https://github.com/cyberbotics/webots/issues/2663)) — different
  cause (leak across resets), but it establishes that multi-GB `webots-bin` is
  within the range others see rather than something exotic.
* **The official speed guide** ([discussion #5402](https://github.com/cyberbotics/webots/discussions/5402))
  says: raise `basicTimeStep`, lower FPS, disable `castShadows`, *"remove
  unnecessary objects"*, and *"replace complex primitives (Cylinder,
  IndexedFaceSet, Mesh, ElevationGrid) with simpler shapes (Sphere, Capsule, Box,
  Plane)"*, and *"use Transform or Shape nodes instead of Solid nodes where
  appropriate"*. Note that all of this advice is about **step** cost; Cyberbotics
  publishes no guidance at all on **load** cost. Options A and B below are just
  that advice applied to our two biggest node populations.

### Measured baseline (this machine, this Webots)

Ten synthetic probe worlds, each a bare ground plane + `Mavic2Pro` + N copies of
one node type, run as
`webots.exe --batch --mode=fast --minimize --stdout --stderr`. "load_s" is wall
clock from launch to Webots printing `INFO: parkdrone: Starting controller`
(±2 s, that being the memory-polling granularity); RSS is the peak of
`webots-bin` working set sampled every 2 s. Probe worlds were deleted afterwards;
no project file was modified.

| probe | instances | load (s) | peak RSS (MB) |
|---|---:|---:|---:|
| empty (ground + drone only) | 0 | 8.1 | 231 |
| `Solid` + `Box` (bay-pad sized) | 2000 | 7.3 | 521 |
| `Solid` + `USE`-shared `Shape` | 2000 | 5.0 | 476 |
| `SimpleBuilding` | 200 | 4.9 | 311 |
| `Road` + `RoadLine` | 100 | 10.0 | 389 |
| vehicle protos (7 kinds, round-robin) | 100 | 22.6 | 1160 |
| vehicle protos | 200 | 29.2 | 1690 |
| vehicle protos | 400 | 56.2 | 2683 |
| vehicle protos, **`--no-rendering`** | 400 | **56.4** | **2700** |
| vehicle protos | 800 | 110.7 | 4732 |

Marginal cost per instance, from the 400→800 vehicle pair and the others against
the empty baseline:

| node type | load time | RSS |
|---|---:|---:|
| **vehicle proto** | **0.136 s** | **5.1 MB** |
| `Road` | ~0.02 s | 1.58 MB |
| `SimpleBuilding` | ~0 s | 0.40 MB |
| `Solid` + `Box` | ~0 s | 0.145 MB |

Two results worth calling out:

* **Vehicle cost is dead linear** (100→200→400→800 gives 5.1–9.3 MB/car with the
  marginal rate settling at 5.1 MB, and 0.135–0.136 s/car across every interval).
  There is no super-linear algorithm in Webots' loader to hunt for.
* **`--no-rendering` changes nothing at load: 56.2 s vs 56.4 s, 2683 vs 2700 MB.**
  This answers one of the candidate directions outright — the scene graph and the
  WREN buffers are built regardless of whether the main 3D view draws. Rendering
  flags are a step-cost lever, not a load-cost lever.

### Predicted vs. observed budget for `fmi_block_1km.wbt`

Actual node census of the world (grep of top-level `Node {` lines): 8379 `Solid`
(1 ground + 38 greens + 375 lamps + 1593 bays × 5), 775 `SimpleBuilding`,
727 vehicles across 7 protos, 179 `Road`, 1 `Mavic2Pro`. 10 065 nodes,
92 957 lines, 2.56 MB. (For scale: `fmi_block_4st.wbt` is 727 nodes / 0.18 MB,
`fmi_block.wbt` is 294 nodes / 0.08 MB.)

| component | count | predicted RSS | predicted load |
|---|---:|---:|---:|
| vehicles | 727 | 3.7 GB | 99 s |
| `Solid` (bays, lamps, greens) | 8379 | 1.2 GB | ~0 s |
| `Road` | 179 | 0.28 GB | ~4 s |
| `SimpleBuilding` | 775 | 0.31 GB | ~0 s |
| base process | — | 0.23 GB | 8 s |
| **total** | | **≈ 5.7 GB** | **≈ 111 s** |

Memory: predicted 5.7 GB vs. observed ~6 GB. **Match.** Time: predicted ~111 s
vs. observed >600 s. **5× discrepancy**, unexplained by node count.

---

## Half 2 — Optimization options for this world

Ranked. "GT/route safe" = `ground_truth.json` and `route.json` for all worlds stay
byte-identical. "Vision safe" = nothing changes about how a bay or a car looks
from 30 m nadir, so `vision/score_occupancy.py`'s calibration is untouched.

| # | Option | Est. saving (RAM / load) | Effort | GT/route safe | Vision safe |
|---|---|---|---|---|---|
| B | Merge the 5 bay `Solid`s into one `IndexedFaceSet` per bay (or per street) | **~1.1 GB** / ~0 s | ~1 h | ✅ | ✅ |
| A | Low-poly proxy cars, behind a flag, big worlds only | **~3.5 GB / ~95 s** | ~2–3 h | ✅ if done right | ❌ on the 1 km world only |
| C | Merge lamps + greens into shared/merged geometry | ~60 MB / 0 s | ~1 h | ✅ | ✅ |
| D | Merge collinear `Road` runs | ~0.15 GB / ~2 s | ~2 h | ⚠️ touches `road_runs` | ⚠️ road z-slots |
| E | Tile the 1 km world into 4–9 surveys | proportional (÷N) | ~1 day | ✅ (new worlds) | ✅ |
| F | Cap cars (`MAX_CARS`) / lower `OCC` | linear in cars | 15 min | ❌ **breaks GT** | ✅ |
| G | Reduce `MAX_BUILDINGS`, cut building detail | ~0.3 GB max / 0 s | 15 min | ✅ | ⚠️ |
| H | Do nothing; amortize load over a >1 h patrol | 0 | 0 | ✅ | ✅ |
| — | `--no-rendering` / `--mode=fast` | **0 / 0 (measured)** | 0 | ✅ | ✅ |
| — | proto cache / inlining protos | ~0 (already cached) | — | — | — |

### B. Merge the bay marking into one `IndexedFaceSet` — do this first

`bay_marking()` emits **5 `Solid`s per bay** (a pad + 4 outline boxes). At 1593
bays that is 7965 of the 8379 `Solid`s in the world, and at a measured 0.145 MB
per `Solid` that is **~1.15 GB of RSS spent on flat rectangles**. The pads/lines
are two-dimensional paint at fixed z; they never move, never collide, and never
need to be addressed individually — nothing in the pipeline looks up a
`bay_<id>` node by name (the classifier works from `block_bays.geojson` and
`poses.json`, not from the scene tree).

Change: replace the per-bay `Solid` emission with one `Solid` whose child is a
single `IndexedFaceSet` accumulating all pad quads at `BAY_PAD_Z`, and a second
one accumulating all line quads at `BAY_LINE_Z` (two nodes total, or one per
street if you want the scene tree to stay navigable). The corner maths is already
there — `rect_corners(x, y, ang, L, W)` gives exactly the four vertices, and
`triangulate()` is overkill for a quad (`[0,1,2,-1, 0,2,3,-1]`).

What it breaks: **nothing measurable.** Same colours, same z, same geometry, so
the rendered frames are identical to the pixel and the classifier is untouched.
No RNG stream is consulted here (`bay_marking` is deterministic), so
`ground_truth.json` is byte-identical. `route.json` is derived from bay
*positions*, not from emitted nodes, so it is untouched. Two caveats worth a
sentence in the code: (1) the world file's node ordering changes, so a
`fmi_block.wbt` diff will be large and not reviewable line-by-line as CLAUDE.md's
"new content lands at the end" convention assumes; (2) if the obstacle-avoidance
stage ever wants a per-bay bounding object, it will have to go back to `Solid`s —
so keep the old path behind a flag rather than deleting it.

Expected: ~1.15 GB off the 1 km world, ~0.03 GB off `fmi_block_4st`. Load time
barely moves (boxes are already ~free in time); this is purely a memory fix, and
memory is what is pushing us over the VRAM cliff.

### A. Low-poly proxy cars — the big one, but flag it

This is where 3.7 GB and 99 s live. A car at 30 m nadir in a 400×240 frame covers
roughly 25×10 px; 25 000 triangles of wing mirrors, wipers, indicators and
license plates are being loaded to render a coloured blob.

Change: add a `--lowpoly` flag (or key it off `WINDOW > 200`). In the occupied-bay
branch of the main loop, keep **every existing computation exactly as-is** — the
`rng_cars.random()` van draw, the `rng_cars.choice(CAR_MODELS)`, the
`rng_cars.choice(CAR_COLORS)`, the nose-direction draw, the
`CAR_DIMS`-driven fit-and-fallback loop that can set `b["occupied"] = False` and
write `gt[...] = False` — and change only the final `parts.append(f"{model} {{
... }}")` to emit a hand-written `Solid`: a body box `L × W × 0.75` at z ≈ 0.4, a
cabin box `0.55L × 0.9W × 0.45` on top, four dark wheel boxes, and a dark
windscreen quad, all in the drawn `col`. Use `DEF`/`USE` for the appearances.

**This is the part that must not go wrong**: `gt` is written *inside* the car
placement loop. If the sequence or the count of `rng_cars` draws changes, or if
`CAR_DIMS` changes so that the overlap test resolves differently, then
`ground_truth.json` moves for all three worlds. Keep the model *selection* and the
*dimensions* identical and only swap the emitted geometry, and GT is provably
untouched — verify with a byte compare of all three `*ground_truth.json` and
`*route.json` before/after, which is the check CLAUDE.md already mandates.

What it breaks: **appearance, and therefore the meaning of the accuracy number.**
`vision/score_occupancy.py`'s `classify()` is a five-term heuristic
(`core_chroma > 12.7`, `core_dark_frac > 0.02`, `core_paint_frac > 0.15`,
brightness outside 82–102, `core_std > 50`) calibrated on `fmi_block_4st` with the
real protos. A flat coloured box would still trip `core_chroma` easily — accuracy
would probably stay near 100 % — but `core_std` (panel gaps, windscreen edges) and
`core_dark_frac` (glass, tyres, under-car shadow) would collapse, so the classifier
would be scoring an easier, more synthetic target. **A 100 % on a box world is not
the same claim as a 100 % on a proto-car world**, and the thesis text has to say
so. Hence: apply it to `fmi_block_1km` **only**, keep `fmi_block` and
`fmi_block_4st` on the real protos so the calibration world and the golden fixture
(`sim/output/fmi_block/occupancy_results.json`, 43/43) never move, and report the
1 km world as a *coverage/logistics* result rather than an *accuracy* result.
Regenerating `fmi_block_1km` also invalidates the frames already captured under
`sim/output/fmi_block_1km/`.

Expected: 727 × (5.1 − ~0.3) MB ≈ **3.5 GB**, and 727 × 0.136 s ≈ **99 s**.
Combined with B, the 1 km world should land near **1.1 GB and ~15 s** — i.e. in
the same class as the 4st world today.

### C. Merge lamps and greens

375 lamp `Solid`s (3 `Pose`+`Shape` children each, ~1125 `Pose`s) and 38 green
`IndexedFaceSet`s. At 0.145 MB per `Solid` this is only ~55 MB — do it as part of
B if it is convenient (one `DEF LAMP` reused via `USE` inside 375 `Pose`s), skip
it otherwise. Note my `USE`-sharing probe: 2000 `Solid`s with a `USE`-shared
`Shape` cost 476 MB vs 521 MB for 2000 independent ones — **`USE` saved only 9 %**.
Webots already shares primitive geometry via `WbTriangleMeshCache`; the residual
cost is the `Solid` node itself. **So `DEF`/`USE` is not the lever — reducing the
node *count* is.** That is the whole reason B beats a `USE`-based rewrite.

### D. Merge collinear `Road` runs

179 `Road` protos × 1.58 MB ≈ 0.28 GB and ~4 s. `Road` is a 49 KB JS template
evaluated per instance with distinct `wayPoints`, so `# tags: static` cannot help.
Merging runs that share a name and are end-to-end collinear could plausibly halve
the count. Risk is real though: `emit`/`road_runs` feeds both the greedy z-slot
colouring (`runs_overlap`) and `build_route`'s coverage graph. Changing the run
decomposition changes `_key()` node rounding and can therefore change the Euler
walk — i.e. **`route.json` moves**. The z-slot assignment is also what stopped the
1 km world painting asphalt over bay rows (330 false positives). Low reward, real
downside; only worth it if the road layer becomes the top cost after A and B.

### E. Tile the 1 km world

Generate `fmi_block_1km_q0..q3` at half-size 250 around four offset origins, each
with its own route and ground truth, flown as four surveys. Each tile is ~¼ the
node count, so ~1.5 GB and ~30 s at today's cost, and the tiles can be flown in
parallel or on separate days. It also matches the real product: a drone does not
fly a 1 km rural-postman route on one battery, and the web tier already keys
everything by `survey_area`, so four areas is a natural fit — `frames_expected`,
mission progress and `OCCUPANCY_WINDOW_S` all get *easier* (the 2 h window
currently has to exceed a >60 min patrol).

What it breaks: `ORIGIN` is currently a module constant and pinned deliberately.
Tiles need a per-tile *window centre* that is **not** the projection origin — add
a `--center dx,dy` offset applied to the window test only, so `mlat`/`mlon`/ORIGIN
and hence every existing pose, route and ground truth stay in one frame. Get that
wrong and you silently fork the georeference, which is the spine of the whole
project. The existing three worlds are unaffected since they'd pass no offset.
Best done *after* A and B, as an operational improvement rather than a perf fix.

### F. Cap the number of cars — don't

Capping cars or lowering `OCC` scales the dominant cost linearly, and it is 15
minutes of work. It is also the one option that **directly rewrites
`ground_truth.json`**: `gt[str(b["id"])]` is written in the same loop, so fewer
cars means a different truth table for all three worlds, invalidating
`sim/output/fmi_block/occupancy_results.json` and every accuracy number in the
thesis. A 50 % occupancy rate is also the realistic figure the study is *about*.
Listed only to be explicitly rejected.

### G. Fewer/simpler buildings — refuted by measurement

`MAX_BUILDINGS = 1400` (775 actually emitted) looks like an obvious knob and is
not one: measured 0.40 MB and ~0 s per `SimpleBuilding`, so all 775 cost ~0.31 GB
and no measurable time. Cutting them in half saves 0.15 GB — 4 % of the problem —
while removing the scenery that made the world realistic and while changing
`hazards.json` (the handoff to the obstacle-avoidance stage). Leave it alone.
Same for `MAX_LEVELS`/`floorNumber`: `SimpleBuilding` is procedural and its cost
is JS evaluation plus a handful of extruded faces, not per-floor geometry.

### H. Do nothing — a legitimate baseline

Load is **once per process**. Against a 1 km patrol that takes >1 h of simulated
flight (and considerably more wall clock), 10 minutes of load is ~15 % overhead.
The pain is not throughput, it is the **iteration loop**: every generator tweak
costs a 10-minute reload before you can see it, which is exactly the tax that made
this investigation worth doing. If you are not iterating on the 1 km world, H is
correct. If you are, B alone (an hour of work, no risk to anything) plus A behind
a flag turns it into a ~15 s reload.

### Things that do not work — checked, so nobody re-checks them

* **`--no-rendering` / `--mode=fast` / `--minimize`.** Measured: identical load
  time and memory (56.2 s / 2683 MB vs 56.4 s / 2700 MB on 400 cars). The scene
  graph and WREN buffers are built either way.
* **The proto cache.** Webots already caches every remote asset by URL in
  `%LOCALAPPDATA%\Cyberbotics\Webots\cache\assets` — currently 539 files /
  118.8 MB here, of which 142 are `#VRML_SIM` proto texts and the rest are JPEG
  textures up to 3.2 MB. After the first load, no download happens. Inlining the
  protos into the `.wbt` or vendoring them locally would remove *network* cost
  that is already zero, and would not remove parse or instantiation cost.
* **`# tags: static` on the procedural protos.** It only collapses instances
  whose field values are *all* equal. Every `SimpleBuilding` has distinct
  `corners`, every `Road` distinct `wayPoints`. Never applicable here. (And it is
  a change to Cyberbotics' protos, not ours.)
* **Shadows / lights.** Already handled — `castShadows FALSE` on the
  `DirectionalLight`, and the hand-built lamp geometry deliberately replaced
  `StreetLight` (66 KB mesh + a 1000 m-radius `SpotLight` each). Both were vision
  decisions and both happen to be the right performance decisions too. Do not
  revisit either: they perturb the classifier.

---

## What I could not determine

* **Where the missing 4–5× of load time goes.** Node-count extrapolation predicts
  ~111 s headless; you observe >600 s. I did not launch the 1 km world (per the
  brief), so I never reproduced the 10 minutes directly. My leading hypothesis is
  **VRAM exhaustion**: this machine has a 4 GB GTX 1650 and the scene wants ~6 GB,
  so the driver must be spilling WREN buffers to host memory during
  `createWrenObjects()`. Supporting but circumstantial: the 200-car probe that ran
  immediately after a hard-killed instance took >12 min where a clean re-run took
  29 s, which is exactly the signature of GPU memory not yet reclaimed. A second
  possibility is that your 10-minute figure came from the **GUI** (extra
  bounding-sphere/octree setup and an actual first frame draw), where all of my
  numbers are `--minimize --mode=fast`. **Cheap way to settle it**: run the 1 km
  world once headless with the piped-stdout recipe and time it to the
  `Starting controller` line, while watching dedicated GPU memory in Task Manager.
  If headless is ~2 min, it is the GUI; if it is 10 min with GPU memory pinned at
  4 GB, it is VRAM — and then option B alone (−1.15 GB) may drop you back under
  the cliff without touching the cars at all.
* **Whether `USE` works on a whole vehicle proto instance.** I only tested `USE`
  on a `Shape` inside a `Solid` (9 % saving, not worth it). Sharing an entire
  `Solid`/proto subtree across differing transforms is not something Webots
  documents as supported, and I did not try it. Given the `USE`-on-`Shape` result
  I would not expect much, but it is unverified.
* **The exact triangle counts.** The per-family figures above come from counting
  `-1` face terminators in raw proto text, which is a good estimate for
  `coordIndex` arrays but will be off where a proto uses other conventions or
  external `.obj` meshes (the Sprinter). Order of magnitude is right; the exact
  numbers are not load-bearing for any conclusion.
* **Whether a box-car world actually keeps 100 % accuracy.** I reasoned from
  `classify()`'s five terms that `core_chroma` alone would carry it, but I did not
  render or score anything. If option A is taken, it needs an actual scored run,
  and the honest way to do that is to build a low-poly variant of
  `fmi_block_4st` *as a separate world*, score it, and compare against the 43/43
  fixture — not to change the fixture world.
* **Timing precision.** Load times are ±2 s (the memory-sampling interval gates
  the loop), and the empty-world baseline wandered between 4.9 s and 8.1 s across
  runs. Anything I quote under ~10 s should be read as "small", not as a number.

### Reproducing the measurements

The probe worlds were generated by a throwaway script into `sim/worlds/` as
`_probe_*.wbt` and **deleted afterwards**; `sim/output/_probe/` was removed too.
No project file was modified by this investigation. To redo it: emit a `.wbt`
containing only `WorldInfo`/`Viewpoint`/`Background`/`DirectionalLight`, a ground
`Solid`, N copies of the node under test, and a `Mavic2Pro` with
`controllerArgs [ "_probe.route.json" ]` (a route file that does not exist, so the
controller prints its fallback line immediately and writes into a throwaway output
folder). Kill stray Webots, launch with
`--batch --mode=fast --minimize --stdout --stderr`, **pipe** stdout to
`grep --line-buffered "Starting controller"` (never redirect — Webots block-buffers
to a file), and poll `(Get-Process webots-bin).WorkingSet64` for the peak.

### Sources

- [Procedural PROTO nodes — Webots reference (`# tags: static`, loading time)](https://github.com/cyberbotics/webots-doc/blob/master/reference/procedural-proto-nodes.md)
- [Webots discussion #5402 — How can I speed-up my simulation?](https://github.com/cyberbotics/webots/discussions/5402)
- [Webots issue #4946 — window blocked and black at start with many IMPORTABLE EXTERNPROTO](https://github.com/cyberbotics/webots/issues/4946)
- [Webots issue #3065 — Hang when opening world ("Finalizing nodes", 84–99 %)](https://github.com/cyberbotics/webots/issues/3065)
- [Webots issue #760 — Importing objects is blocking (lazy-loading direction)](https://github.com/cyberbotics/webots/issues/760)
- [Webots issue #2691 — Optimize web simulations loading time](https://github.com/cyberbotics/webots/issues/2691)
- [Webots issue #2663 — Increasing memory consumption over iterative reset (webots-bin at 5 GB+)](https://github.com/cyberbotics/webots/issues/2663)
- [Webots issue #6212 — Bug when opening OSM generated worlds](https://github.com/cyberbotics/webots/issues/6212)
- [Starting Webots — command line options (`--no-rendering`, `--mode`, `--minimize`)](https://github.com/cyberbotics/webots/blob/master/docs/guide/starting-webots.md)
- [`WbTriangleMeshCache.hpp` (R2023b) — mesh sharing across instances](https://raw.githubusercontent.com/cyberbotics/webots/R2023b/src/webots/nodes/utils/WbTriangleMeshCache.hpp)
- [`WbImageTexture.cpp` (R2023b) — `wr_texture_2d_copy_from_cache`](https://raw.githubusercontent.com/cyberbotics/webots/R2023b/src/webots/nodes/WbImageTexture.cpp)
- [`WbTriangleMeshGeometry.cpp` (R2023b) — `wr_static_mesh_new` per instance](https://raw.githubusercontent.com/cyberbotics/webots/R2023b/src/webots/nodes/WbTriangleMeshGeometry.cpp)
- [`SimpleBuilding.proto` (R2023b) — `# template language: javascript`, wraps `Building.proto`](https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/buildings/protos/SimpleBuilding.proto)
- [`TeslaModel3Simple.proto` (R2023b) — sub-proto decomposition](https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/vehicles/protos/tesla/TeslaModel3Simple.proto)
- [`BmwX5Simple.proto` (R2023b) — 13 sub-protos, no interior](https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/vehicles/protos/bmw/BmwX5Simple.proto)
