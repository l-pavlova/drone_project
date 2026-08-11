# Collision detection & obstacle avoidance — strategy options

Written 2026-08-04 as the decision input for the next task after the scenery work.

> ## STATUS 2026-08-11 — decisions taken, stage D shipped
>
> **The user chose STEER (not climb), and confirmed the staged order: sensors first, then
> avoidance.** So the open question at the bottom of §4 is settled — a coverage gap on the
> deviated segment is accepted, and the frames are marked; a mis-calibrated frame producing a
> *wrong* occupancy answer is not.
>
> ### Done
> - **§4.1 test world** — `fmi_block_obst` (`generate_world.py 130 0.5 fmi_block_obst --collide
>   --obstacles --chase`). Same window/route/bays as `fmi_block_4st`, plus a four-structure
>   course anchored to fractions along the finished route: `tower` 45 m head-on, `slab` 38 m
>   offset 7 m, `mast` 40 m × 0.9 m across, `trap` 34 m U opening toward the drone. Placement
>   skips bays and enforces 30 m between structures. They land in `<name>.hazards.json` flagged
>   `"synthetic": true`. All three survey worlds still regenerate byte-identical.
> - **§4.2 sensable world** — `--collide` now also gives the generated light poles a Cylinder
>   bounding object. (The note below about `StreetLight` having none is obsolete: poles became
>   procedural primitives in the scenery work, so it was a two-line change, not a sibling Solid.)
> - **§4.3 stage D** — 9-ray `DistanceSensor` fan (±40°, 80 m, `type "laser"`) in the Mavic2Pro
>   `bodySlot`, emitted only with `--collide`. Detection logged, brake-to-hover, navigator
>   untouched. **Verified: acquires `tower` at 76 m and halts at a 12.20 m standoff, held to
>   ±2 cm for 400+ s at 30.00 m.** Archived `sim/output/fmi_block_obst.stageD-halt/`; the
>   obstacle-blind baseline crash is in `sim/output/fmi_block_obst_baseline/`.
> - **§4.6 frame marking** — `"avoiding": true` + `"obstacle_m"` land in `poses.json` now, as
>   this document recommended, so the scorer can exclude them later.
> - New: `output/<area>/flight_log.csv`, line-buffered, one row per second. Webots' stdout kept
>   being lost and the decisive seconds fall *between* waypoint captures.
>
> ### What stage D taught (seven runs; details and constants in CLAUDE.md)
> - **Sensor range is a dynamics spec, not a perception one.** `A_BRAKE` ≈ 0.22 m/s² (tilt capped
>   at ~2°, no drag), so stopping from `V_MAX` 5 m/s needs `v²/2a` ≈ 57 m. A 40 m rangefinder
>   cannot stop this aircraft at any level of algorithmic cleverness.
> - Braking on the raw closest return is worse than useless: the wide rays grab scenery off to
>   the side, the minimum flips between targets, and the limit *rises* mid-brake. Needs a
>   bearing/corridor gate plus a ratchet that only ever lowers the limit.
> - The bare braking curve permits a drone below it to keep accelerating — true only for a
>   vehicle that can brake instantly. This one takes ~25 m to reverse a trend, so the curve
>   charges 3 s of travel against the measured range.
> - Proportional velocity control fades to zero effort as it approaches target (measured
>   0.133 m/s² against a planned 0.15). Braking against an obstacle wants saturated tilt.
> - **Stopping is not enough.** It stopped correctly at 12.7 m, then crept 13 m into the tower
>   over 95 s: damping opposes velocity and cannot null a steady hover drift. A halted drone
>   must station-keep on a latched point.
>
> ### Not started
> **§4.4 stage B — steer around**, as a yaw bias on the same channel principle (never a lateral
> position command; the threat bearing is already read and logged for it). Relevant new
> evidence: with 0.13–0.29 m/s² of authority, steering needs far less room than stopping, which
> is the quantitative argument that B is the real answer and D is only a supervisor.
> **§4.5 (offline A1/A2 pre-check)** and the `mast` / `slab` / `trap` cases are all still open —
> only `tower` has been flown.

---

## 1. What we actually have to work with

**The drone today.** `sim/controllers/parkdrone/parkdrone.py` is one control loop that holds
`TARGET_ALT = 30.0` m and has **zero** obstacle logic. It reads IMU/GPS/gyro, points the gimbal to
nadir, and steers car-style toward the next waypoint (yaw to aim, then throttle forward). There is
no forward sensor on the robot node at all.

**The hazard list.** `generate_world.py` already writes `worlds/<name>.hazards.json` — every
building whose roof reaches the 30 m flight level within 10 m of the route:

| world | hazards |
|---|---|
| `fmi_block` | `[]` — empty |
| `fmi_block_4st` | `[]` — empty |
| `fmi_block_1km` | **3 buildings**: 42 m at 8.6 m from route, 27 m at 9.5 m, 27 m at 9.5 m |

**This is the single most important planning fact:** *the only world with anything to hit is the
one that takes >10 minutes and ~6 GB to load.* Both fast dev worlds are hazard-free at 30 m. Any
option below needs a purpose-built small test world with a deliberate conflict, or the
edit–run–observe loop is 10+ minutes per iteration.

**What is sensable.** Webots range devices only see nodes that have a `boundingObject`. Right now:

- Buildings — `enableBoundingObject` is wired to the `--collide` flag, **off by default**.
- Light poles — generated primitives via the `solid()` helper, which emits `Solid { children [ Shape ] }`
  with **no `boundingObject` field at all**. Invisible to sensors *and* to physics.
- Bay pads, paint lines, roads, greens — same `solid()` helper, also no bounding object. Harmless
  (they're flat on the ground) but worth knowing before enabling anything scene-wide.
- Cars — vehicle protos, which do carry their own bounding objects.
- Ground — a `Plane` bounding object already.

So "make the world sensable" is not one switch: it's `--collide` **plus** giving poles a bounding
object (a sibling `Solid` with a `Cylinder`).

---

## 2. The constraint that shapes every option

**Avoidance manoeuvres fight the vision stage.** The occupancy classifier is calibrated at 30 m
nadir, and `score_occupancy.py` projects bays through a pinhole model using exactly that altitude
and the recorded gimbal angles. Two consequences:

- **Climbing over an obstacle changes the ground sample distance** (~16 px/m at 30 m) and takes the
  scene outside the calibration the classifier was tuned on. Frames captured mid-climb are not
  comparable to the rest of the survey.
- **Steering laterally moves the bay row out from under the camera.** Coverage is already the known
  weak spot (~4 uncovered bays from capture scatter); a 10 m detour past a building is a guaranteed
  gap in exactly the segment where the drone was supposed to be looking.

Neither is a blocker — but whichever option is chosen, the **frames captured during an avoidance
manoeuvre must be marked**, so the scorer can exclude or down-weight them rather than silently
scoring a bay that was never properly in frame. That's a small addition to `poses.json` (an
`avoiding: true` flag) and it should land in whichever option is picked.

---

## 3. The options

### Option A — Map-based avoidance, no sensors (offline path planning)

Use the building footprints and heights *already parsed* in `generate_world.py` to make the route
safe before the drone ever flies. Two sub-variants:

- **A1, altitude profile:** raise cruise altitude over conflicting segments.
- **A2, lateral nudge:** push conflicting waypoints away from the footprint, keeping 30 m.

**For:** deterministic and repeatable; costs zero physics; needs no bounding objects, so the slow
1 km world doesn't get slower; fully testable offline (the route is a JSON file — you can verify
clearances with a script, no Webots run at all). Reuses geometry we already have.

**Against:** it is *path planning*, not obstacle avoidance — it cannot react to anything OSM doesn't
know about (cranes, trees, wires, another drone), which is precisely the thesis claim. A1 breaks the
vision calibration as described above; A2 trades collision risk for coverage loss.

**Verdict:** cheap and genuinely useful as a *baseline safety net*, but on its own it doesn't
deliver the stated task.

---

### Option B — Reactive sensing + local avoidance (the full version)

Add range sensing to the drone node (a forward `DistanceSensor` fan, or one `Lidar` with a
horizontal layer), regenerate worlds with `--collide`, give poles bounding objects, and insert an
avoidance layer between the navigator and the stabilizer.

**For:** this is the real deliverable, and it mirrors the hardware vision in the master plan
(Pixhawk + RPi + range sensors). It handles the unmapped obstacle. It is the version that makes a
thesis chapter.

**Against:** three real costs.
1. It touches the control loop — the one file full of hard-won invariants (no roll-strafe toward a
   target, `TILT_MAX` ≤ 1.0, the arrival/orbit logic). Any avoidance that issues a lateral position
   command will reproduce the tumble bug that took days to find the first time.
2. `--collide` adds collision geometry for ~775 buildings on the 1 km world, which is already the
   performance problem under investigation.
3. Pure reactive avoidance deadlocks in concave geometry (a courtyard traps it), so it needs at
   least a timeout-and-climb escape hatch anyway.

**Integration point, if chosen:** the avoidance layer should output a **speed limit and a yaw bias**,
not a position target — i.e. it feeds the same two channels the existing navigator already uses
(`v_des` and the aim point). That keeps every invariant intact by construction, because the
stabilizer never sees a new kind of command.

---

### Option C — Hybrid: hazard list arms the sensors

Load `<world>.hazards.json` in the controller, and enable/act on the range sensors only when within
some radius of a known hazard. Map knowledge decides *when to pay attention*; sensing decides *what
to do*.

**For:** keeps sensor cost near zero over the 99% of the route with nothing near it (relevant given
the 1 km world's performance). Gives a natural place to set the `avoiding` flag on frames. Degrades
gracefully — an unmapped obstacle in an unarmed segment is still a miss, but the mapped ones are
covered twice.

**Against:** more moving parts than either pure option, and the "only look where the map says" logic
is exactly the assumption a reviewer will poke at. Mitigable by making the arming radius generous.

---

### Option D — Safety supervisor only (detect and stop)

Sense, but don't route around: on a detection, brake to a hover and either hold, climb straight up,
or abort the leg. No lateral planning at all.

**For:** the smallest possible change, provably safe, and it cannot corrupt the survey geometry
because the drone never leaves the route laterally. A good *first* increment regardless of the final
choice — it proves the sensing rig works before any avoidance logic depends on it.

**Against:** not avoidance; a patrol that stops in front of a building never finishes.

---

## 4. Recommendation

**Stage it: D → B, with A as the always-on backstop; C's arming trick only if performance demands.**

1. **Build a small dedicated test world with a deliberate conflict** — a hazard-free 4-street-sized
   world with one tall building placed on the route. Without this, every iteration costs a 10-minute
   load. This is the actual first task, before any controller code.
2. **Make the world sensable:** `--collide` for buildings, plus a `boundingObject` on the generated
   poles (they have none — the `solid()` helper emits no bounding object at all).
3. **Ship D first:** sensors on the drone, detection logged, brake-to-hover on contact. Verifies the
   rig end to end without touching the navigator.
4. **Then B**, with avoidance expressed strictly as a **speed limit + yaw bias** into the existing
   navigator channels — never as a lateral position command.
5. **Keep A1/A2 as an offline pre-check** that runs at generation time and warns (as `hazards.json`
   already does) when the planned route cannot be flown safely at 30 m.
6. **Mark avoidance frames in `poses.json`** so the scorer can exclude them.

Open question for you: **does the avoidance manoeuvre prefer climbing or steering?** Climbing keeps
the bay row under the camera but breaks the classifier's altitude calibration; steering keeps the
altitude but loses coverage on the deviated segment. My inclination is **steer, and mark the frames
as unscored** — coverage gaps are already a known, measured limitation, whereas a silently
mis-calibrated frame produces a *wrong* occupancy answer, which is worse than no answer.
