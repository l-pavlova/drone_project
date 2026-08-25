# MathHelper

Math / physics concepts used in PARKDRONE, as short reminders.

## Radians ↔ degrees

Half a circle = 180° = π rad, so 1 rad ≈ 57.3°.

- rad → deg: `× 180/π` (e.g. 0.4 rad ≈ 23°)
- deg → rad: `× π/180` (e.g. 30° ≈ 0.524 rad)

All project math (Webots rotations, yaw errors, gimbal angles) is in radians.

## Axis-angle rotation (Webots `orientation`)

Four numbers `x y z θ`: rotate by angle θ (radians) around axis (x, y, z),
right-hand rule. `0 1 0 0.4` = tilt down ~23° about the y-axis.

## Braking distance (`v² / 2a`)

From constant deceleration `a`, stopping from speed `v` takes

```
d = v² / (2a)
```

Quadratic in speed: doubling the speed **quadruples** the distance.

In PARKDRONE this sizes the obstacle sensor, not the other way round. Tilt is
capped for stability and the sim has no drag, so `a ≈ 0.22 m/s²`; from
`V_MAX = 5 m/s` that is `25 / 0.44 ≈ 57 m`. A 40 m rangefinder therefore *cannot*
stop this drone at cruise speed — hence the 80 m range. Add a reaction allowance
(`v · t_react`) on top, because the aircraft does not begin braking the instant it
sees something.

The same relation runs backwards along the route to set the speed profile:
`v_allowed² = v_next² + 2·a·d`, applied from the last waypoint upstream, gives the
fastest speed at each waypoint that can still be braked down for the corner ahead.

## Tangent angle to a circle (`arcsin(R/d)`)

Half the angular size of an obstacle: from a point at distance `d` from the centre
of a circle of radius `R`, the angle between the line to the **centre** and either
tangent line is

```
θ = arcsin(R / d)
```

Because the radius meets the tangent at a right angle, centre / touch point /
viewpoint form a right triangle with opposite `R` and hypotenuse `d`.

- `d → R` (touching the circle): `θ → 90°` — turn almost side-on.
- `d` large: `θ → 0` — barely deviate.
- `d < R`: undefined. That is the "already inside the disc" case, where steering
  is meaningless and braking is the only answer.

Stage B's steering law is `heading = bearing(centre) ± (arcsin(R/d) + margin)`:
turn away from the centre by just enough to graze the edge, plus a margin that
turns "grazing" into "passing clear". Which sign depends on which side has more
room in the ray fan.

## Camera footprint at altitude

For a pinhole camera at altitude `h` with horizontal field of view `fov`, looking
straight down:

```
width = 2 · h · tan(fov / 2)
```

At `h = 30 m`, `fov = 0.785 rad (45°)`: `width ≈ 24.8 m` across 400 px, i.e.
**≈ 16 px per metre**. That is why waypoints are 10 m apart (the footprint is only
~25 × 15 m) and why flying higher trades resolution for coverage: at 50 m the
footprint grows ~1.7× while the scale drops to ~10 px/m.

## WGS84 (EPSG:4326)

The globe-shaped coordinate system GPS reports and the one we store: **lon/lat in
degrees** on an ellipsoid (equatorial radius a ≈ 6 378 137 m, flattened ~1/298.257
at the poles). Angles, not metres — so you cannot subtract two of them and call the
result a distance.

- **Order matters.** GeoJSON/PostGIS write `[lon, lat]`; Leaflet and GPS readouts
  say `lat, lon`. Every conversion between them is a flip.
- **A degree is not a fixed length.** 1° of latitude ≈ 111 320 m everywhere, but
  1° of longitude ≈ `111 320 · cos(lat)` — at Sofia (lat ≈ 42.67°) that is
  ≈ 81 900 m, only ~74% of a latitude degree.
- That cosine is exactly the project's flat-earth projection to local ENU metres:
  `x = (lon − lon₀)·111320·cos(lat₀)`, `y = (lat − lat₀)·111320`. Valid because a
  city block is small enough for the curvature to be negligible.

Rule of thumb: WGS84 for **storing and sharing** geometry (the `bay` table, the
GeoJSON files), local ENU metres for **doing maths** (pose, camera footprint,
distances).

## ENU (local East-North-Up metres)

A flat Cartesian frame in **metres**, pinned to one origin point on the ground:
**x = east, y = north, z = up**. Turns round-earth degrees into plain vectors you
can add, subtract and take distances of.

The project's projection (`ORIGIN` in `generate_world.py`, mirrored in
`score_occupancy.py` and `packages/contracts`):

```
x = (lon − lon₀) · 111320 · cos(lat₀)
y = (lat − lat₀) · 111320
```

- One frame for everything: bay rectangles in the Webots world, the drone's
  `x, y, alt, yaw` in `poses.json`, and the reprojection of a detected car back
  onto a bay. **`ORIGIN` must never drift** or those stop lining up.
- Webots' world axes are ENU-like; identity orientation looks along **+x** (east).
- **NED** (North-East-Down) is the same idea with axes permuted and z flipped —
  it is what ArduPilot/MAVLink use, so altitudes there are *negative* upwards.
  Watch the sign when crossing between flight logs and the sim.

## Gradient descent & the learning rate

Training moves each weight downhill on the loss surface:

```
w ← w − lr · ∂L/∂w
```

The gradient gives only the **direction**; `lr` (the learning rate, the step size)
gives **how far** to go. The direction is a *local* slope, valid only near the
current point — so too large a step overshoots the valley and lands higher on the
far side, and repeating that diverges. Too small a step descends correctly but
takes forever.

Two things make the step especially delicate when fine-tuning the car detector:

- **We start from good weights, not random ones.** `yolov8s.pt` already knows what
  a car is; a big step from near the solution destroys that knowledge rather than
  improving it (catastrophic forgetting).
- **An epoch is 11 optimizer steps** (22 train images, batch 2). Ultralytics'
  defaults assume thousands of steps per epoch, where one bad step is averaged away
  by the next thousand. Here one bad step *is* 1/11 of the epoch.

Measured in `vision/runs/probe1` (`docs/training.md` §3.1): `optimizer="auto"`
ignores `lr0` and picked `AdamW(lr=0.002)`. The 3-epoch warmup ramps the rate from
0.0002, so epoch 4 peaks at mAP50 **0.714** — then epoch 5, the first at ~0.001,
collapses to **0.005** and never beats the peak again. `probe2`/`merged1` fix it
with an explicit `lr0=0.0005`, a quarter of what auto chose, and climb smoothly.

How each failure looks on the curve:

| symptom | cause |
|---|---|
| train loss *rises*, metrics drop to ~0 in one epoch, then crawl back | step too large |
| everything falls, but far too slowly, no plateau | step too small |

**Warmup** exists for this: start at a fraction of `lr0` and ramp up over the first
few epochs, so the first (worst-informed) gradients cannot wreck the initial
weights. **Decay** is the mirror image — shrink the step toward the end so the model
settles into the minimum instead of bouncing around it.
