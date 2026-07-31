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
