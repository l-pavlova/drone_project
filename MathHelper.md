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
