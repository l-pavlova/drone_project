# PARKDRONE — Webots simulation

A simulated FMI-block neighbourhood: ground, streets, painted parking bays (from
the real Sofia dataset), parked cars on a known-occupancy subset, OSM buildings /
green areas / light poles as scenery, and a Mavic 2 PRO drone with a downward
camera that flies a patrol and saves frames.

## Layout

```
sim/
  generate_world.py            # builds worlds/fmi_block.wbt from ../data/block_bays.geojson
                               # (+ block_roads.geojson streets, block_areas.geojson scenery)
  worlds/
    fmi_block.wbt              # the generated world  (regenerate any time)
    ground_truth.json         # bay_id -> occupied (true labels for scoring detection)
    <name>.hazards.json       # buildings that reach the 30 m flight level near the route
  controllers/parkdrone/
    parkdrone.py              # flight + lawnmower patrol + frame capture
  output/                     # frame_###.png + poses.json (created on run)
```

## Run it

1. Install **Webots** (free, Cyberbotics) — desktop app.
2. Regenerate the world if you want a different size:
   `python generate_world.py 75 0.5`  → 150×150 m window, ~50% occupied.
   (`python generate_world.py 250 0.5` = the full 500 m FMI block — heavier.)
3. Open `worlds/fmi_block.wbt` in Webots. It runs the `parkdrone` controller,
   which takes off to 30 m, flies the lawnmower, and writes frames + poses to
   `output/`.

## Known caveats (we'll fix these together once Webots is installed)

- **Untested here** — written to Webots conventions but not run. Expect to tune.
- **Proto version:** the world pulls `Mavic2Pro.proto` via EXTERNPROTO pinned to
  `R2023b`. If your Webots is a different release, change `WEBOTS_VER` in
  `generate_world.py` to match and regenerate.
- **Device names** (`inertial unit`, `gps`, `gyro`, `camera`, `front left
  propeller`, `camera roll`/`camera pitch`) assume the Mavic2Pro proto — verify in
  the Webots node tree if the controller errors.
- **Flight tuning:** the lawnmower navigator (K_YAW/K_FWD) is a first cut; if it
  overshoots or circles, lower the gains.
- **Camera angle:** the stock `Mavic2Pro` gimbal pitch only reaches **-0.5 rad
  (~-29 deg)** down, not true nadir — the controller is clamped to that. An oblique
  view still works (georeferencing uses the known camera orientation), but for a
  cleaner top-down survey we can upgrade to nadir by either (a) a local copy of
  `Mavic2Pro.proto` with the camera-pitch `minPosition` widened to ~-1.6, or
  (b) flying higher so the oblique footprint still covers the lane below. We'll
  decide once we see the first frames.

## Next

- Confirm it flies and captures clean downward frames.
- Feed `output/frame_*.png` into the detector (`../vision/`), map detections to bays
  via `poses.json` (georeferencing), and score against `ground_truth.json`.
