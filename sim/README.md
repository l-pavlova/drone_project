# PARKDRONE — Webots simulation

A simulated Sofia (Lozenets / FMI) neighbourhood: ground, OSM streets, painted
parking bays from the real Sofia dataset, parked cars on a known-occupancy
subset, OSM buildings / green areas / light poles as scenery, and a Mavic 2 PRO
drone with a downward camera that flies a street-following patrol and saves
nadir frames.

This is the working simulation stage of the project. It flies, it captures, and
its output is scored by `../vision/score_occupancy.py`. Deep design notes and the
hard-won invariants live in the repo-root `CLAUDE.md`; this file is the short
operational guide.

## Layout

```
sim/
  generate_world.py             # builds a world from ../data/block_bays.geojson
                                # (+ block_roads.geojson, block_areas.geojson)
  worlds/
    <name>.wbt                  # the generated world (regenerate any time)
    <name>.route.json           # the patrol waypoints
    <name>.ground_truth.json    # bay_id -> occupied (true labels for scoring)
    <name>.hazards.json         # structures reaching the 30 m flight level
    route.json / ground_truth.json          # the default world keeps these names
  controllers/parkdrone/
    parkdrone.py                # stabiliser + waypoint navigator + frame capture
                                # + the obstacle speed-limit layer
  output/<survey_area>/         # frame_###.png, poses.json, flight_log.csv
```

Worlds are per-scale file sets. The `.wbt` passes its route file to the
controller via `controllerArgs`, and the controller keys its output folder off
that name — so the survey area *is* the world name.

## The worlds

| world | build command | what it is |
|---|---|---|
| `fmi_block` | `generate_world.py 75 0.5 fmi_block --chase` | 150 m default; held-out scoring world |
| `fmi_block_4st` | `generate_world.py 130 0.5 fmi_block_4st --chase` | 260 m, 4 streets; the classifier **calibration** world |
| `fmi_block_1km` | `generate_world.py 500 0.5 fmi_block_1km --chase` | 1 km, 1593 bays, 727 cars, 1976 waypoints |
| `fmi_block_obst` | `generate_world.py 130 0.5 fmi_block_obst --collide --obstacles --chase` | obstacle-avoidance test world |
| `fmi_block_1km_lp` | `generate_world.py 500 0.5 fmi_block_1km_lp --chase --lowpoly` | 1 km with proxy box cars — fast, **not accuracy-comparable** |

Pass `--chase` whenever regenerating, or the GUI viewpoint reverts from the
ride-along "Mounted Shot" to the trailing "Tracking Shot".

## Flags

- `--chase` — ride-along viewpoint. Viewing only; never touches the drone's own
  camera or any captured frame.
- `--collide` — give buildings and poles bounding objects **and** mount the 9-ray
  DistanceSensor fan on the drone. Survey worlds get neither. Only for obstacle
  work: the survey controller flies a fixed 30 m with no avoidance logic.
- `--obstacles` — the synthetic obstacle course (`tower`, `slab`, `mast`, `trap`).
- `--lowpoly` — proxy box cars instead of vehicle protos. **Big worlds only.**
  It changes what a car looks like, so it changes what the accuracy number means
  (measured 98.2% vs 100%; the misses are dark cars). Never for `fmi_block` or
  `fmi_block_4st`.
- `--bay-solids` — restore the pre-2026-08-11 per-bay `Solid` emission. Only
  needed if something wants a per-bay scene node (e.g. a bounding object); it is
  ~1.1 GB and ~115 s more expensive on the 1 km world.

**After any change to `generate_world.py`, regenerate all worlds and check that
`ground_truth.json` and `route.json` come out byte-identical.** That is the
project's standing regression test — the RNG streams are deliberately split so
cosmetic changes cannot move the labels.

## Run it

Webots is at `C:\Program Files\Webots\msys64\mingw64\bin\webots.exe`. Open a
`.wbt` in the GUI, or run headless:

```bash
"/c/Program Files/Webots/msys64/mingw64/bin/webots.exe" \
  --batch --mode=fast --minimize --stdout --stderr worlds/fmi_block.wbt \
  2>&1 | grep -aE "reached|complete" | head -200
```

Three run-time gotchas, each of which cost real debugging time:

- **Kill stray Webots first** (`taskkill //F //IM webots-bin.exe; taskkill //F
  //IM webotsw.exe`). A leftover instance causes a port conflict and the next run
  hangs with no output — and it can slow a load by 25×.
- **Capture stdout by PIPING, not redirecting.** Webots block-buffers to a file
  and loses it when killed. (`grep` also block-buffers into a redirect — use
  `grep --line-buffered`.)
- **A folder with existing captures makes the controller RESUME, not re-fly.**
  Move `output/<area>/` aside for a clean run.

Disk output is the reliable source of truth even when the console is lost.

## Performance

`fmi_block_1km` used to take >10 minutes and ~6 GB to load. As of 2026-08-11 it
is **167 s / 5.3 GB**, and `fmi_block_1km_lp` is **57 s / 1.7 GB**. The wins came
from cutting Webots *node count*, not from rendering flags — `--no-rendering`
changes load cost by exactly zero. Full measurements and two refuted hypotheses
are in `../docs/webots_1km_performance.md`.

Webots runs here are **deterministic**: the same world and controller reproduce
frames, poses and scores exactly. A changed score means a changed world or
controller, never noise.

## Notes

- `Mavic2Pro.proto` is pinned to **R2023b** via `WEBOTS_VER` in
  `generate_world.py`. Change it if your Webots release differs, then regenerate.
- Device names (`camera`, `inertial unit`, `gps`, `gyro`, `camera roll`,
  `camera pitch`, `front/rear left/right propeller`) assume that proto.
- The gimbal **does** reach true nadir: pitch range is ~[-0.5, +1.7] rad where
  **down is positive**, so the controller sets `+pi/2`. (An older version of this
  file claimed otherwise; `getMinPosition()` points the camera *up* and yields
  sky-only frames.)

## Next

- Obstacle stage B: steer around, as a yaw bias on the navigator's own channel.
- Finish and score the 1 km flight (318/1976 frames captured so far).
- The off-nadir projection false positives — see "Known open issues" in
  `../CLAUDE.md`.
