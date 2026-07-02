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
```
`spaces_25.geojson` (31,705 parking-space points) and `zones_34.geojson` are the source datasets from the Sofiaplan API; see `data/README.md` for schema and field meanings (Bulgarian property values).

### 2. Webots simulation (run from `sim/`)
```bash
python generate_world.py [half_m] [occ_frac]   # default 75 0.5 -> worlds/fmi_block.wbt + worlds/ground_truth.json
```
This reads `../data/block_bays.geojson` (plus `block_roads.geojson` if present), projects to local metres, and emits the world (ground, OSM streets as Webots `Road` protos, painted bays, real car models — 7 vehicle Simple protos — on a known-occupancy subset, follow-drone viewpoint) plus `ground_truth.json` (bay_id -> occupied). Car/model randoms come from a separate `random.Random(7)` stream so `ground_truth.json` stays stable. `DirectionalLight` has `castShadows FALSE` — shadow mapping paints streak artifacts on the road/ground in the nadir frames. Then run the world (see "Running Webots" below). The route (`worlds/route.json`) is a depth-first walk of the OSM street centerlines of every street that has bays (start at the westernmost street end, shortest branch first at each junction, backtracks/transits along the roads) — waypoints every 10 m because the camera footprint at 30 m is only ~25×15 m. The controller `controllers/parkdrone/parkdrone.py` takes off to 30 m, flies that route (square-lawnmower fallback if route.json is missing), and writes `output/frame_###.png` + `output/poses.json` at each waypoint (plus timed diagnostic `snap_###.png`).

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
- The controller writes `frame_###.png` / `poses.json` directly to `sim/output/`, so disk output is the reliable source of truth even if console is lost.
- `EXTERNPROTO` for `Mavic2Pro.proto` is pinned to **R2023b** via `WEBOTS_VER` in `generate_world.py`; change it if your Webots release differs, then regenerate the world. Controller device names (`camera`, `inertial unit`, `gps`, `gyro`, `camera roll`, `camera pitch`, `front/rear left/right propeller`) assume that proto.

## Architecture (the big picture)

**Coordinate flow / georeferencing is the spine of the project.** Everything is tied together by a single local projection: lon/lat (EPSG:4326) → local ENU metres about the block centroid, using `mlat = 111320`, `mlon = 111320*cos(lat0)`. Both `generate_world.py` and the controller use this same convention. `poses.json` records the drone's `x, y, alt, yaw` in those metres at each captured frame — so a detected car's image position can be projected to ground metres and matched to the nearest bay in `block_bays.geojson`, then scored against `ground_truth.json`. When touching projection math, keep `generate_world.py`, the controller, and (future) the detector consistent.

**`parkdrone.py` is one control loop.** Each step: read IMU/GPS/gyro → point gimbal to nadir → optionally capture a frame → compute roll/pitch/yaw/vertical disturbances → mix into four propeller velocities (mixing & base gains adapted from Webots' official Mavic2Pro sample). On top of the stock stabilizer sits a **lawnmower waypoint navigator** that steers car-style: yaw to point the nose at the next waypoint, then throttle forward.

Hard-won controller invariants — **do not regress these** (they are why the sim works now; details in the project memory):
- **Camera nadir uses POSITIVE pitch.** The Mavic2Pro `camera pitch` range is ~`[-0.5, +1.7]` rad where **down is positive**; set `+pi/2` for true nadir. `getMinPosition()` (-0.5) points the camera *up* and yields sky-only frames.
- **Altitude needs vertical-velocity damping** (`K_VD`), or it overshoots ~30→50 m and crashes.
- **Yaw needs rate damping** (`K_YAWD`, from the gyro's yaw rate), or it is pure-proportional and the drone spins in circles, never facing a waypoint.
- **Steer with yaw + forward only; never roll-strafe toward the target** — a lateral *position* command saturates while off-heading and tumbles the drone. Roll is used only to damp sideways drift.
- **Waypoint arrival: keep `WP_REACH` at 6 m and capture at closest approach.** At cruise speed the turn radius is ~4 m, so the drone can settle into a stable ORBIT inside a tighter basin (constant distance — the patrol hangs forever, circling). Arrival fires on `WP_CAPTURE` (2.5 m), receding >1 m past the closest pass, or a `WP_TIMEOUT` (8 s) orbit bail-out.
- **`TILT_MAX` > ~1.0 dips lift and crashes.** Forward speed is a velocity-target controller (`v_des = clamp(K_POS*fwd_err, 0, V_MAX)`) that ramps down on approach so row-end U-turns stay tight; once the patrol finishes the controller station-keeps (brakes drift) instead of sailing off.
- Working gains live at the top of the loop: `K_YAW=1.0 K_YAWD=0.8 K_POS=0.6 K_VEL=0.4 TILT_MAX=1.0 V_MAX=2.5` (plus the Webots-sample stabilizer gains `K_VT/K_VP/K_ROLL/K_PITCH`).

## Windows / Git Bash conventions

- The shell is Git Bash. **Windows backslash paths break** in commands — use forward slashes (`/c/Users/...`) or quote carefully.
- The data scripts deliberately use Python `urllib` + UTF-8 (`sys.stdout.reconfigure(encoding="utf-8")`) instead of shelling out, because **Git Bash mangles Cyrillic** (street names, zone types are in Bulgarian). For manual downloads use `curl --ssl-no-revoke` (Windows cert revocation is flaky); the scripts already disable cert verification for these public read-only GETs.
