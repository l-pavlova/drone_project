"""Live uplink: watch a flying sim survey and POST each frame as it lands on disk.

    API_KEY=<key> python -m parkdrone_vision.sim_uplink [survey_area] [api_base]
                        [--idle-exit S] [--poll S] [--from N] [--once]

Run this next to a Webots flight and the map updates while the drone is still
in the air, instead of after the fact.

**Why a sidecar and not HTTP inside the controller.** `parkdrone.py` is one
control loop: every millisecond it spends in a socket is a millisecond the
physics step does not run, and a hung server would fly the drone into a wall.
The controller already publishes everything needed — `frame_###.png` plus a
`poses.json` it rewrites after every capture — so this process reads that and
does the talking. The flight loop keeps no knowledge of the web tier at all.

**The ordering that makes it safe:** the controller saves the PNG *before* it
appends the pose and rewrites `poses.json`. So a pose appearing in the file is
proof its image is already complete on disk — no partial-file race, no
inotify-style guessing about write completion. (`poses.json` itself is rewritten
non-atomically, so a read can land mid-write; that is expected and retried.)

Disk stays the source of truth: this only reads. Delete nothing, and the
flight's own resume logic is untouched.
"""
import json
import os
import sys
import time
import urllib.error

from .config import SIM_OUTPUT_ROOT, SIM_WORLDS_ROOT
from .ingest_client import post_frame, post_json
from .vision.scoring import pose_idx

# Two stdout fixes, both learned the hard way on this project:
#   * UTF-8 — Windows consoles default to cp1252, and a UnicodeEncodeError on a
#     progress line would kill an uplink mid-flight, exactly when you cannot
#     afford to lose it (the tools/ data scripts do this for the same reason).
#   * line buffering — this runs for the length of a patrol (over an hour for
#     the 1 km route) and its output is normally redirected to a log. Python
#     block-buffers a redirected stream, so without this the log stays EMPTY
#     until the buffer fills, and anything unflushed is lost if the process is
#     killed. Same failure mode as Webots' own stdout (see CLAUDE.md).
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace", line_buffering=True)


def _flag(args, name, default, cast=float):
    """Read `--name value` out of argv, returning (value, remaining args)."""
    if name not in args:
        return default, args
    i = args.index(name)
    if i + 1 >= len(args):
        raise SystemExit(f"{name} needs a value")
    return cast(args[i + 1]), args[:i] + args[i + 2:]


def _read_poses(path):
    """Poses recorded so far, or None if the file is missing/mid-rewrite.

    The controller rewrites this whole file after every capture, so a read can
    catch it truncated. That is not an error — it resolves on the next poll.
    """
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return None
    except (OSError, ValueError):
        return None


def _expected_frames(survey_area):
    """Waypoint count from the route file, so the mission has a progress target."""
    names = [f"{survey_area}.route.json"]
    if survey_area == "fmi_block":
        names.append("route.json")  # the default world predates per-area names
    for name in names:
        path = os.path.join(SIM_WORLDS_ROOT, name)
        try:
            with open(path, encoding="utf-8") as f:
                route = json.load(f)
        except (OSError, ValueError):
            continue
        if isinstance(route, dict):
            route = route.get("waypoints") or route.get("route") or []
        return len(route) or None
    return None


def main() -> None:
    args = sys.argv[1:]
    idle_exit, args = _flag(args, "--idle-exit", 0.0)
    poll_s, args = _flag(args, "--poll", 1.0)
    start_from, args = _flag(args, "--from", 0, int)
    once = "--once" in args
    args = [a for a in args if a != "--once"]

    survey_area = args[0] if args else "fmi_block"
    api_base = args[1] if len(args) > 1 else "http://localhost:4000"
    api_key = os.environ.get("API_KEY")
    if not api_key:
        raise SystemExit("set API_KEY (from register_drone)")
    drone_id = os.environ.get("DRONE_ID", "drone-1")

    out_dir = os.path.join(SIM_OUTPUT_ROOT, survey_area)
    poses_file = os.path.join(out_dir, "poses.json")
    expected = _expected_frames(survey_area)

    print(f"uplink: {out_dir} -> {api_base}  (drone {drone_id}, area {survey_area})")
    print(f"  route target: {expected if expected else 'unknown'} waypoints · "
          f"poll {poll_s}s · " + (f"idle exit {idle_exit}s" if idle_exit else "runs until Ctrl-C"))

    sent: set[int] = set()
    mission_id = None
    posted = duplicates = errors = 0
    warned_duplicate = False
    last_progress = time.monotonic()

    try:
        while True:
            poses = _read_poses(poses_file)

            # A shorter file than what we have already sent means the survey area
            # was re-flown from scratch (its output dir was cleared). Start over
            # rather than sit there thinking everything is already uploaded.
            if poses is not None and sent and max(sent, default=-1) >= 0:
                highest = max((pose_idx(p) for p in poses), default=-1)
                if highest < max(sent):
                    print(f"! poses.json restarted (now ends at {highest}) — treating as a new flight")
                    sent.clear()
                    mission_id = None

            for pose in poses or []:
                idx = pose_idx(pose)
                if idx in sent or idx < start_from:
                    continue
                png_path = os.path.join(out_dir, f"frame_{idx:03d}.png")
                try:
                    if os.path.getsize(png_path) == 0:
                        continue  # still being written; next poll
                    with open(png_path, "rb") as f:
                        png = f.read()
                except OSError:
                    continue  # pose recorded but image not readable yet

                if mission_id is None:
                    try:
                        body = {"survey_area": survey_area, "area": "sim uplink"}
                        if expected:
                            body["frames_expected"] = expected
                        mission_id = post_json(
                            f"{api_base}/api/v1/ingest/mission/start", body, api_key
                        )["mission_id"]
                        print(f"mission {mission_id}")
                    except (urllib.error.URLError, OSError) as exc:
                        print(f"! mission start failed ({exc}) — retrying")
                        break

                meta = json.dumps({
                    "drone_id": drone_id,
                    "survey_area": survey_area,
                    "mission_id": mission_id,
                    "pose": pose,
                })
                try:
                    status = post_frame(
                        f"{api_base}/api/v1/ingest/frame", api_key, png, meta,
                        f"frame_{idx}.png",
                    )
                except (urllib.error.URLError, OSError) as exc:
                    print(f"! frame {idx} not sent ({exc}) — will retry")
                    break  # leave it unsent; the next poll picks it up again

                sent.add(idx)
                last_progress = time.monotonic()
                if status == 202:
                    posted += 1
                    print(f"  frame {idx:>4}  queued" + (f"  ({posted}/{expected})" if expected else ""))
                elif status == 200:
                    duplicates += 1
                    if not warned_duplicate:
                        warned_duplicate = True
                        # Ingest is idempotent on (drone, survey_area, frame_idx),
                        # so a re-flight of the same area is NOT reprocessed.
                        print(f"! frame {idx} already ingested — this survey area has been flown "
                              f"before, so these frames are IGNORED: no classify job, no "
                              f"delta, nothing new on the map.\n"
                              f"  Clear it and re-fly:  cd web && pnpm clear {survey_area}")
                else:
                    errors += 1
                    print(f"! frame {idx}: HTTP {status}")

            if once and poses is not None:
                break
            if idle_exit and sent and time.monotonic() - last_progress > idle_exit:
                print(f"no new frames for {idle_exit:.0f}s — assuming the patrol is done")
                break
            time.sleep(poll_s)
    except KeyboardInterrupt:
        print()  # keep the summary off the ^C line

    if mission_id:
        try:
            post_json(f"{api_base}/api/v1/ingest/mission/{mission_id}/end", {}, api_key)
            print(f"mission {mission_id} closed")
        except (urllib.error.URLError, OSError) as exc:
            print(f"! could not close mission ({exc})")

    print(f"uplink done: {posted} queued, {duplicates} duplicate, {errors} failed")
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
