"""Live uplink: watch a flying sim survey and POST each frame as it lands on disk.

    API_KEY=<key> python -m parkdrone_vision.sim_uplink [survey_area] [api_base]
                        [--idle-exit S] [--poll S] [--from N] [--once]
                        [--root DIR] [--pattern frame_%04d.jpg] [--rate FPS]

Run this next to a Webots flight and the map updates while the drone is still
in the air, instead of after the fact.

**It also uplinks REAL footage**, with `--root <stills dir> --pattern
frame_%04d.jpg`. That is a pair of flags rather than a second program on
purpose: everything hard here -- mission start/end, the retry that leaves an
unsent frame unsent, resume detection, idle exit, the ordering rule below -- is
identical for a directory of DJI stills, and a copy of it would be a copy that
drifts. The only real differences are where the images live, what they are
called, and that a real `poses.json` (from `tools/dji_stills.py`) numbers its
frames by `video_frame` rather than carrying a `frame_idx`.

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
from .vision.scoring import pose_idx as _sim_pose_idx

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


def frame_index(pose, order):
    """The frame number to ingest this pose under.

    A sim pose carries `frame_idx` (or the legacy `i`) and `pose_idx` reads it.
    A real pose from `tools/dji_stills.py` carries neither -- it has `file` and
    `video_frame`, because a still is identified by which video frame it was cut
    from. `order` (its position in poses.json) is the fallback, and it is the
    right one: ingest is idempotent per (drone, mission, frame_idx), so the
    index only has to be stable and unique WITHIN a flight, and a still's place
    in the list is exactly that. Using `video_frame` instead would work too but
    produces sparse five-digit indices that read as gaps in the ops dashboard.
    """
    try:
        return _sim_pose_idx(pose)
    except (KeyError, TypeError):
        return order


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
    rate, args = _flag(args, "--rate", 0.0)
    root, args = _flag(args, "--root", None, str)
    pattern, args = _flag(args, "--pattern", None, str)
    once = "--once" in args
    args = [a for a in args if a != "--once"]

    survey_area = args[0] if args else "fmi_block"
    api_base = args[1] if len(args) > 1 else "http://localhost:4000"
    api_key = os.environ.get("API_KEY")
    if not api_key:
        raise SystemExit("set API_KEY (from register_drone)")
    drone_id = os.environ.get("DRONE_ID", "drone-1")

    out_dir = root or os.path.join(SIM_OUTPUT_ROOT, survey_area)
    poses_file = os.path.join(out_dir, "poses.json")
    # A real stills directory has no route file; its frame count IS its pose
    # count, which is only known once the extraction has finished, so the
    # mission simply has no plan-vs-actual target. That is honest -- inventing
    # one would put a fictional denominator on the ops dashboard.
    expected = None if root else _expected_frames(survey_area)

    print(f"uplink: {out_dir} -> {api_base}  (drone {drone_id}, area {survey_area})")
    print(f"  route target: {expected if expected else 'unknown'} waypoints · "
          f"poll {poll_s}s · " + (f"idle exit {idle_exit}s" if idle_exit else "runs until Ctrl-C")
          + (f" · replaying at {rate} frame/s" if rate > 0 else ""))

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
                highest = max((frame_index(p, n) for n, p in enumerate(poses)),
                              default=-1)
                if highest < max(sent):
                    print(f"! poses.json restarted (now ends at {highest}) — treating as a new flight")
                    sent.clear()
                    mission_id = None

            for order, pose in enumerate(poses or []):
                idx = frame_index(pose, order)
                if idx in sent or idx < start_from:
                    continue
                # Prefer the pose's OWN filename when it has one: a real stills
                # directory records what it wrote, and reconstructing the name
                # from an index would have to re-guess the extraction's numbering.
                if pose.get("file"):
                    png_path = os.path.join(out_dir, pose["file"])
                elif pattern:
                    png_path = os.path.join(out_dir, pattern % idx)
                else:
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
                        body = {"survey_area": survey_area,
                                "area": "real uplink" if root else "sim uplink"}
                        if expected:
                            body["frames_expected"] = expected
                        mission_id = post_json(
                            f"{api_base}/api/v1/ingest/mission/start", body, api_key
                        )["mission_id"]
                        print(f"mission {mission_id}")
                    except (urllib.error.URLError, OSError) as exc:
                        print(f"! mission start failed ({exc}) — retrying")
                        break

                # Stamp the index INTO the pose. The ingest endpoint reads it
                # with pose_idx(), and a real pose from tools/dji_stills.py has
                # no frame_idx to read -- it is identified by `file`. Done here
                # rather than on disk because poses.json is the record of what
                # was captured, and the index is a fact about this upload.
                meta = json.dumps({
                    "drone_id": drone_id,
                    "survey_area": survey_area,
                    "mission_id": mission_id,
                    "pose": {**pose, "frame_idx": idx},
                })
                try:
                    status = post_frame(
                        f"{api_base}/api/v1/ingest/frame", api_key, png, meta,
                        os.path.basename(png_path),
                    )
                except (urllib.error.URLError, OSError) as exc:
                    print(f"! frame {idx} not sent ({exc}) — will retry")
                    break  # leave it unsent; the next poll picks it up again

                sent.add(idx)
                if rate > 0:
                    # DEMO PACING. A live flight paces itself -- frames appear
                    # on disk as the drone captures them -- but a directory of
                    # already-extracted stills has no such clock, so the uplink
                    # would empty it as fast as the server accepts, and a map
                    # that repaints in eight seconds shows nothing happening.
                    # --rate 1.0 replays the 1 Hz stills at the speed they were
                    # flown, so the map advances in step with the rendered
                    # video beside it. It is a REPLAY, not a live feed, and the
                    # flag name says so.
                    time.sleep(1.0 / rate)
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
                        print(f"! frame {idx} was already ingested IN THIS MISSION, so it is "
                              f"skipped — that is the retry case working.\n"
                              f"  Since migration 0009 a re-FLIGHT is a new mission and ingests "
                              f"on its own, so seeing this across a whole flight means the "
                              f"mission id was reused — check that mission/start was called.")
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
