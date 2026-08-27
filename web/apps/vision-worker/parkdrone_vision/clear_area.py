"""Wipe a survey area so it can be flown and scored again from scratch.

    python -m parkdrone_vision.clear_area <survey_area> [flags]
    (from web/:  pnpm clear <survey_area>)

**Why this exists.** Ingest is idempotent on `(drone_id, survey_area,
frame_idx)`, so re-flying an area that is already in the database posts frames
that come back 200-duplicate: no classify job, no `bay_state` change, and
therefore no WebSocket delta. The map just sits there while the drone flies,
and nothing in the logs looks broken — the uplink says "already ingested" once
and then goes quiet. Clearing the area first is the fix, and doing it by hand
means four DELETEs in the right order plus the object store plus the capture
folder, which is exactly the kind of thing that gets half-done.

What it removes, and why each one is needed:
  * `frame` rows (CASCADE takes `frame_job`) — the idempotency ledger. Without
    this the re-flight is ignored outright.
  * the frame images in the object store — orphaned the moment their rows go.
  * `observation` rows for the area — the per-look history the occupancy vote
    is computed from. Left behind, the old votes keep out-voting the new looks.
  * `bay_state` rows for the bays this area observed — deltas fire on a
    *change*, so a state that is already correct pushes nothing and the map
    never repaints. Note `bay_state` has no `survey_area` column (a bay has one
    current answer, whoever saw it), so the rows to drop are resolved from the
    observations *before* those are deleted.
  * `bay_delta` rows announcing those bays — the WebSocket replay log. A client
    reconnecting with an old `?since=` would otherwise be replayed news about
    bays this command just deleted. Resolved from the observations for the same
    reason `bay_state` is.
  * `mission` rows for the area — otherwise the ops dashboard keeps showing
    stale plan-vs-actual progress for a flight that no longer exists.
  * the on-disk captures in `sim/output/<area>/` — `frame_*.png`, `snap_*.png`,
    `poses.json`, `flight_log.csv`. The controller RESUMES from `poses.json`,
    so leaving it behind means the next run continues the old patrol instead of
    re-flying it. `--db-only` keeps them.

**A scored fixture is refused.** An output folder holding
`occupancy_results.json` is a verification fixture (`replay.py` compares the
live pipeline against it); deleting the frames it was computed from would
quietly invalidate the golden test. `--force` is the explicit opt-in.

**A running uplink recovers by itself.** `sim_uplink` keeps the frame indices it
has sent in memory, but it watches for `poses.json` restarting at a lower index
and starts over when it does — which is precisely what a cleared folder plus a
fresh flight produce. Clearing only the database (`--db-only`) while an uplink
is mid-flight does NOT re-post what it already sent: restart it in that case.
"""
import os
import re
import sys

from .config import SIM_OUTPUT_ROOT
from .cleanup import delete_objects
from .db import vision_db

# Same whitelist as ground_truth.py: the name reaches a filesystem path, so it
# is validated rather than escaped.
_SAFE_AREA = re.compile(r"^[A-Za-z0-9_-]{1,64}$")

# Everything a flight writes into sim/output/<area>/. Listed rather than
# rm -rf'd so an unrecognised file (notes, a moved-in fixture) survives and the
# folder is only removed once it is genuinely empty.
_CAPTURE_RE = re.compile(r"^(frame|snap)_\d+\.png$")
_CAPTURE_FILES = ("poses.json", "flight_log.csv")

FIXTURE = "occupancy_results.json"

# Windows consoles default to cp1252 and this prints em-dashes; a
# UnicodeEncodeError here would abort a delete half-way (same fix as
# sim_uplink.py and the tools/ data scripts).
for _stream in (sys.stdout, sys.stderr):  # the refusal message goes to stderr
    if hasattr(_stream, "reconfigure"):
        _stream.reconfigure(encoding="utf-8", errors="replace")


def areas(conn):
    """Survey areas that currently have anything stored, with row counts."""
    totals: dict[str, list[int]] = {}
    with conn.cursor() as cur:
        cur.execute("SELECT survey_area, count(*) FROM frame GROUP BY 1")
        for area, n in cur.fetchall():
            totals.setdefault(area, [0, 0])[0] = n
        cur.execute("SELECT survey_area, count(*) FROM observation GROUP BY 1")
        for area, n in cur.fetchall():
            totals.setdefault(area, [0, 0])[1] = n
    conn.rollback()
    return totals


def clear_db(conn, survey_area):
    """Drop an area's frames, observations, bay states and missions.

    One transaction: a half-cleared area is worse than an uncleared one, because
    the frame ledger and the vote history would then disagree about what has
    been seen. Objects are deleted first (see cleanup.py — an object with no row
    is unreachable garbage, a row with no object is a case the classify threads
    already handle).
    """
    counts = {}
    with conn.cursor() as cur:
        cur.execute(
            "SELECT image_uri FROM frame WHERE survey_area = %s", (survey_area,)
        )
        uris = [u for (u,) in cur.fetchall()]
    counts["objects"] = delete_objects(uris)

    with conn.cursor() as cur:
        # Resolve the affected bays BEFORE the observations that name them go.
        cur.execute(
            """DELETE FROM bay_state
                WHERE bay_id IN (SELECT DISTINCT bay_id FROM observation
                                  WHERE survey_area = %s)""",
            (survey_area,),
        )
        counts["bay_state"] = cur.rowcount
        # ...and the announcements of those states. bay_delta is the WebSocket
        # replay log, so a client reconnecting with an old ?since= would
        # otherwise be told about bays this command just deleted. It has no
        # survey_area column (a delta is about a bay, not a flight), so the
        # cleared area's bays are named directly -- while the observations that
        # name them are still here to resolve.
        cur.execute(
            """DELETE FROM bay_delta
                WHERE payload->>'bay_id' IN (SELECT DISTINCT bay_id FROM observation
                                              WHERE survey_area = %s)""",
            (survey_area,),
        )
        counts["bay_delta"] = cur.rowcount
        cur.execute("DELETE FROM observation WHERE survey_area = %s", (survey_area,))
        counts["observation"] = cur.rowcount
        # The curb-run layer's three tables (migration 0014), for the same reason
        # as the bay ones: leaving run_state behind would keep publishing a free
        # count derived from evidence this command just deleted, and leaving the
        # evidence behind would let the next flight's recompute count the old
        # flight's cars. run_state is resolved from the evidence, so it goes first.
        cur.execute(
            """DELETE FROM run_state
                WHERE world = %s
                   OR run_id IN (SELECT DISTINCT run_id FROM run_observation
                                  WHERE world = %s)""",
            (survey_area, survey_area),
        )
        counts["run_state"] = cur.rowcount
        cur.execute("DELETE FROM run_detection WHERE world = %s", (survey_area,))
        counts["run_detection"] = cur.rowcount
        cur.execute("DELETE FROM run_observation WHERE world = %s", (survey_area,))
        counts["run_observation"] = cur.rowcount
        cur.execute("DELETE FROM frame WHERE survey_area = %s", (survey_area,))
        counts["frame"] = cur.rowcount  # CASCADE also takes frame_job
        # Missions are referenced by frame, so they can only go once it is empty.
        cur.execute("DELETE FROM mission WHERE survey_area = %s", (survey_area,))
        counts["mission"] = cur.rowcount
    conn.commit()
    return counts


def clear_orphans(conn):
    """Sweep state that no longer has evidence behind it. Only for --all.

    `clear_db` resolves which `bay_state` rows to drop by looking up the bays an
    area OBSERVED, which is right per-area but leaves two kinds of row behind:
    one whose observations were already gone (a state restored by hand, or an
    area cleared twice), and one the dev toggle wrote, which never had an
    observation at all. Either paints a colour on a map that nothing in the
    database can explain -- the exact confusion --all exists to end.

    Deliberately NOT run for a single area: there, a bay_state row belonging to
    some other flight is not an orphan, it is that flight's answer.
    """
    counts = {}
    with conn.cursor() as cur:
        cur.execute("DELETE FROM bay_state")
        counts["bay_state"] = cur.rowcount
        cur.execute("DELETE FROM bay_delta")
        counts["bay_delta"] = cur.rowcount
        cur.execute("DELETE FROM run_state")
        counts["run_state"] = cur.rowcount
        cur.execute("DELETE FROM run_detection")
        counts["run_detection"] = cur.rowcount
        cur.execute("DELETE FROM run_observation")
        counts["run_observation"] = cur.rowcount
    conn.commit()
    return counts


def clear_disk(survey_area, force=False):
    """Delete the flight's captures. Returns (files_deleted, note)."""
    out_dir = os.path.join(SIM_OUTPUT_ROOT, survey_area)
    if not os.path.isdir(out_dir):
        return 0, f"no capture folder at {out_dir}"
    if os.path.isfile(os.path.join(out_dir, FIXTURE)) and not force:
        raise SystemExit(
            f"refusing to clear {out_dir}:\n"
            f"  it holds {FIXTURE} — a scored golden fixture, and these frames are\n"
            f"  what that result was computed from (replay.py checks against it).\n"
            f"  Move it aside, or pass --force if you really mean to drop it."
        )

    deleted = 0
    locked = []
    for name in os.listdir(out_dir):
        if _CAPTURE_RE.match(name) or name in _CAPTURE_FILES:
            try:
                os.remove(os.path.join(out_dir, name))
            except OSError:
                # On Windows an open handle makes the file undeletable rather
                # than deleting it on close, and `flight_log.csv` is held open
                # for the whole flight (it is line-buffered telemetry). Reporting
                # it and moving on beats aborting: the database rows -- which are
                # what the map reads -- have still gone, and a stale capture only
                # matters to the controller's resume, which is what the name
                # tells the caller to go and check.
                locked.append(name)
                continue
            deleted += 1
    note = ""
    if locked:
        note = (f"{len(locked)} file(s) in use, not deleted: {', '.join(locked[:3])}"
                + (" ..." if len(locked) > 3 else "")
                + " — stop the flight/uplink holding them")
    elif not os.listdir(out_dir):
        os.rmdir(out_dir)
        note = "folder removed"
    return deleted, note


def _confirm(what):
    if not sys.stdin.isatty():
        raise SystemExit("refusing to delete unattended — pass --yes")
    print(f"about to delete {what}")
    return input("type 'yes' to continue: ").strip().lower() == "yes"


def main() -> None:
    args = sys.argv[1:]
    flags = {a for a in args if a.startswith("-")}
    names = [a for a in args if not a.startswith("-")]
    unknown = flags - {"--db-only", "--disk-only", "--force", "--yes", "-y", "--list",
                       "--all", "-h", "--help"}
    if unknown:
        raise SystemExit(f"unknown flag(s): {' '.join(sorted(unknown))} (try --help)")
    if "-h" in flags or "--help" in flags:
        print(__doc__)
        print("Flags: --db-only  --disk-only  --force  --yes  --list  --all")
        return

    # --all: every stored survey area, because a per-area clear is NOT a blank
    # map. `bay_state` has no survey_area column -- a bay is a bay -- so clearing
    # one area leaves every other area's verdicts painted on the same block, and
    # this project's areas deliberately overlap (fmi_block, fmi_block_4st and
    # dji_0035 all cover the FMI block). Flying a freshly cleared world therefore
    # opens onto the previous flight's colours, which looks exactly like the new
    # flight having already finished.
    if "--all" in flags:
        if names:
            raise SystemExit("--all takes no survey area name")
        conn = vision_db.connect()
        try:
            names = sorted(areas(conn))
        finally:
            conn.close()
        if names:
            print(f"clearing ALL {len(names)} stored survey area(s): {', '.join(names)}")
        else:
            # NOT an early return: "no area has frames or observations" is not the
            # same as "the map is blank". A bay_state row written by the dev
            # toggle, or left by an area cleared twice, has no evidence behind it
            # and no area to be found under -- so it survives exactly here, and
            # the sweep below is the only thing that removes it.
            print("no survey area has frames or observations — sweeping leftovers")

    conn = None
    # `--all` has already resolved (or deliberately emptied) `names`, and an
    # empty list there means "sweep the leftovers", not "show me the usage".
    if "--all" not in flags and ("--list" in flags or not names):
        conn = vision_db.connect()
        totals = areas(conn)
        conn.close()
        if not totals:
            print("nothing stored — no survey area has frames or observations")
        else:
            print("stored survey areas:")
            for area, (frames, obs) in sorted(totals.items()):
                print(f"  {area:<24} {frames:>5} frames  {obs:>6} observations")
        if not names:
            print("\nusage: python -m parkdrone_vision.clear_area <survey_area> [--db-only]"
                  " [--disk-only] [--force] [--yes]")
            # A bare call is "show me what is stored", not a usage error — it is
            # the first thing you run when a re-flight is not showing up.
            return

    do_db = "--disk-only" not in flags
    do_disk = "--db-only" not in flags
    force = "--force" in flags
    assume_yes = "--yes" in flags or "-y" in flags

    for survey_area in names:
        if not _SAFE_AREA.match(survey_area):
            raise SystemExit(f"not a valid survey area name: {survey_area!r}")

        scope = " and ".join(
            ([f"all database rows for '{survey_area}'"] if do_db else [])
            + ([f"the captures in sim/output/{survey_area}/"] if do_disk else [])
        )
        if not assume_yes and not _confirm(scope):
            print("aborted")
            continue

        # Disk first: it is the check that can refuse (a golden fixture), and
        # refusing after the rows are already gone would leave a half-done job.
        if do_disk:
            try:
                deleted, note = clear_disk(survey_area, force)
            except SystemExit as refused:
                # A scored golden fixture. For ONE named area that refusal is the
                # whole answer -- you asked for this area and it must not go. But
                # --all means "give me a blank map", and a map is database state:
                # aborting the sweep there would leave the areas after this one
                # painted, which is the exact half-done result --all exists to
                # avoid. So keep the fixture's frames, clear its rows, carry on.
                if "--all" not in flags:
                    raise
                print(f"  disk: kept (scored golden fixture) — {survey_area}")
                deleted, note = 0, None
            else:
                print(f"  disk: {deleted} capture file(s) deleted"
                      + (f" — {note}" if note else ""))
        if do_db:
            conn = vision_db.connect()
            try:
                c = clear_db(conn, survey_area)
            finally:
                conn.close()
            print(f"  db:   {c['frame']} frames (+jobs), {c['observation']} observations, "
                  f"{c['bay_state']} bay states, {c['bay_delta']} deltas, "
                  f"{c['mission']} missions, {c['objects']} stored images")

        print(f"cleared '{survey_area}' — re-fly it and the map will repaint live.")
        if do_db and not do_disk:
            print("  note: a sim_uplink already running keeps the frame indices it has sent;"
                  "\n        restart it to re-post them (it only resets itself when"
                  " poses.json does).")

    if "--all" in flags and do_db:
        conn = vision_db.connect()
        try:
            o = clear_orphans(conn)
        finally:
            conn.close()
        left = ", ".join(f"{k} {v}" for k, v in o.items() if v)
        print("swept state with no evidence behind it"
              + (f": {left}" if left else " (nothing left over)"))
        print("the map is now blank — every bay reports occupied:null until a "
              "flight lands.")


if __name__ == "__main__":
    main()
