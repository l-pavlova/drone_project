"""Wipe a survey area so it can be flown and scored again from scratch.

    python -m parkdrone_vision.clear_area <survey_area> [flags]
    (from web/:  pnpm clear <survey_area>)

**Why this exists.** Ingest is idempotent on `(drone_id, survey_area,
frame_idx)`, so re-flying an area that is already in the database posts frames
that come back 200-duplicate: no classify job, no `bay_state` change, and
therefore no WebSocket delta. The map just sits there while the drone flies,
and nothing in the logs looks broken — the uplink says "already ingested" once
and then goes quiet. Clearing the area first is the fix, and doing it by hand
means three DELETEs in the right order plus the object store plus the capture
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
        cur.execute("DELETE FROM observation WHERE survey_area = %s", (survey_area,))
        counts["observation"] = cur.rowcount
        cur.execute("DELETE FROM frame WHERE survey_area = %s", (survey_area,))
        counts["frame"] = cur.rowcount  # CASCADE also takes frame_job
        # Missions are referenced by frame, so they can only go once it is empty.
        cur.execute("DELETE FROM mission WHERE survey_area = %s", (survey_area,))
        counts["mission"] = cur.rowcount
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
    for name in os.listdir(out_dir):
        if _CAPTURE_RE.match(name) or name in _CAPTURE_FILES:
            os.remove(os.path.join(out_dir, name))
            deleted += 1
    note = ""
    if not os.listdir(out_dir):
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
    unknown = flags - {"--db-only", "--disk-only", "--force", "--yes", "-y", "--list", "-h", "--help"}
    if unknown:
        raise SystemExit(f"unknown flag(s): {' '.join(sorted(unknown))} (try --help)")
    if "-h" in flags or "--help" in flags:
        print(__doc__)
        print("Flags: --db-only  --disk-only  --force  --yes  --list")
        return

    conn = None
    if "--list" in flags or not names:
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
            deleted, note = clear_disk(survey_area, force)
            print(f"  disk: {deleted} capture file(s) deleted" + (f" — {note}" if note else ""))
        if do_db:
            conn = vision_db.connect()
            try:
                c = clear_db(conn, survey_area)
            finally:
                conn.close()
            print(f"  db:   {c['frame']} frames (+jobs), {c['observation']} observations, "
                  f"{c['bay_state']} bay states, {c['mission']} missions, "
                  f"{c['objects']} stored images")

        print(f"cleared '{survey_area}' — re-fly it and the map will repaint live.")
        if do_db and not do_disk:
            print("  note: a sim_uplink already running keeps the frame indices it has sent;"
                  "\n        restart it to re-post them (it only resets itself when"
                  " poses.json does).")


if __name__ == "__main__":
    main()
