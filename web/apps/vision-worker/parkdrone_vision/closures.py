"""Load street closures into the DB, and lift them — with a live delta each way.

    python -m parkdrone_vision.closures load [path]   # default data/closures.geojson
    python -m parkdrone_vision.closures list [--all]
    python -m parkdrone_vision.closures end <closure_id>
    python -m parkdrone_vision.closures rm  <closure_id>

A closure is the first authoritative fact in this system that is NOT derived
from the camera (TODO #13). It overrides the published answer for every bay it
covers: an empty bay on a closed street is not available parking, however
clearly the detector saw that it was empty.

ONE SOURCE FILE, TWO CONSUMERS. `data/closures.geojson` is the same file
`sim/generate_world.py --closures` reads to drop closed streets from the flight
route, so the sim and the web tier cannot disagree about what is closed. That is
deliberate: a closure that the drone skips but the map still advertises (or the
reverse) is the exact failure this feature exists to prevent.

WHY A DELTA IS PUBLISHED HERE. The override is applied at READ time, so the
database needs no bay_state write for a closure to take effect — but the map is
push-driven, and a browser already holding the old answer would keep showing it
until the next reload. Publishing through the normal channel repaints it at
once, on every replica, using the machinery that already exists.

WHAT THIS DOES NOT DO: a closure that expires on its OWN CLOCK pushes no delta,
because nothing runs at that instant to notice. Such a bay corrects itself on
the client's next fetch or reconnect. That is the same, already-accepted
limitation as a bay_state row going stale under OCCUPANCY_WINDOW_S, which also
pushes nothing when it crosses the window.
"""
import json
import os
import sys

from .db import vision_db as db
from .db import web_db
from .processing import deltas as delta_channel

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_FILE = os.path.normpath(
    os.path.join(HERE, "..", "..", "..", "..", "data", "closures.geojson"))


def _publish_for(conn, bay_ids):
    """Announce that these bays' published answer changed.

    The delta carries the bays' CURRENT state, re-read after the closure write:
    a delta is an idempotent "set bay X to this", so the client ends up correct
    whether it applies this one or refetches. `closed` is what actually moved.
    """
    if not bay_ids:
        return []
    with conn.cursor() as cur:
        cur.execute(
            f"""SELECT b.bay_id,
                       CASE WHEN {web_db._FRESH} THEN s.occupied END,
                       CASE WHEN {web_db._FRESH} THEN s.confidence END,
                       s.updated_at,
                       {web_db._CLOSED}
                  FROM bay b LEFT JOIN bay_state s USING (bay_id)
                 WHERE b.bay_id = ANY(%(ids)s)""",
            {"ids": list(bay_ids), "window_s": web_db.OCCUPANCY_WINDOW_S},
        )
        rows = cur.fetchall()
    msgs = [{"bay_id": bid, "occupied": occ, "confidence": conf,
             "updated_at": upd.isoformat() if upd else None, "closed": closed}
            for (bid, occ, conf, upd, closed) in rows]
    return delta_channel.publish(conn, msgs)


def load(path):
    if not os.path.exists(path):
        sys.exit(f"no such file: {path}")
    feats = json.load(open(path, encoding="utf-8"))["features"]
    conn = db.connect()
    touched, n = set(), 0
    with conn.cursor() as cur:
        for f in feats:
            pr = f["properties"]
            cid = pr.get("id")
            if not cid or f["geometry"]["type"] != "Polygon":
                print(f"  skipped (needs an id and a Polygon): {cid!r}")
                continue
            cur.execute(
                """INSERT INTO street_closure
                     (closure_id, label, reason, geom, valid_from, valid_to, source)
                   VALUES (%s, %s, %s,
                           ST_SetSRID(ST_GeomFromGeoJSON(%s), 4326), %s, %s, %s)
                   ON CONFLICT (closure_id) DO UPDATE SET
                     label = EXCLUDED.label, reason = EXCLUDED.reason,
                     geom = EXCLUDED.geom, valid_from = EXCLUDED.valid_from,
                     valid_to = EXCLUDED.valid_to, source = EXCLUDED.source""",
                (cid, pr.get("street"), pr.get("reason"),
                 json.dumps(f["geometry"]), pr.get("valid_from"),
                 pr.get("valid_to"), pr.get("source")),
            )
            n += 1
        # Bay lookup AFTER the writes, in the same transaction, so a re-shaped
        # closure announces the bays it covers now (and, because the geometry is
        # already updated, a bay it no longer covers is announced as open again
        # only if it is still in the set below - see the note in `end`).
        for f in feats:
            cid = f["properties"].get("id")
            if cid:
                touched.update(web_db.bays_in_closure(conn, cid))
        cursors = _publish_for(conn, touched)
    conn.commit()
    conn.close()
    print(f"loaded {n} closure(s) from {os.path.basename(path)}; "
          f"{len(touched)} bay(s) affected, {len(cursors)} delta(s) published")


def end(closure_id):
    """Lift a closure NOW by closing its validity window (keeps the history)."""
    conn = db.connect()
    # Bays are resolved BEFORE the update: after it, the closure is inactive and
    # ST_Intersects would still match but the validity test would not, so the
    # set is the same either way here — but resolving first is the rule that
    # also holds for `rm`, where the row is gone afterwards.
    bays = web_db.bays_in_closure(conn, closure_id)
    with conn.cursor() as cur:
        cur.execute(
            "UPDATE street_closure SET valid_to = now() WHERE closure_id = %s",
            (closure_id,),
        )
        if cur.rowcount == 0:
            conn.close()
            sys.exit(f"no such closure: {closure_id}")
        cursors = _publish_for(conn, bays)
    conn.commit()
    conn.close()
    print(f"ended {closure_id}; {len(bays)} bay(s) reopened, "
          f"{len(cursors)} delta(s) published")


def remove(closure_id):
    conn = db.connect()
    bays = web_db.bays_in_closure(conn, closure_id)
    with conn.cursor() as cur:
        cur.execute("DELETE FROM street_closure WHERE closure_id = %s", (closure_id,))
        if cur.rowcount == 0:
            conn.close()
            sys.exit(f"no such closure: {closure_id}")
        cursors = _publish_for(conn, bays)
    conn.commit()
    conn.close()
    print(f"deleted {closure_id}; {len(bays)} bay(s) reopened, "
          f"{len(cursors)} delta(s) published")


def show(active_only):
    conn = db.connect()
    fc = web_db.closures(conn, None, active_only=active_only)
    conn.close()
    if not fc["features"]:
        print("no closures" + (" active" if active_only else ""))
        return
    for f in fc["features"]:
        p = f["properties"]
        print(f"  {p['closure_id']:<28} {p.get('label') or '?':<20} "
              f"{p['bays']:>4} bays   {p.get('valid_from') or '-'} -> "
              f"{p.get('valid_to') or 'open'}   {p.get('reason') or ''}")


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")   # street names are Cyrillic
    argv = sys.argv[1:]
    cmd = argv[0] if argv else "list"
    if cmd == "load":
        load(argv[1] if len(argv) > 1 else DEFAULT_FILE)
    elif cmd == "list":
        show(active_only="--all" not in argv)
    elif cmd == "end" and len(argv) > 1:
        end(argv[1])
    elif cmd == "rm" and len(argv) > 1:
        remove(argv[1])
    else:
        print(__doc__.split("\n\n")[1].strip(), file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
