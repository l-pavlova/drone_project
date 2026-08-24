"""The shared delta channel: publish once to Postgres, fan out everywhere.

A bay delta used to go straight from the classify thread into this process's
WebSocket hub, which is exactly why a second replica was unsafe — a browser
attached to replica B never saw anything replica A produced.

Now every delta takes one path, and only one:

    process_frame ──► bay_delta row (id = the global cursor)
                 └──► pg_notify('bay_delta', payload)   [same transaction]
                                    │
              every replica's LISTEN thread ──► its own local clients

Two properties come out of that and both are the point:

* **NOTIFY is transactional.** Postgres delivers it on COMMIT, so a delta is
  never announced for state that then rolls back — no ordering rule to get
  right, no window where the wire is ahead of the database.
* **A replica hears its OWN deltas back through the listener.** It would be a
  microsecond faster to also push locally at publish time, but then two
  delivery paths exist, and they can interleave differently for a local client
  than for a remote one. One path means every client sees the same order.
"""
import json

CHANNEL = "bay_delta"


def publish(conn, deltas):
    """Record and announce deltas. Call INSIDE the caller's transaction.

    Returns the assigned cursors. The row is the replay buffer as well as the
    ordering: a client reconnecting to any replica resumes from bay_delta.id,
    which is one sequence shared by all of them — unlike the old per-process
    counter, which restarted at 0 on every boot.
    """
    if not deltas:
        return []
    cursors = []
    with conn.cursor() as cur:
        for delta in deltas:
            # Tag it here rather than in the hub so what is stored is byte-for-
            # byte what goes on the wire, cursor included. One statement: draw
            # the id, fold it into the payload, insert -- so a replayed message
            # and a live one are the identical object. (The client keys its
            # resume point off `cursor`; a replayed message missing one would
            # silently stall the cursor.)
            msg = delta if delta.get("type") else {"type": "bay_delta", **delta}
            cur.execute(
                """WITH n AS (
                       SELECT nextval(pg_get_serial_sequence('bay_delta','id')) AS id
                   )
                   INSERT INTO bay_delta (id, payload)
                   SELECT n.id, %s::jsonb || jsonb_build_object('cursor', n.id::text)
                     FROM n
                   RETURNING id, payload""",
                (json.dumps(msg),),
            )
            cursor, payload = cur.fetchone()
            # pg_notify rather than the NOTIFY statement: the channel and the
            # payload are bind parameters, not text spliced into SQL.
            cur.execute("SELECT pg_notify(%s, %s)", (CHANNEL, json.dumps(payload)))
            cursors.append(cursor)
    return cursors


def replay(conn, since, slack, limit):
    """Deltas a reconnecting client missed, oldest first.

    Deliberately replays a little BEHIND `since`. Sequence ids are handed out
    before commit, so with concurrent writers id 100 can become visible after
    101; a strict `id > since` would then skip it forever. A delta is an
    idempotent "set bay X to this state", so re-sending a handful costs nothing,
    while missing one leaves a bay stale on the map until the next REST refresh.
    """
    with conn.cursor() as cur:
        cur.execute(
            """SELECT payload FROM bay_delta
                WHERE id > GREATEST(%(since)s - %(slack)s, 0)
                ORDER BY id LIMIT %(limit)s""",
            {"since": since, "slack": slack, "limit": limit},
        )
        return [row[0] for row in cur.fetchall()]


def head(conn):
    """The newest cursor — what a fresh client resumes from."""
    with conn.cursor() as cur:
        cur.execute("SELECT COALESCE(MAX(id), 0) FROM bay_delta")
        return int(cur.fetchone()[0])
