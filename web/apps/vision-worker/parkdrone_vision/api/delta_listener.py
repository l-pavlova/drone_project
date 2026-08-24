"""LISTEN for bay deltas published by ANY replica and hand them to this hub.

The other half of processing/deltas.py. One daemon thread, one dedicated
autocommit connection, blocking in select() on the connection's socket — so it
costs a thread and nothing else while idle.

Two details that are easy to get wrong:

* **The connection must be autocommit.** LISTEN inside an open transaction only
  starts receiving once that transaction commits, and psycopg2 opens one
  implicitly on the first statement — the listener would go quiet forever.
* **A replica hears its OWN deltas back through here**, and that is deliberate.
  Publishing locally as well would be a hair faster and would create two
  delivery paths that can interleave differently for a local client than for a
  remote one. One path means every client sees the same order.
"""
import asyncio
import json
import select
import threading

import psycopg2.extensions

from ..db import vision_db
from ..processing.deltas import CHANNEL

# How long select() blocks before looping. Only bounds how quickly the thread
# notices a dead connection; notifications wake it immediately.
_TIMEOUT_S = 5.0


def start_thread(hub, loop) -> threading.Thread:
    t = threading.Thread(target=_listen_loop, args=(hub, loop), daemon=True)
    t.start()
    return t


def _connect():
    conn = vision_db.connect()
    conn.set_isolation_level(psycopg2.extensions.ISOLATION_LEVEL_AUTOCOMMIT)
    with conn.cursor() as cur:
        cur.execute(f"LISTEN {CHANNEL}")
    return conn


def _listen_loop(hub, loop) -> None:
    conn = None
    while True:
        try:
            if conn is None:
                conn = _connect()
            # A psycopg2 connection is a SOCKET, so select() works here on
            # Windows too — it is plain files that are not selectable there.
            if select.select([conn], [], [], _TIMEOUT_S)[0]:
                conn.poll()
                while conn.notifies:
                    note = conn.notifies.pop(0)
                    try:
                        msg = json.loads(note.payload)
                    except ValueError:
                        continue
                    # thread -> event loop: the only thread-safe door in. Wait
                    # on the result so a fan-out error surfaces here instead of
                    # disappearing into a discarded future.
                    asyncio.run_coroutine_threadsafe(hub.deliver(msg), loop).result()
        except Exception as exc:
            # A dropped connection must not end the listener: without it this
            # replica's clients silently stop updating while everything else
            # looks healthy.
            print(f"! delta listener reconnecting: {exc}")
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass
            conn = None
            threading.Event().wait(1.0)
