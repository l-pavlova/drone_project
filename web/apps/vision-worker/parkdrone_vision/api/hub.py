"""WebSocket fan-out: this replica's half of the shared delta channel.

The hub used to be the whole story — a classify thread called `broadcast`, the
hub stamped a cursor from a process-local counter, kept the message in a
process-local deque and sent it to process-local clients. All three of those
words are why a second replica was unsafe: a browser attached to replica B never
saw replica A's deltas, and `?since=` meant something different on each process
(and after every restart, since the counter began at 0).

Now the hub only DELIVERS. Ordering, replay and cross-replica fan-out belong to
`processing/deltas.py` + the `bay_delta` table:

    process_frame --publish--> bay_delta row + pg_notify
                                        |
        every replica's delta_listener --> hub.deliver() --> its own clients

So the hub holds no cursor of its own, and a message it sends was already
assigned its cursor by the database. `connect(since=...)` replays out of the
table, which is the same answer whichever replica the client lands on.
"""
import asyncio
import json


class Hub:
    def __init__(self):
        self._clients: set = set()
        # A reconnect must see every delta after its cursor. Serialising
        # connect + deliver closes the otherwise tiny gap between replaying the
        # backlog and registering the new client.
        self._lock = asyncio.Lock()

    async def connect(self, ws, replay: list | None = None, cursor: int = 0) -> None:
        """Register a client, after replaying what it missed.

        The caller does the DB work (a blocking read has no business on the
        event loop) and hands the result in; the hub's job is only to get the
        ordering right — backlog first, then the cursor, then live traffic.
        """
        async with self._lock:
            for msg in replay or []:
                await self._safe_send(ws, msg)
            await self._safe_send(ws, {"type": "snapshot_cursor", "cursor": str(cursor)})
            self._clients.add(ws)

    def disconnect(self, ws) -> None:
        self._clients.discard(ws)

    def client_count(self) -> int:
        return len(self._clients)

    async def deliver(self, msg: dict) -> None:
        """Send one already-published delta to this replica's clients.

        Verbatim: the cursor and the `type` tag were set at publish time, so a
        live message and a replayed one are the identical object.
        """
        async with self._lock:
            dead = []
            for ws in list(self._clients):
                try:
                    await ws.send_text(json.dumps(msg))
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self._clients.discard(ws)

    @staticmethod
    async def _safe_send(ws, msg) -> None:
        try:
            await ws.send_text(json.dumps(msg))
        except Exception:
            pass
