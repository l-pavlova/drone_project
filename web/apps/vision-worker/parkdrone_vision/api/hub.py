"""WebSocket fan-out hub.

The classifier runs in this same process, so deltas are pushed straight to
connected browsers. `broadcast` is a coroutine on the server's event loop; the
classify threads and the sync dev routes reach it via
asyncio.run_coroutine_threadsafe(...). A small bounded replay buffer lets a
reconnecting client catch up with ?since=<cursor>.
"""
import asyncio
import json
from collections import deque

REPLAY_MAX = 500


class Hub:
    def __init__(self, replay_max: int = REPLAY_MAX):
        self._clients: set = set()
        self._replay: deque = deque(maxlen=replay_max)
        self._cursor = 0
        # A reconnect must see every delta after its cursor.  Serialising
        # connect + broadcast closes the otherwise tiny gap between replaying
        # the buffer and registering the new client.
        self._lock = asyncio.Lock()

    async def connect(self, ws, since: int | None = None) -> None:
        async with self._lock:
            if since:
                for cursor, msg in list(self._replay):
                    if cursor > since:
                        await self._safe_send(ws, msg)
            await self._safe_send(ws, {"type": "snapshot_cursor", "cursor": str(self._cursor)})
            self._clients.add(ws)

    def disconnect(self, ws) -> None:
        self._clients.discard(ws)

    def client_count(self) -> int:
        return len(self._clients)

    async def broadcast(self, deltas: list[dict]) -> None:
        """Fan out bay deltas. Each is tagged {type:"bay_delta", ...} to match
        the frontend's useOccupancySocket."""
        async with self._lock:
            dead = []
            for delta in deltas:
                self._cursor += 1
                # Include the cursor on every event, not just at connection
                # time.  The browser can then reconnect from the precise last
                # event it applied instead of silently losing an outage window.
                msg = {
                    **(delta if delta.get("type") else {"type": "bay_delta", **delta}),
                    "cursor": str(self._cursor),
                }
                self._replay.append((self._cursor, msg))
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
