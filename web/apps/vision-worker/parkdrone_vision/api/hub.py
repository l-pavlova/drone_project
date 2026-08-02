"""WebSocket fan-out hub.

The classifier runs in this same process, so deltas are pushed straight to
connected browsers. `broadcast` is a coroutine on the server's event loop; the
classify threads and the sync dev routes reach it via
asyncio.run_coroutine_threadsafe(...). A small bounded replay buffer lets a
reconnecting client catch up with ?since=<cursor>.
"""
import json
from collections import deque

REPLAY_MAX = 500


class Hub:
    def __init__(self, replay_max: int = REPLAY_MAX):
        self._clients: set = set()
        self._replay: deque = deque(maxlen=replay_max)
        self._cursor = 0

    async def connect(self, ws, since: int | None = None) -> None:
        if since:
            for cursor, msg in list(self._replay):
                if cursor > since:
                    await self._safe_send(ws, msg)
        await self._safe_send(ws, {"type": "snapshot_cursor", "cursor": str(self._cursor)})
        self._clients.add(ws)

    def disconnect(self, ws) -> None:
        self._clients.discard(ws)

    async def broadcast(self, deltas: list[dict]) -> None:
        """Fan out bay deltas. Each is tagged {type:"bay_delta", ...} to match
        the frontend's useOccupancySocket."""
        dead = []
        for delta in deltas:
            msg = delta if delta.get("type") else {"type": "bay_delta", **delta}
            self._cursor += 1
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
