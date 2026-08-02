"""End-to-end harness: replay a sim survey area's frames through the LIVE server
(ingest -> in-process queue -> classify threads -> WebSocket push) and assert the
pushed deltas + final /bays state match the offline result. Exercises the same
HTTP/WS contract the drone and the dashboard use.

    API_KEY=<key> python -m parkdrone_vision.replay_ingest [survey_area] [api_base]
"""
import asyncio
import json
import os
import sys
import urllib.error
import urllib.request
import uuid

from .config import SIM_OUTPUT_ROOT
from .vision.scoring import pose_idx


def _post_json(url, obj, api_key=None):
    req = urllib.request.Request(url, data=json.dumps(obj).encode(), method="POST")
    req.add_header("content-type", "application/json")
    if api_key:
        req.add_header("x-api-key", api_key)
    with urllib.request.urlopen(req) as resp:
        return json.loads(resp.read())


def _get_json(url):
    with urllib.request.urlopen(url) as resp:
        return json.loads(resp.read())


def _post_frame(url, api_key, png, meta_str, filename):
    boundary = uuid.uuid4().hex
    body = (
        f'--{boundary}\r\nContent-Disposition: form-data; name="meta"\r\n\r\n'
        f"{meta_str}\r\n"
    ).encode()
    body += (
        f'--{boundary}\r\nContent-Disposition: form-data; name="frame"; '
        f'filename="{filename}"\r\nContent-Type: image/png\r\n\r\n'
    ).encode()
    body += png + b"\r\n" + f"--{boundary}--\r\n".encode()
    req = urllib.request.Request(url, data=body, method="POST")
    req.add_header("Content-Type", f"multipart/form-data; boundary={boundary}")
    req.add_header("x-api-key", api_key)
    try:
        with urllib.request.urlopen(req) as resp:
            return resp.status
    except urllib.error.HTTPError as e:
        return e.code


async def main() -> None:
    survey_area = sys.argv[1] if len(sys.argv) > 1 else "fmi_block"
    api_base = sys.argv[2] if len(sys.argv) > 2 else "http://localhost:4000"
    api_key = os.environ.get("API_KEY")
    if not api_key:
        raise SystemExit("set API_KEY (from register_drone)")
    drone_id = os.environ.get("DRONE_ID", "drone-1")
    out_dir = os.path.join(SIM_OUTPUT_ROOT, survey_area)

    with open(os.path.join(out_dir, "poses.json"), encoding="utf-8") as f:
        poses = json.load(f)
    with open(os.path.join(out_dir, "occupancy_results.json"), encoding="utf-8") as f:
        expected = json.load(f)["results"]

    loop = asyncio.get_running_loop()

    # 1) collect pushed deltas over the WebSocket
    import websockets

    received: dict[str, bool] = {}
    stop = asyncio.Event()

    async def collector():
        async with websockets.connect(api_base.replace("http", "ws") + "/ws/occupancy") as ws:
            while not stop.is_set():
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
                except asyncio.TimeoutError:
                    continue
                msg = json.loads(raw)
                if msg.get("type") == "bay_delta":
                    received[msg["bay_id"]] = msg["occupied"]

    task = asyncio.create_task(collector())
    await asyncio.sleep(0.5)
    print("ws connected")

    # 2) start a mission
    mission = await loop.run_in_executor(
        None, _post_json, f"{api_base}/api/v1/ingest/mission/start",
        {"survey_area": survey_area, "area": "verification", "frames_expected": len(poses)}, api_key,
    )
    print("mission", mission["mission_id"])

    # 3) stream every frame through the ingest endpoint
    for pose in poses:
        i = pose_idx(pose)
        with open(os.path.join(out_dir, f"frame_{i:03d}.png"), "rb") as f:
            png = f.read()
        meta = json.dumps(
            {"drone_id": drone_id, "survey_area": survey_area, "mission_id": mission["mission_id"], "pose": pose}
        )
        status = await loop.run_in_executor(
            None, _post_frame, f"{api_base}/api/v1/ingest/frame", api_key, png, meta, f"frame_{i}.png"
        )
        if status != 202:
            print(f"frame {i}: HTTP {status}")
    print(f"posted {len(poses)} frames")

    # 4) wait for the queue to drain + deltas to settle (stable for 2s)
    prev, stable = -1, 0
    for _ in range(40):
        await asyncio.sleep(1.0)
        if received and len(received) == prev:
            stable += 1
            if stable >= 2:
                break
        else:
            stable = 0
        prev = len(received)

    await loop.run_in_executor(
        None, _post_json, f"{api_base}/api/v1/ingest/mission/{mission['mission_id']}/end", {}, api_key
    )
    stop.set()
    await task

    # 5) assert final /bays state matches the offline predictions
    fc = await loop.run_in_executor(None, _get_json, f"{api_base}/api/v1/bays")
    state = {str(f["properties"]["bay_id"]): f["properties"]["occupied"] for f in fc["features"]}
    match = mismatch = 0
    bad = []
    for bay_id, r in expected.items():
        if state.get(bay_id) == r["pred"]:
            match += 1
        else:
            mismatch += 1
            bad.append(f"{bay_id}: offline={r['pred']} live={state.get(bay_id)}")

    print(f"\nWebSocket deltas received: {len(received)}")
    print(f"final /bays vs occupancy_results: match={match} mismatch={mismatch}")
    if bad:
        print("  " + "\n  ".join(bad[:20]))
    sys.exit(1 if mismatch else 0)


if __name__ == "__main__":
    asyncio.run(main())
