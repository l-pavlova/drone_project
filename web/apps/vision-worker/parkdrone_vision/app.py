"""PARKDRONE server — a single FastAPI process that replaces the Node API, the
separate vision worker, and Redis.

  ingest (auth + S3 + frame row) ─┐
                                  ├─ jobs.enqueue → in-process queue → classify
  read  (PostGIS GeoJSON/summary) │                 threads (jobs.py) → hub push
  dev toggle ─────────────────────┘
  WS /ws/occupancy ── hub fan-out to browsers

Read/ingest/dev handlers are sync `def`, so Starlette runs them in its
threadpool and blocking psycopg2/boto3 never touch the event loop. Only the
WebSocket endpoint is async. Contract is identical to the retired Node tier, so
the React app and operator scripts are unchanged.
"""
import asyncio
import json
import uuid
from contextlib import asynccontextmanager

from fastapi import (
    Depends,
    FastAPI,
    File,
    Form,
    HTTPException,
    UploadFile,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.responses import JSONResponse

from . import db, jobs, s3, web_db
from .auth import require_drone
from .config import API_PORT, CLASSIFY_THREADS, ENABLE_DEV_ROUTES
from .hub import Hub
from .pool import borrow, close_pool, init_pool

# FMI block origin — matches the ENU ORIGIN used across the project (and dev.ts).
FMI = {"lon": 23.3298956, "lat": 42.6747105}


@asynccontextmanager
async def lifespan(app: FastAPI):
    init_pool()
    conn = db.connect()
    bays = db.load_bays_enu(conn)
    conn.close()

    loop = asyncio.get_running_loop()
    hub = Hub()
    app.state.hub = hub
    app.state.loop = loop

    # crash recovery: rebuild the in-memory queue from unscored frame rows
    rconn = db.connect()
    recovered = jobs.recover(rconn)
    rconn.close()

    jobs.start_workers(CLASSIFY_THREADS, bays, hub, loop)
    print(
        f"parkdrone server on :{API_PORT} — {len(bays)} bays, "
        f"{CLASSIFY_THREADS} classify threads, recovered {recovered} queued frames"
    )
    yield
    close_pool()


app = FastAPI(lifespan=lifespan)


@app.get("/health")
def health():
    return {"ok": True}


# ---- ingest (auth-guarded) -------------------------------------------------

@app.post("/api/v1/ingest/mission/start", status_code=201)
def mission_start(body: dict, drone_id: str = Depends(require_drone)):
    world = body.get("world")
    if not world:
        raise HTTPException(status_code=400, detail="world required")
    mission_id = str(uuid.uuid4())
    with borrow(commit=True) as conn:
        web_db.insert_mission(
            conn, mission_id, drone_id, world,
            body.get("area"), body.get("frames_expected"),
        )
    return {"mission_id": mission_id}


@app.post("/api/v1/ingest/mission/{mission_id}/end")
def mission_end(mission_id: str, drone_id: str = Depends(require_drone)):
    with borrow(commit=True) as conn:
        web_db.end_mission(conn, mission_id, drone_id)
    return {"ok": True}


@app.post("/api/v1/ingest/frame", status_code=202)
def ingest_frame(
    frame: UploadFile = File(...),
    meta: str = Form(...),
    drone_id: str = Depends(require_drone),
):
    try:
        meta_obj = json.loads(meta)
    except Exception:
        raise HTTPException(status_code=400, detail="meta must be JSON")
    if meta_obj.get("drone_id") != drone_id:
        raise HTTPException(status_code=403, detail="drone_id mismatch")

    world = meta_obj["world"]
    pose = meta_obj["pose"]
    mission_id = meta_obj.get("mission_id")

    # Store bytes first (matches the retired Node order), then the frame row.
    key = f"{world}/{drone_id}/frame_{pose['i']:03d}.png"
    image_uri = s3.put_frame(key, frame.file.read())

    frame_id = str(uuid.uuid4())
    with borrow(commit=True) as conn:
        fid, duplicate = web_db.insert_frame(
            conn, frame_id, drone_id, mission_id, world, pose, image_uri
        )
        if duplicate:
            return JSONResponse(
                status_code=200, content={"frame_id": fid, "duplicate": True}
            )
        if mission_id:
            web_db.bump_mission_done(conn, mission_id)

    jobs.enqueue(
        {
            "frame_id": frame_id,
            "world": world,
            "frame_idx": pose["i"],
            "pose": pose,
            "image_uri": image_uri,
        }
    )
    return {"frame_id": frame_id}


# ---- reads (public) --------------------------------------------------------

@app.get("/api/v1/bays")
def get_bays(bbox: str | None = None, zona: str | None = None):
    b = None
    if bbox:
        parts = bbox.split(",")
        if len(parts) != 4:
            raise HTTPException(status_code=400, detail="bbox must be minLon,minLat,maxLon,maxLat")
        try:
            nums = [float(p) for p in parts]
        except ValueError:
            raise HTTPException(status_code=400, detail="bbox must be finite numbers")
        b = {"minLon": nums[0], "minLat": nums[1], "maxLon": nums[2], "maxLat": nums[3]}
    with borrow() as conn:
        return web_db.feature_collection(conn, b, zona)


@app.get("/api/v1/summary")
def get_summary():
    with borrow() as conn:
        return {"zones": web_db.summary(conn)}


@app.get("/api/v1/bays/{bay_id}")
def get_bay(bay_id: str):
    with borrow() as conn:
        d = web_db.detail(conn, bay_id)
    if d is None:
        raise HTTPException(status_code=404, detail="bay not found")
    return d


# ---- dev/test manual occupancy toggle --------------------------------------

def _set_occupancy(bay_id: str | None, occupied: bool):
    with borrow(commit=True) as conn:
        if bay_id and bay_id.strip():
            target = bay_id.strip() if web_db.bay_exists(conn, bay_id.strip()) else None
        else:
            target = web_db.nearest_bay(conn, FMI["lon"], FMI["lat"])
        if not target:
            raise HTTPException(status_code=404, detail="no matching bay")
        updated_at = web_db.upsert_state(conn, target, occupied, 1, None, "manual")
    ua = updated_at.isoformat()
    delta = {"bay_id": target, "occupied": occupied, "confidence": 1, "updated_at": ua}
    asyncio.run_coroutine_threadsafe(app.state.hub.broadcast([delta]), app.state.loop).result()
    return {"bay_id": target, "occupied": occupied, "updated_at": ua}


if ENABLE_DEV_ROUTES:
    @app.post("/api/v1/dev/occupy")
    def dev_occupy(bay_id: str | None = None):
        return _set_occupancy(bay_id, True)

    @app.post("/api/v1/dev/free")
    def dev_free(bay_id: str | None = None):
        return _set_occupancy(bay_id, False)

    print("dev routes enabled: POST /api/v1/dev/occupy | /free")


# ---- realtime occupancy push ----------------------------------------------

@app.websocket("/ws/occupancy")
async def ws_occupancy(ws: WebSocket):
    await ws.accept()
    raw = ws.query_params.get("since")
    since = int(raw) if raw and raw.isdigit() else None
    hub: Hub = app.state.hub
    await hub.connect(ws, since)
    try:
        while True:
            await ws.receive_text()  # client sends nothing; this just detects close
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        hub.disconnect(ws)
