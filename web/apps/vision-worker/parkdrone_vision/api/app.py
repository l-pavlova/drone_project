"""PARKDRONE server — one FastAPI process owning the web edge and the vision CV.

  ingest (auth + S3 + frame row) ─┐
                                  ├─ jobs.enqueue → in-process queue → classify
  read  (PostGIS GeoJSON/summary) │                 threads (jobs.py) → hub push
  dev toggle ─────────────────────┘
  route proxy ── OSRM driving directions to a free bay (routing.py)
  WS /ws/occupancy ── hub fan-out to browsers

Read/ingest/dev handlers are sync `def`, so Starlette runs them in its
threadpool and blocking psycopg2/boto3 never touch the event loop. Only the
WebSocket endpoint is async.
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
    Query,
    UploadFile,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.responses import JSONResponse

from .. import routing, s3
from ..config import API_PORT, CLASSIFY_THREADS, ENABLE_DEV_ROUTES
from ..db import vision_db, web_db
from ..db.pool import borrow, close_pool, init_pool
from ..processing import jobs
from ..vision.scoring import pose_idx
from .auth import require_drone
from .hub import Hub

# FMI block origin — matches the ENU ORIGIN used across the project.
FMI = {"lon": 23.3298956, "lat": 42.6747105}

# Process-wide singletons built in lifespan. They are reached through the
# dependency providers below rather than `app.state`, which FastAPI discourages
# ("for most of the cases you would instead use FastAPI dependencies") — that
# way a handler declares what it needs and a test can swap it via
# app.dependency_overrides. The classify threads don't go through either: they
# receive hub/loop as plain arguments from start_workers.
_hub: Hub | None = None
_loop: asyncio.AbstractEventLoop | None = None


def get_hub() -> Hub:
    if _hub is None:  # only reachable if lifespan never ran
        raise RuntimeError("hub not initialised")
    return _hub


def get_loop() -> asyncio.AbstractEventLoop:
    if _loop is None:
        raise RuntimeError("event loop not captured")
    return _loop


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _hub, _loop

    init_pool()
    conn = vision_db.connect()
    bays = vision_db.load_bays_enu(conn)
    conn.close()

    loop = _loop = asyncio.get_running_loop()
    hub = _hub = Hub()

    # crash recovery: rebuild the in-memory queue from unscored frame rows
    rconn = vision_db.connect()
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
    survey_area = body.get("survey_area")
    if not survey_area:
        raise HTTPException(status_code=400, detail="survey_area required")
    mission_id = str(uuid.uuid4())
    with borrow(commit=True) as conn:
        web_db.insert_mission(
            conn, mission_id, drone_id, survey_area,
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

    survey_area = meta_obj["survey_area"]
    # Normalize the pose once, here at the edge: everything downstream (the
    # frame row, the job payload, the rebuild in unscored_frames) then speaks
    # the canonical `frame_idx` and needs no fallback of its own. Drone builds
    # older than 2026-08-01 post the legacy `i`.
    pose = dict(meta_obj["pose"])
    pose["frame_idx"] = pose_idx(pose)
    pose.pop("i", None)
    mission_id = meta_obj.get("mission_id")

    # Store the bytes first, then the frame row, so a committed row always has
    # an image behind it for the classify threads to fetch.
    key = f"{survey_area}/{drone_id}/frame_{pose['frame_idx']:03d}.png"
    image_uri = s3.put_frame(key, frame.file.read())

    frame_id = str(uuid.uuid4())
    with borrow(commit=True) as conn:
        fid, duplicate = web_db.insert_frame(
            conn, frame_id, drone_id, mission_id, survey_area, pose, image_uri
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
            "survey_area": survey_area,
            "frame_idx": pose["frame_idx"],
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


# ---- driving directions (public) -------------------------------------------

def _lonlat(raw: str, what: str) -> tuple[float, float]:
    parts = raw.split(",")
    if len(parts) != 2:
        raise HTTPException(status_code=400, detail=f"{what} must be lon,lat")
    try:
        lon, lat = float(parts[0]), float(parts[1])
    except ValueError:
        raise HTTPException(status_code=400, detail=f"{what} must be finite numbers")
    # also rejects NaN/inf, since every comparison against them is False
    if not (-180.0 <= lon <= 180.0 and -90.0 <= lat <= 90.0):
        raise HTTPException(status_code=400, detail=f"{what} out of range")
    return lon, lat


@app.get("/api/v1/route")
def get_route(origin: str = Query(..., alias="from"), to: str = Query(...)):
    """Road route from the driver to a bay, proxied to OSRM (see routing.py).

    Both params are `lon,lat` (GeoJSON order, like the rest of the API). Sync
    `def`, so the blocking urllib call runs in Starlette's threadpool.
    """
    flon, flat = _lonlat(origin, "from")
    tlon, tlat = _lonlat(to, "to")
    try:
        return routing.driving_route(flon, flat, tlon, tlat)
    except routing.RoutingError as exc:
        # 502 is expected and handled: the map falls back to a straight line
        raise HTTPException(status_code=502, detail=str(exc))


# ---- dev/test manual occupancy toggle --------------------------------------

def _set_occupancy(bay_id: str | None, occupied: bool, hub: Hub, loop):
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
    # sync handler on a threadpool thread -> the loop's only thread-safe door
    asyncio.run_coroutine_threadsafe(hub.broadcast([delta]), loop).result()
    return {"bay_id": target, "occupied": occupied, "updated_at": ua}


if ENABLE_DEV_ROUTES:
    @app.post("/api/v1/dev/occupy")
    def dev_occupy(
        bay_id: str | None = None,
        hub: Hub = Depends(get_hub),
        loop: asyncio.AbstractEventLoop = Depends(get_loop),
    ):
        return _set_occupancy(bay_id, True, hub, loop)

    @app.post("/api/v1/dev/free")
    def dev_free(
        bay_id: str | None = None,
        hub: Hub = Depends(get_hub),
        loop: asyncio.AbstractEventLoop = Depends(get_loop),
    ):
        return _set_occupancy(bay_id, False, hub, loop)

    print("dev routes enabled: POST /api/v1/dev/occupy | /free")


# ---- realtime occupancy push ----------------------------------------------

@app.websocket("/ws/occupancy")
async def ws_occupancy(ws: WebSocket, hub: Hub = Depends(get_hub)):
    await ws.accept()
    raw = ws.query_params.get("since")
    since = int(raw) if raw and raw.isdigit() else None
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
