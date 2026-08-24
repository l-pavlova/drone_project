"""PARKDRONE server — one FastAPI process owning the web edge and the vision CV.

  ingest (auth + S3 + frame row) ─┐   frame_job row ─claim─► classify threads
                                  ├─  (jobs.py; ANY replica may claim it)
  read  (PostGIS GeoJSON/summary) │        │
  dev toggle ─────────────────────┘   bay_delta + NOTIFY ─► every replica's hub
  route proxy ── OSRM driving directions to a free bay (routing.py)
  metrics ────── pipeline/ingest/fleet health, JSON + Prometheus (metrics.py)
  WS /ws/occupancy ── hub fan-out to browsers

Read/ingest/dev handlers are sync `def`, so Starlette runs them in its
threadpool and blocking psycopg2/boto3 never touch the event loop. Only the
WebSocket endpoint is async.
"""
import asyncio
import json
import os
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
from fastapi.responses import JSONResponse, PlainTextResponse

from .. import cleanup, config, nofly, routing, s3
from ..config import (
    ADMIN_API_KEY,
    API_PORT,
    CLASSIFY_THREADS,
    CLEANUP_INTERVAL_S,
    ENABLE_DEV_ROUTES,
)
from ..db import vision_db, web_db
from ..db.pool import borrow, close_pool, init_pool
from ..processing import deltas, jobs, pipeline
from ..vision.scoring import pose_idx
from . import delta_listener, metrics
from .auth import require_admin, require_drone
from .hub import Hub

# FMI block origin — matches the ENU ORIGIN used across the project.
FMI = {"lon": 23.3298956, "lat": 42.6747105}

# Process-wide singletons built in lifespan. They are reached through the
# dependency providers below rather than `app.state`, which FastAPI discourages
# ("for most of the cases you would instead use FastAPI dependencies") — that
# way a handler declares what it needs and a test can swap it via
# app.dependency_overrides. The classify threads never touch the hub at all any
# more -- they publish to the shared delta channel and the listener thread
# (which does get hub/loop as plain arguments) delivers.
_hub: Hub | None = None
_loop: asyncio.AbstractEventLoop | None = None


def get_hub() -> Hub:
    if _hub is None:  # only reachable if lifespan never ran
        raise RuntimeError("hub not initialised")
    return _hub


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _hub, _loop

    init_pool()
    conn = vision_db.connect()
    bays, bay_index = vision_db.load_bays_enu(conn)
    conn.close()

    loop = _loop = asyncio.get_running_loop()
    hub = _hub = Hub()

    # No crash-recovery step any more: unfinished work is CLAIMED (migration
    # 0011), so it is picked up by the dispatcher's ordinary poll — by this
    # replica or by any other that is already running. Report the backlog rather
    # than claiming it, so the banner does not lie about who will do it.
    rconn = vision_db.connect()
    backlog = web_db.claimable_count(rconn, config.MAX_ATTEMPTS)
    rconn.close()

    jobs.start_workers(CLASSIFY_THREADS, bays, bay_index)
    # Deltas reach this replica's clients only through the shared channel, so
    # the listener is not optional: without it the map never moves.
    delta_listener.start_thread(hub, loop)
    sweeping = cleanup.start_thread()
    print(
        f"parkdrone server on :{API_PORT} — {len(bays)} bays, "
        f"{CLASSIFY_THREADS} classify threads, replica {config.REPLICA_ID}, "
        f"{backlog} frames claimable, "
        f"frame cleanup {'every %ds' % CLEANUP_INTERVAL_S if sweeping else 'disabled'}"
    )
    # Say it out loud rather than let an unset key look like a secured one. The
    # metrics endpoints expose queue depth, ingest rates, fleet state and model
    # accuracy - an operational map of the system, which is a different thing
    # from a bay's occupancy being public.
    print("admin metrics require x-admin-key (/api/v1/metrics, /metrics)"
          if ADMIN_API_KEY else
          "! admin metrics are UNAUTHENTICATED (/api/v1/metrics, /metrics) - "
          "set ADMIN_API_KEY to require x-admin-key")
    yield
    close_pool()


app = FastAPI(lifespan=lifespan)


@app.get("/health")
def health():
    # `occupancy_window_s` is served here rather than left for the client to
    # guess: the browser applies its own staleness TTL when painting bays, and
    # a second hard-coded number is a bug waiting to happen -- it used to be 10
    # minutes against a 2 h server window, so a landed survey greyed out while
    # the API still considered it current. One authority, published.
    return {"ok": True,
            "occupancy_window_s": config.OCCUPANCY_WINDOW_S,
            "occupancy_backend": pipeline.backend_name()}


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
    # frame row, the job payload rebuilt by claim_frames) then speaks
    # the canonical `frame_idx` and needs no fallback of its own. Drone builds
    # older than 2026-08-01 post the legacy `i`.
    pose = dict(meta_obj["pose"])
    pose["frame_idx"] = pose_idx(pose)
    pose.pop("i", None)
    mission_id = meta_obj.get("mission_id")

    # Store the bytes first, then the frame row, so a committed row always has
    # an image behind it for the classify threads to fetch.
    # The mission is part of the key, not decoration: since 0009 a re-flight of
    # an area ingests instead of being dropped as a duplicate, so a key without
    # it would overwrite the earlier flight's image while that flight's frame row
    # still points at it -- the row would then serve a different flight's pixels
    # to recovery and to cleanup. Mission-less frames keep the old key exactly.
    # Keep the uploaded file's extension. PIL sniffs the format from the bytes
    # so a JPEG under a .png key would still decode, but real DJI stills are
    # JPEG and a key that lies about its content is a trap for anyone who later
    # pulls one out of the bucket by hand.
    ext = os.path.splitext(frame.filename or "")[1].lower()
    if ext not in (".png", ".jpg", ".jpeg"):
        ext = ".png"
    key = (f"{survey_area}/{drone_id}/"
           + (f"{mission_id}/" if mission_id else "")
           + f"frame_{pose['frame_idx']:03d}{ext}")
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

    # The frame_job row committed above IS the queue entry (any replica can
    # claim it); this only spares the dispatcher its poll interval.
    jobs.wake()
    return {"frame_id": frame_id}


# ---- reads (public) --------------------------------------------------------

def _bbox(raw: str | None) -> dict | None:
    """`minLon,minLat,maxLon,maxLat` -> the dict shape the readers take."""
    if not raw:
        return None
    parts = raw.split(",")
    if len(parts) != 4:
        raise HTTPException(status_code=400, detail="bbox must be minLon,minLat,maxLon,maxLat")
    try:
        nums = [float(p) for p in parts]
    except ValueError:
        raise HTTPException(status_code=400, detail="bbox must be finite numbers")
    return {"minLon": nums[0], "minLat": nums[1], "maxLon": nums[2], "maxLat": nums[3]}


@app.get("/api/v1/bays")
def get_bays(bbox: str | None = None, zona: str | None = None):
    with borrow() as conn:
        return web_db.feature_collection(conn, _bbox(bbox), zona)


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


# ---- UAS no-fly zones (public) ---------------------------------------------

_RESTRICTIONS = {"PROHIBITED", "REQ_AUTHORISATION", "CONDITIONAL"}


@app.get("/api/v1/nofly")
def get_nofly(bbox: str | None = None, restriction: str | None = None):
    """Published UAS geographical zones as GeoJSON (see nofly.py).

    Static reference data, so no DB round trip — it is parsed once and served
    from memory. Pass `bbox` to get only what the viewport needs; the full
    national set is 881 zones and no client wants all of it.
    """
    if restriction is not None and restriction not in _RESTRICTIONS:
        raise HTTPException(
            status_code=400,
            detail=f"restriction must be one of {sorted(_RESTRICTIONS)}",
        )
    return nofly.feature_collection(_bbox(bbox), restriction)


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


# ---- operational metrics ---------------------------------------------------
#
# The admin dashboard's data source (and P7's observability hook). See
# metrics.py for what is measured in-process vs. read from Postgres. Sync `def`
# for the same reason as the other read routes: psycopg2 blocks.

@app.get("/api/v1/metrics")
def get_metrics(
    window_s: int = Query(metrics.DEFAULT_WINDOW_S, ge=1, le=86400),
    hub: Hub = Depends(get_hub),
    _admin: None = Depends(require_admin),
):
    with borrow() as conn:
        return metrics.snapshot(conn, window_s, hub)


@app.get("/metrics", response_class=PlainTextResponse)
def get_metrics_prometheus(hub: Hub = Depends(get_hub),
                           _admin: None = Depends(require_admin)):
    """Same snapshot in Prometheus exposition format, on the conventional path."""
    with borrow() as conn:
        snap = metrics.snapshot(conn, metrics.DEFAULT_WINDOW_S, hub)
    return PlainTextResponse(
        metrics.prometheus(snap), media_type="text/plain; version=0.0.4; charset=utf-8"
    )


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
        # Same channel as a classified delta, in the same transaction as the
        # state it announces -- so a manual toggle reaches every replica's
        # clients and lands in the replay log, exactly like a real verdict.
        deltas.publish(conn, [{"bay_id": target, "occupied": occupied,
                               "confidence": 1, "updated_at": ua}])
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

def _replay_since(since: int):
    """(missed deltas, current head cursor) for a (re)connecting client."""
    with borrow() as conn:
        cursor = deltas.head(conn)
        missed = deltas.replay(conn, since, config.DELTA_REPLAY_SLACK,
                               config.DELTA_REPLAY_MAX) if since else []
    return missed, cursor


@app.websocket("/ws/occupancy")
async def ws_occupancy(ws: WebSocket, hub: Hub = Depends(get_hub)):
    await ws.accept()
    raw = ws.query_params.get("since")
    since = int(raw) if raw and raw.isdigit() else 0
    # Replay comes out of bay_delta, so it reads the same on every replica --
    # the point of the exercise. Blocking DB work goes to a worker thread: this
    # is the one async handler in the app and the event loop must not stall on
    # it while other clients are receiving deltas.
    replay, cursor = await asyncio.to_thread(_replay_since, since)
    await hub.connect(ws, replay, cursor)
    try:
        while True:
            await ws.receive_text()  # client sends nothing; this just detects close
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        hub.disconnect(ws)
