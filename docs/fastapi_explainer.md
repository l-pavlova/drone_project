# How FastAPI works (using our `parkdrone_vision` server as the example)

FastAPI's core idea: **you write a plain Python function, decorate it, and the type
hints on its parameters tell FastAPI what to pull out of the HTTP request and
validate for you.** Below walks through our actual `web/apps/vision-worker/parkdrone_vision/api/app.py`
since it exercises most of the framework's moving parts.

## The app object and startup/shutdown

```python
app = FastAPI(lifespan=lifespan)
```

Everything hangs off one `app` object. `lifespan` (`app.py:42-65`) is a special async
generator: code before `yield` runs once at startup, code after runs once at
shutdown. Ours uses it to open the DB pool, load bays into memory, and spin up the
classify-thread pool — the kind of one-time setup you don't want repeated per-request.

## A route is just a decorated function

```python
@app.get("/health")
def health():
    return {"ok": True}
```

`@app.get("/health")` registers this function to handle `GET /health`. Whatever you
`return` — a dict here — gets JSON-serialized automatically. No manual `res.json()`
like Express; the return value *is* the response.

## Parameters come from type hints, based on where you declare them

This is the part that feels like magic until you see the pattern:

- **Path params** — `@app.get("/api/v1/bays/{bay_id}")` / `def get_bay(bay_id: str)`:
  FastAPI matches `bay_id` in the function signature to `{bay_id}` in the path.
- **Query params** — `def get_bays(bbox: str | None = None, zona: str | None = None)`:
  any parameter that *isn't* in the path and isn't a special type becomes a
  `?bbox=...&zona=...` query parameter automatically. The `| None = None` makes it
  optional.
- **Request body (form/file)** — in `ingest_frame` (`app.py:99-104`):
  ```python
  def ingest_frame(
      frame: UploadFile = File(...),
      meta: str = Form(...),
      drone_id: str = Depends(require_drone),
  ):
  ```
  `File(...)` and `Form(...)` tell FastAPI these come from a multipart form body,
  not JSON — matching how the drone actually POSTs a PNG plus a JSON string field.
- **Headers/auth** — in `auth.py`:
  `def require_drone(x_api_key: str | None = Header(default=None))` — same trick,
  but for the `x-api-key` header.

If a required field is missing or the wrong type, FastAPI returns a
`422 Unprocessable Entity` before your function body even runs. You get input
validation for free just by writing type hints.

## `Depends()` — dependency injection

```python
drone_id: str = Depends(require_drone)
```

This is FastAPI's signature feature. `require_drone` (`auth.py:18`) is itself an
ordinary function that reads the API key header, looks it up, and either raises
`HTTPException(401/403)` or returns a `drone_id` string. Any route that wants "an
authenticated drone" just adds this one parameter — FastAPI calls `require_drone`
first, and either the request never reaches your handler (auth failed) or
`drone_id` shows up already resolved. It's how you share cross-cutting logic (auth,
in this case) without decorators or middleware boilerplate.

## Raising errors

```python
raise HTTPException(status_code=400, detail="survey_area required")
```

You don't `return` errors — you `raise` them, from anywhere, including inside a
`Depends` function, and FastAPI catches it and turns it into the right HTTP
response.

## Sync `def` vs `async def` — this one matters for our app specifically

FastAPI/Starlette lets a route be either `def` or `async def`. Our read/ingest/dev
handlers are plain `def` on purpose (see the `app.py` module docstring) — Starlette
runs sync functions in a background threadpool automatically, so a blocking
`psycopg2` call doesn't freeze the server. Only `ws_occupancy` is `async def`,
because it needs to sit on the event loop awaiting messages without blocking
anything else. Get that distinction wrong (e.g. a blocking `async def` handler) and
every other request stalls behind it.

## WebSockets

```python
@app.websocket("/ws/occupancy")
async def ws_occupancy(ws: WebSocket):
    await ws.accept()
    ...
    await ws.receive_text()
```

Same decorator pattern, just a different primitive (`WebSocket` instead of a normal
request/response). `await ws.receive_text()` in a loop is really just "block here
until the client disconnects" — the server never expects the browser to send
anything, it's using the receive call purely to detect `WebSocketDisconnect`.

## The best part for learning it: free interactive docs

Because every route's parameters are typed, FastAPI generates an OpenAPI schema
automatically and serves a UI at **`/docs`**. With the server running
(`python -m parkdrone_vision.server`), open **http://localhost:4000/docs** — a full
interactive UI listing every route from `app.py`, its expected params, and a
"Try it out" button that fires real requests (handy for testing the `dev/occupy`
endpoint by hand instead of `curl`). That page is generated entirely from the type
hints already on the routes — nothing extra to write.
