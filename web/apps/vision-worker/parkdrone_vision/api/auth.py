"""API-key auth dependencies: per-drone for ingest, shared-secret for admin.

Reads `x-api-key`, matches its SHA-256 against drone.api_key_hash, bumps
last_seen, and yields the drone_id. 401 on missing key, 403 on unknown key.
"""
import hashlib
import hmac

from fastapi import Header, HTTPException

from ..config import ADMIN_API_KEY

from ..db import web_db
from ..db.pool import borrow


def hash_api_key(key: str) -> str:
    return hashlib.sha256(key.encode()).hexdigest()


def require_admin(x_admin_key: str | None = Header(default=None)) -> None:
    """Gate the operational endpoints on ADMIN_API_KEY, if one is configured.

    Compared with compare_digest rather than `==`: the difference is
    unobservable here in practice (the timing signal is buried under the HTTP
    round trip), but a key comparison that is written the wrong way tends to get
    copied into somewhere it does matter.

    With no key configured this is a no-op, so local dev and the existing
    quickstart keep working unchanged - and server.py prints a warning at
    startup so "unset" cannot pass silently for "secured".
    """
    if not ADMIN_API_KEY:
        return
    if not x_admin_key:
        raise HTTPException(status_code=401, detail="missing x-admin-key")
    if not hmac.compare_digest(x_admin_key, ADMIN_API_KEY):
        raise HTTPException(status_code=403, detail="invalid admin key")


def require_drone(x_api_key: str | None = Header(default=None)) -> str:
    if not x_api_key:
        raise HTTPException(status_code=401, detail="missing x-api-key")
    with borrow(commit=True) as conn:
        drone_id = web_db.lookup_drone(conn, hash_api_key(x_api_key))
        if not drone_id:
            raise HTTPException(status_code=403, detail="invalid api key")
        web_db.bump_last_seen(conn, drone_id)
    return drone_id
