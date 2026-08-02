"""Drone API-key auth as a FastAPI dependency.

Reads `x-api-key`, matches its SHA-256 against drone.api_key_hash, bumps
last_seen, and yields the drone_id. 401 on missing key, 403 on unknown key.
"""
import hashlib

from fastapi import Header, HTTPException

from ..db import web_db
from ..db.pool import borrow


def hash_api_key(key: str) -> str:
    return hashlib.sha256(key.encode()).hexdigest()


def require_drone(x_api_key: str | None = Header(default=None)) -> str:
    if not x_api_key:
        raise HTTPException(status_code=401, detail="missing x-api-key")
    with borrow(commit=True) as conn:
        drone_id = web_db.lookup_drone(conn, hash_api_key(x_api_key))
        if not drone_id:
            raise HTTPException(status_code=403, detail="invalid api key")
        web_db.bump_last_seen(conn, drone_id)
    return drone_id
