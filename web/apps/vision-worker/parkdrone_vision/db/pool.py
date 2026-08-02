"""A small threaded psycopg2 connection pool for the web edge.

The read/ingest FastAPI handlers are declared as sync `def`, so Starlette runs
them in its anyio threadpool — blocking psycopg2 calls never touch the event
loop. Each such thread borrows a connection from this pool for the duration of
one request. (The classify threads, by contrast, each own a *dedicated*
long-lived connection via db.connect(); they are not part of this pool.)
"""
from contextlib import contextmanager

from psycopg2.pool import ThreadedConnectionPool

from ..config import required

_pool: ThreadedConnectionPool | None = None


def init_pool(minconn: int = 1, maxconn: int = 10) -> ThreadedConnectionPool:
    global _pool
    if _pool is None:
        _pool = ThreadedConnectionPool(
            minconn, maxconn, dsn=required("DATABASE_URL")
        )
    return _pool


def close_pool() -> None:
    global _pool
    if _pool is not None:
        _pool.closeall()
        _pool = None


@contextmanager
def borrow(commit: bool = False):
    """Yield a pooled connection, returning it afterwards.

    commit=True commits on clean exit; any exception rolls back. Read handlers
    leave commit False (autocommit-style SELECTs still need the tx closed, so we
    rollback to release locks/snapshots cleanly).
    """
    assert _pool is not None, "pool not initialised"
    conn = _pool.getconn()
    try:
        yield conn
        if commit:
            conn.commit()
        else:
            conn.rollback()
    except Exception:
        conn.rollback()
        raise
    finally:
        _pool.putconn(conn)
