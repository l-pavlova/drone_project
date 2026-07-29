"""Register (or re-key) a drone and print a fresh API key (Python port of
apps/api/src/scripts/register-drone.ts).

    python -m parkdrone_vision.register_drone <drone_id> [name]

The plaintext key is shown ONCE; only its SHA-256 hash is stored.
"""
import secrets
import sys

from .api.auth import hash_api_key
from .db import vision_db as db


def main() -> None:
    if len(sys.argv) < 2:
        print("usage: register_drone <drone_id> [name]", file=sys.stderr)
        sys.exit(1)
    drone_id = sys.argv[1]
    name = sys.argv[2] if len(sys.argv) > 2 else drone_id
    key = secrets.token_hex(24)

    conn = db.connect()
    with conn.cursor() as cur:
        cur.execute(
            """INSERT INTO drone (drone_id, name, api_key_hash)
               VALUES (%s, %s, %s)
               ON CONFLICT (drone_id) DO UPDATE
                 SET name = EXCLUDED.name, api_key_hash = EXCLUDED.api_key_hash""",
            (drone_id, name, hash_api_key(key)),
        )
    conn.commit()
    conn.close()
    print(f"registered drone '{drone_id}'")
    print(f"API key (store securely, shown once): {key}")


if __name__ == "__main__":
    main()
