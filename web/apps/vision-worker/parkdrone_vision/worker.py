"""Live worker: BRPOP frame jobs off Redis, score them, persist + publish.

Job payload (JSON) produced by the Node ingest API:
    {"frame_id","world","frame_idx","pose",{...},
     "image_uri":"s3://bucket/key"  OR  "image_path":"/local/frame.png"}

Idempotency is enforced upstream by the frame table's UNIQUE(drone_id,world,i);
re-processing a frame is harmless (observations are additive and the majority
vote is stable).
"""
import io
import json

import numpy as np
import redis
from PIL import Image

from . import db
from .config import JOBS_QUEUE, REDIS_URL, S3_BUCKET
from .pipeline import process_frame

_s3 = None


def _s3_client():
    global _s3
    if _s3 is None:
        import boto3

        from .config import S3_ACCESS_KEY, S3_ENDPOINT, S3_REGION, S3_SECRET_KEY

        _s3 = boto3.client(
            "s3",
            endpoint_url=S3_ENDPOINT,
            region_name=S3_REGION,
            aws_access_key_id=S3_ACCESS_KEY,
            aws_secret_access_key=S3_SECRET_KEY,
        )
    return _s3


def load_image(job):
    if job.get("image_path"):
        return np.array(Image.open(job["image_path"]).convert("RGB"))
    uri = job["image_uri"]  # s3://bucket/key
    assert uri.startswith("s3://")
    _, _, rest = uri.partition("s3://")
    bucket, _, key = rest.partition("/")
    obj = _s3_client().get_object(Bucket=bucket or S3_BUCKET, Key=key)
    return np.array(Image.open(io.BytesIO(obj["Body"].read())).convert("RGB"))


def main():
    r = redis.from_url(REDIS_URL)
    conn = db.connect()
    bays = db.load_bays_enu(conn)
    print(f"vision worker up: {len(bays)} bays loaded, waiting on {JOBS_QUEUE}")

    while True:
        # finite server-side block so the client socket never read-times-out;
        # None simply means "no job this window" -> keep waiting.
        try:
            item = r.brpop(JOBS_QUEUE, timeout=5)
        except redis.exceptions.TimeoutError:
            continue
        if item is None:
            continue
        _, raw = item
        job = json.loads(raw)
        try:
            img = load_image(job)
            res = process_frame(
                conn,
                job["world"],
                job["frame_idx"],
                img,
                job["pose"],
                bays,
                redis_client=r,
                frame_id=job.get("frame_id"),
            )
            print(
                f"frame {job['world']}#{job['frame_idx']}: "
                f"scored {res['scored']} bays, {len(res['deltas'])} deltas"
            )
        except Exception as exc:  # keep the loop alive; a bad frame must not stall the queue
            conn.rollback()
            print(f"! failed job {job.get('frame_id')}: {exc}")


if __name__ == "__main__":
    main()
