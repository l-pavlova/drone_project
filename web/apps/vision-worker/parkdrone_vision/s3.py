"""Object-store access for frames (MinIO / S3).

The FastAPI ingest handler writes frame bytes here (`put_frame`) and stores only
the returned s3:// uri in the frame row; the classify threads read them back
(`get_frame_array`). Lifted out of the old worker.py so both sides share one
client. A 1-day expiry lifecycle rule on the bucket keeps only the latest run
per world (see infra/docker-compose.yml).
"""
import io

import numpy as np
from PIL import Image

from .config import (
    S3_ACCESS_KEY,
    S3_BUCKET,
    S3_ENDPOINT,
    S3_REGION,
    S3_SECRET_KEY,
)

_client = None


def client():
    global _client
    if _client is None:
        import boto3

        _client = boto3.client(
            "s3",
            endpoint_url=S3_ENDPOINT,
            region_name=S3_REGION,
            aws_access_key_id=S3_ACCESS_KEY,
            aws_secret_access_key=S3_SECRET_KEY,
        )
    return _client


def put_frame(key: str, body: bytes, content_type: str = "image/png") -> str:
    """Store a frame image; return the s3:// uri the classify threads will fetch."""
    client().put_object(
        Bucket=S3_BUCKET, Key=key, Body=body, ContentType=content_type
    )
    return f"s3://{S3_BUCKET}/{key}"


def get_frame_array(uri_or_path: str) -> np.ndarray:
    """Load a frame as an HxWx3 RGB array from an s3:// uri or a local path."""
    if not uri_or_path.startswith("s3://"):
        return np.array(Image.open(uri_or_path).convert("RGB"))
    _, _, rest = uri_or_path.partition("s3://")
    bucket, _, key = rest.partition("/")
    obj = client().get_object(Bucket=bucket or S3_BUCKET, Key=key)
    return np.array(Image.open(io.BytesIO(obj["Body"].read())).convert("RGB"))
