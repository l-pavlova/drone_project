"""Minimal client for the drone-facing ingest API.

Shared by the verification harness (`replay_ingest`) and the live sim uplink
(`sim_uplink`) so the multipart body is built in exactly ONE place — it is the
wire format the real drone firmware will have to reproduce, and two hand-rolled
copies would drift.

stdlib `urllib` only, deliberately: these tools run wherever the sim runs
(including under Webots' bundled Python, which has no site-packages of ours),
and the project already avoids adding a request library for that reason.
"""
import json
import urllib.error
import urllib.request
import uuid


def post_json(url, obj, api_key=None, timeout=30):
    req = urllib.request.Request(url, data=json.dumps(obj).encode(), method="POST")
    req.add_header("content-type", "application/json")
    if api_key:
        req.add_header("x-api-key", api_key)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read())


def get_json(url, timeout=30):
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return json.loads(resp.read())


def post_frame(url, api_key, png, meta_str, filename, timeout=60):
    """POST one frame as multipart/form-data. Returns the HTTP status.

    202 = accepted and queued, 200 = duplicate (already ingested for this
    drone/survey area/frame index, so nothing was re-enqueued). Both are
    successes; the caller decides what to make of a duplicate.
    """
    boundary = uuid.uuid4().hex
    body = (
        f'--{boundary}\r\nContent-Disposition: form-data; name="meta"\r\n\r\n'
        f"{meta_str}\r\n"
    ).encode()
    body += (
        f'--{boundary}\r\nContent-Disposition: form-data; name="frame"; '
        f'filename="{filename}"\r\nContent-Type: image/png\r\n\r\n'
    ).encode()
    body += png + b"\r\n" + f"--{boundary}--\r\n".encode()
    req = urllib.request.Request(url, data=body, method="POST")
    req.add_header("Content-Type", f"multipart/form-data; boundary={boundary}")
    req.add_header("x-api-key", api_key)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status
    except urllib.error.HTTPError as e:
        return e.code
