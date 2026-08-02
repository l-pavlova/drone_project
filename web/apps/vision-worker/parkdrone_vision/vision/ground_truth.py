"""Eval-only ground-truth labels for a survey area.

The server classifies every visible bay because production has no ground truth
to compare against — `observation.gt` is there for the sim and eval runs, where
the world generator knows exactly which bays it parked a car in. This module is
the one place that turns a survey-area name into those labels, so accuracy can
be measured on the SAME path production uses, rather than only in the offline
golden test.

Where they come from: `sim/worlds/<area>.ground_truth.json` (or the legacy
`ground_truth.json` for the default `fmi_block` world), written by
`generate_world.py` as `{bay_id: occupied}`. A deployment that has no sim worlds
beside it simply finds nothing and records `gt = NULL`, which is the correct
answer there — accuracy is then reported as unknown, not as zero.
"""
import json
import os
import re
import threading

from ..config import GROUND_TRUTH_ROOT

# survey_area arrives from the drone over HTTP and is about to be used in a file
# path, so it is whitelisted rather than escaped: anything that is not a plain
# world name is treated as "no labels".
_SAFE_AREA = re.compile(r"^[A-Za-z0-9_-]{1,64}$")

_lock = threading.Lock()
_cache: dict[str, dict[str, bool] | None] = {}


def labels_for(survey_area: str) -> dict[str, bool] | None:
    """`{bay_id: occupied}` for an area, or None if it has no labels.

    Cached — including the misses, which is the production case and must not
    re-stat the filesystem on every frame. Bay ids are normalised to strings to
    match the rest of the web tier (they are ints in block_bays.geojson).
    """
    with _lock:
        if survey_area in _cache:
            return _cache[survey_area]

    labels = _load(survey_area)
    with _lock:
        _cache[survey_area] = labels
    return labels


def _load(survey_area: str) -> dict[str, bool] | None:
    if not _SAFE_AREA.match(survey_area or ""):
        return None
    candidates = [f"{survey_area}.ground_truth.json"]
    if survey_area == "fmi_block":
        # the default world predates per-area file sets and kept the bare name
        candidates.append("ground_truth.json")
    for name in candidates:
        path = os.path.join(GROUND_TRUTH_ROOT, name)
        if not os.path.isfile(path):
            continue
        try:
            with open(path, encoding="utf-8") as fh:
                raw = json.load(fh)
        except (OSError, ValueError) as exc:
            print(f"! ground truth {name} unreadable: {exc}")
            return None
        return {str(k): bool(v) for k, v in raw.items()}
    return None
